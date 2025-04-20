/*
 * PIUIO HID interface driver – overrides generic Linux HID by matching PIUIO boards
 * and provides legacy /dev/piuio0 char-device compatibility for existing apps.
 *
 * Copyright (C) 2012‑2014 Devin J. Pohly
 * Copyright (C) 2025      Diego Acevedo <diego.acevedo.fernando@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/hid.h>
#include <linux/string.h>
#include <linux/sched.h> // For TASK_INTERRUPTIBLE

#define USB_VENDOR_ID_ANCHOR            0x0547
#define USB_PRODUCT_ID_PYTHON2          0x1002
#define USB_VENDOR_ID_BTNBOARD          0x0d2f
#define USB_PRODUCT_ID_BTNBOARD_LEGACY  0x1010 // Original/Legacy device ID
#define USB_PRODUCT_ID_BTNBOARD_NEW     0x1020 // New device ID

#define PIUIO_RPT_OUT_BASE       0x80 // Base Report ID for legacy output control SET_REPORTs
#define PIUIO_OUTPUT_CHUNK       16   // Size of data chunk for legacy output SET_REPORTs
#define PIUIO_LEGACY_SIZE        8    // Expected size for legacy /dev/piuio0 writes
#define PIUIO_NUM_INPUTS         48   // Max number of inputs expected across supported devices
#define PIUIO_MAX_LEDS           48   // Max number of LEDs expected across supported devices

// Input report size for the NEW (1020) device based on its HID descriptor
#define PIUIO_INPUT_SIZE_NEW     16

#define PIUIO_BTN_REG            BTN_JOYSTICK
#define PIUIO_BTN_EXTRA          BTN_TRIGGER_HAPPY

// HID Class Request Codes (standard defines may exist, but adding for clarity)
#ifndef HID_REQ_SET_IDLE
#define HID_REQ_SET_IDLE                0x0A
#endif

// Forward declaration
struct piuio;

// LED Structure
struct piuio_led {
    struct piuio *piu;
    struct led_classdev cdev;
    u8 idx;
};

// Main Device Structure
struct piuio {
    struct hid_device    *hdev;      // HID device structure
    struct usb_device    *udev;      // USB device structure
    struct input_dev     *idev;      // Input device structure
    u8                    iface;     // Interface number
    char                  phys[64];  // Physical path string
    unsigned long         prev_inputs[BITS_TO_LONGS(PIUIO_NUM_INPUTS)]; // Previous input state bitmap
    u8                    led_shadow[PIUIO_MAX_LEDS]; // Shadow state for LEDs
    struct piuio_led     *led;       // Array of LED structures
    struct miscdevice     misc;      // Legacy /dev/piuioX interface
    char                 *misc_name; // Allocated name for misc device
    spinlock_t            lock;      // Protects led_shadow
};

/**
 * piuio_keycode - Map raw input bit index to a kernel key/button code.
 * @pin: The zero-based index of the input bit.
 *
 * Return: The corresponding BTN_* code.
 */
static inline int piuio_keycode(unsigned pin)
{
    if (pin < (BTN_GAMEPAD - BTN_JOYSTICK))
        return PIUIO_BTN_REG + pin;
    return PIUIO_BTN_EXTRA + (pin - (BTN_GAMEPAD - BTN_JOYSTICK));
}

/**
 * piuio_set_report_legacy_ctrl - Send output report using legacy SET_REPORT control transfer.
 * @piu: Device instance data.
 * @rptid: The Report ID (typically 0x80 + bank number).
 *
 * This function attempts to set LED states using the older control transfer method.
 * WARNING: This method may be incorrect for the 0x1020 device based on its HID descriptor,
 * which defines a standard 16-byte Output report. This function is retained primarily
 * for potential compatibility with the 0x1010 device, whose protocol is unverified.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int piuio_set_report_legacy_ctrl(struct piuio *piu, u8 rptid)
{
    int bank = rptid - PIUIO_RPT_OUT_BASE;
    u8 *buf;
    int i, ret;

    // Use GFP_ATOMIC if we might be called from atomic context (like LED trigger),
    // otherwise GFP_KERNEL is fine. spin_lock context is okay for GFP_ATOMIC.
    // Let's assume GFP_KERNEL is okay for now as brightness_set might block.
    buf = kmalloc(PIUIO_OUTPUT_CHUNK + 1, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    buf[0] = rptid; // Report ID
    spin_lock(&piu->lock);
    for (i = 0; i < PIUIO_OUTPUT_CHUNK; i++) {
        int idx = bank * PIUIO_OUTPUT_CHUNK + i;
        // Ensure index is within bounds
        buf[i+1] = (idx < PIUIO_MAX_LEDS && piu->led_shadow[idx]) ? 1 : 0;
    }
    spin_unlock(&piu->lock);

    ret = usb_control_msg(
        piu->udev,
        usb_sndctrlpipe(piu->udev, 0),
        HID_REQ_SET_REPORT, // 0x09
        USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // 0x21
        (HID_OUTPUT_REPORT << 8) | rptid, // wValue (Report Type | Report ID)
        piu->iface, // wIndex (Interface)
        buf, PIUIO_OUTPUT_CHUNK + 1, // data buffer and length
        msecs_to_jiffies(1000)); // Use reasonable timeout (e.g., 1000ms)

    kfree(buf);

    if (ret < 0) {
         hid_warn(piu->hdev, "SET_REPORT(ctrl) for ID 0x%02x failed: %d\n", rptid, ret);
         return ret;
    } else if (ret != (PIUIO_OUTPUT_CHUNK + 1)) {
        hid_warn(piu->hdev, "SET_REPORT(ctrl) for ID 0x%02x transferred %d bytes (expected %d)\n", rptid, ret, PIUIO_OUTPUT_CHUNK + 1);
        // Maybe return error or maybe it's fine? Returning success for now.
    }

    return 0; // Success
}

/**
 * piuio_raw_event - Callback for processing raw HID input reports.
 * @hdev: The HID device.
 * @report: The HID report structure (may not be fully populated for raw_event).
 * @raw_data: Buffer containing the raw report data from the interrupt endpoint.
 * @size: The size of the data in raw_data.
 *
 * This function is called by the HID core when an input report is received
 * on the interrupt IN endpoint. It parses the data based on the known device
 * types and reports changes to the Linux input subsystem.
 *
 * Return: 1 if the event was handled, 0 otherwise.
 */
static int piuio_raw_event(struct hid_device *hdev, struct hid_report *report,
             u8 *raw_data, int size)
{
    struct piuio *piu = hid_get_drvdata(hdev);
    int i, max_inputs_in_report = 0;
    int inputs_to_process = 0;
    int num_longs = 0;
    bool state_changed = false;
    const unsigned long *current_data_bits = NULL; // Use const ptr

    // Determine expected format based on Product ID
    if (hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
        if (size != PIUIO_INPUT_SIZE_NEW) {
             hid_warn(hdev, "Ignoring input report for 1020 device with unexpected size: %d (expected %d)\n", size, PIUIO_INPUT_SIZE_NEW);
             return 0; // Ignore unexpected size
        }
        // Assume the 16 bytes are the input data, no offset/header.
        // This is based on the descriptor and python script observations.
        current_data_bits = (unsigned long *)raw_data;
        max_inputs_in_report = size * 8; // 128 potential inputs

    } else if (hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
        // Behavior and report format/size for the legacy device are unknown.
        // It might use a different size or transfer mechanism entirely.
        // Safest is to ignore its input reports until verified.
        hid_warn(hdev, "Received input report for legacy 1010 device (size %d) - parsing logic UNKNOWN, ignoring!\n", size);
        return 0; // Ignore

    } else {
        // Should not happen if id_table is correct
        return 0;
    }

    // Limit processing to the maximum inputs we handle (PIUIO_NUM_INPUTS)
    // or the number of bits available in the report, whichever is smaller.
    inputs_to_process = min(max_inputs_in_report, PIUIO_NUM_INPUTS);
    num_longs = BITS_TO_LONGS(inputs_to_process);

    // Compare current bits with previous state
    for (i = 0; i < num_longs; i++) {
        // Check bounds to prevent reading past the end of received data
        if (((void*)&current_data_bits[i]) >= ((void*)raw_data + size)) {
            hid_warn(hdev, "Attempted to read past input buffer boundary (long %d)\n", i);
            break; // Stop processing if we risk reading out of bounds
        }

        unsigned long current_bits = current_data_bits[i];
        unsigned long changed_bits = current_bits ^ piu->prev_inputs[i];

        if (changed_bits != 0) {
            state_changed = true;
            // Iterate over changed bits within this long
            for_each_set_bit(int bit_index, &changed_bits, BITS_PER_LONG) {
                 int global_bit_index = i * BITS_PER_LONG + bit_index;

                 // Ensure we don't process bits beyond our defined inputs
                 if (global_bit_index >= inputs_to_process) break;

                 // Determine pressed state (assumes 0 = pressed / bit cleared)
                 bool pressed = !test_bit(global_bit_index, current_data_bits);

                 // Report event to input subsystem
                 input_event(piu->idev, EV_MSC, MSC_SCAN, global_bit_index + 1); // Raw bit number
                 input_report_key(piu->idev, piuio_keycode(global_bit_index), pressed);
            }
             piu->prev_inputs[i] = current_bits; // Update previous state
        }
    }

    // Sync input events if any changes occurred
    if (state_changed) {
        input_sync(piu->idev);
    }

    return 1; // Indicate event was handled
}

/**
 * piuio_led_set - Callback function to set LED brightness.
 * @cdev: LED class device structure.
 * @b: Desired brightness (LED_OFF or LED_FULL).
 */
static void piuio_led_set(struct led_classdev *cdev,
                          enum led_brightness b)
{
    struct piuio_led *ld = container_of(cdev, struct piuio_led, cdev);
    struct piuio *piu = ld->piu;
    int ret;

    if (ld->idx >= PIUIO_MAX_LEDS) return; // Bounds check

    spin_lock(&piu->lock);
    piu->led_shadow[ld->idx] = (b > 0);
    spin_unlock(&piu->lock);

    // Send update using the legacy control transfer method.
    // WARNING: This might be incorrect for the 1020 device.
    ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (ld->idx / PIUIO_OUTPUT_CHUNK));
    // Error warning is printed within piuio_set_report_legacy_ctrl
}

/**
 * piuio_dev_write - Handle writes to the legacy /dev/piuioX device.
 * @filp: File structure pointer.
 * @buf: User space buffer containing data to write.
 * @len: Number of bytes to write (must match PIUIO_LEGACY_SIZE).
 * @off: File offset (ignored).
 *
 * Assumes the written data is a bitmap for the first N LEDs.
 *
 * Return: Number of bytes written on success, negative error code otherwise.
 */
static ssize_t piuio_dev_write(struct file *filp,
                               const char __user *buf,
                               size_t len, loff_t *off)
{
    struct piuio *piu = filp->private_data;
    u8 tmp[PIUIO_LEGACY_SIZE];
    int pin, ret;

    if (len != PIUIO_LEGACY_SIZE)
        return -EINVAL;
    if (copy_from_user(tmp, buf, len))
        return -EFAULT;

    spin_lock(&piu->lock);
    // Clear and set only the LEDs controlled by the legacy interface
    memset(piu->led_shadow, 0, PIUIO_LEGACY_SIZE * 8); // Clear only affected LEDs
    for (pin = 0; pin < PIUIO_LEGACY_SIZE*8 && pin < PIUIO_MAX_LEDS; pin++)
        piu->led_shadow[pin] = !!(tmp[pin/8] & (1 << (pin%8)));
    spin_unlock(&piu->lock);

    // Update all relevant output banks using legacy method
    // WARNING: May be incorrect for 1020 device.
    for (pin = 0; pin < PIUIO_LEGACY_SIZE*8 && pin < PIUIO_MAX_LEDS; pin += PIUIO_OUTPUT_CHUNK) {
         ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (pin / PIUIO_OUTPUT_CHUNK));
         if (ret < 0) {
              hid_warn(piu->hdev, "Legacy write failed for bank %d: %d\n", pin / PIUIO_OUTPUT_CHUNK, ret);
              // Continue trying other banks? Or return error? Sticking with original behavior.
         }
    }

    return len;
}

// File operations for the legacy /dev/piuioX device
static const struct file_operations piuio_fops = {
    .owner   = THIS_MODULE,
    .write   = piuio_dev_write,
    .open    = simple_open, // Non-exclusive open
    .llseek  = no_llseek,
};

/**
 * piuio_probe - Called when a matching HID device is found.
 * @hdev: Pointer to the HID device structure.
 * @id: The entry from piuio_ids that matched the device.
 *
 * Sets up the driver instance, input device, LEDs, and misc device.
 * Sends the required SET_IDLE command.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int piuio_probe(struct hid_device *hdev,
                       const struct hid_device_id *id)
{
    struct piuio *piu;
    struct input_dev *idev;
    struct usb_interface *intf;
    int i, ret;

    hid_info(hdev, "Probing PIUIO device %04X:%04X\n", id->vendor, id->product);

    intf = to_usb_interface(hdev->dev.parent);
    if (!intf) {
        hid_err(hdev,"Cannot get USB interface\n");
        return -ENODEV;
    }


    // Allocate device instance structure using device-managed memory
    piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
    if (!piu) return -ENOMEM;
    hid_set_drvdata(hdev, piu); // Associate our data with the hid device

    // Store pointers to HID and USB device structures
    piu->hdev = hdev;
    piu->udev = interface_to_usbdev(intf); // Get USB device from interface
    piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;
    spin_lock_init(&piu->lock);

    // --- Input device setup ---
    snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys); // Use physical path from HID device
    idev = devm_input_allocate_device(&hdev->dev); // Device-managed allocation
    if (!idev) return -ENOMEM;
    piu->idev = idev;

    // Populate input device information from HID device
    idev->name = hdev->name;
    idev->phys = piu->phys;
    idev->id.bustype = hdev->bustype;
    idev->id.vendor  = hdev->vendor;
    idev->id.product = hdev->product;
    idev->id.version = hdev->version;
    idev->dev.parent = &hdev->dev; // Link to the HID device

    // Set input event capabilities
    set_bit(EV_KEY, idev->evbit);
    for (i = 0; i < PIUIO_NUM_INPUTS; i++) // Map potential inputs to keycodes
        set_bit(piuio_keycode(i), idev->keybit);
    set_bit(EV_MSC, idev->evbit); // Enable miscellaneous events
    set_bit(MSC_SCAN, idev->mscbit); // Allow reporting raw scan codes (bit indices)

    // Register the input device with the kernel
    ret = input_register_device(idev);
    if (ret) {
        hid_err(hdev, "Failed to register input device: %d\n", ret);
        return ret; // devm_* cleanup handles allocated idev
    }

    // --- LED class device setup ---
    piu->led = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS, sizeof(*piu->led), GFP_KERNEL);
    if (!piu->led) return -ENOMEM; // devm_* cleanup handles idev

    for (i = 0; i < PIUIO_MAX_LEDS; i++) {
        piu->led[i].piu = piu;
        piu->led[i].idx = i;
        // Use unique name including device path segment
        piu->led[i].cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s::output%u", dev_name(&hdev->dev), i);
        if (!piu->led[i].cdev.name) return -ENOMEM; // devm_* cleanup handles previous LEDs/idev
        piu->led[i].cdev.brightness_set_blocking = piuio_led_set; // Use blocking version if needed
        piu->led[i].cdev.max_brightness = 1;

        // Register LED using device-managed function
        ret = devm_led_classdev_register(&hdev->dev, &piu->led[i].cdev);
        if (ret) {
            hid_err(hdev, "Failed to register LED %d: %d\n", i, ret);
            // devm_* cleanup handles previously registered LEDs/idev
            return ret;
        }
    }

    // --- Misc device setup (/dev/piuioX) ---
    piu->misc.minor  = MISC_DYNAMIC_MINOR;
    // Create a unique name like "piuio-0003:0D2F:1020.0001"
    piu->misc_name = kasprintf(GFP_KERNEL, "piuio-%s", dev_name(&hdev->dev));
    if (!piu->misc_name) return -ENOMEM; // devm_* handles input/leds
    piu->misc.name = piu->misc_name;
    piu->misc.fops   = &piuio_fops;
    piu->misc.parent = &hdev->dev; // Associate with the HID device

    ret = misc_register(&piu->misc);
    if (ret) {
        hid_err(hdev, "Failed to register misc device '%s': %d\n", piu->misc.name, ret);
        kfree(piu->misc_name); // Free name if misc_register fails
        // devm_* cleanup handles LEDs and input device automatically
        return ret;
    }
    hid_info(hdev, "Registered misc device /dev/%s\n", piu->misc.name);


    // --- Start HID Hardware Layer ---
    // Connect HIDRAW but prevent kernel's generic HID input node creation
    ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_HIDINPUT);
    if (ret) {
        hid_err(hdev, "hid_hw_start failed: %d\n", ret);
        misc_deregister(&piu->misc); // Clean up misc device manually
        kfree(piu->misc_name);
        // devm_* handles LEDs and input device
        return ret;
    }

    // --- Parse HID Reports ---
    // This helps the HID core understand the device capabilities
    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "hid_parse failed: %d\n", ret);
        hid_hw_stop(hdev); // Clean up hid_hw_start
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        // devm_* handles LEDs and input device
        return ret;
    }

    // --- Open HID Device Communication Channel ---
    // Necessary for sending/receiving reports manually (like SET_IDLE)
    // and allows HID core to manage interrupt endpoints.
    ret = hid_hw_open(hdev);
    if (ret) {
        hid_err(hdev, "hid_hw_open failed: %d\n", ret);
        hid_hw_stop(hdev);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        // devm_* handles LEDs and input device
        return ret;
    }

    // --- Send SET_IDLE Request ---
    // Required by 1020 device to start sending input reports. Might be needed by 1010 too.
    ret = usb_control_msg(piu->udev,
                          usb_sndctrlpipe(piu->udev, 0),
                          HID_REQ_SET_IDLE, // 0x0A
                          USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // 0x21
                          0, // wValue = 0 (Report ID 0, Infinite Idle)
                          piu->iface, // wIndex = Interface number
                          NULL, 0, // No data
                          msecs_to_jiffies(1000)); // Timeout 1 sec
    if (ret < 0 && ret != -EPIPE) { // Ignore Pipe error (stall) which Python script did
        hid_err(hdev, "SET_IDLE control message failed: %d\n", ret);
        hid_hw_close(hdev); // Clean up open
        hid_hw_stop(hdev);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        // devm_* handles LEDs and input device
        return ret;
    } else if (ret == -EPIPE) {
         hid_warn(hdev, "SET_IDLE stalled (pipe error), continuing anyway.\n");
    } else {
         hid_info(hdev, "SET_IDLE command sent successfully.\n");
    }

    // Timer is no longer used, input is handled by raw_event callback

    hid_info(hdev, "PIUIO HID Driver Registered successfully for %04X:%04X\n", id->vendor, id->product);
    return 0; // Success!
}


/**
 * piuio_remove - Called when the HID device is removed or driver unloaded.
 * @hdev: Pointer to the HID device structure.
 *
 * Cleans up resources allocated in probe: misc device, HID hardware state.
 * Device-managed resources (memory, input device, LEDs) are freed automatically.
 */
static void piuio_remove(struct hid_device *hdev)
{
    struct piuio *piu = hid_get_drvdata(hdev);

    hid_info(hdev, "Unregistering PIUIO HID Driver for %04X:%04X\n", hdev->vendor, hdev->product);

    // Close HID hardware connection (must be done before stop)
    hid_hw_close(hdev);

    // Stop HID hardware layer
    hid_hw_stop(hdev);

    // De-register misc device (and free its allocated name)
    misc_deregister(&piu->misc);
    kfree(piu->misc_name);

    // devm_* functions automatically clean up:
    // - Input device (devm_input_allocate_device)
    // - LEDs (devm_kcalloc for piu->led, devm_led_classdev_register)
    // - LED names (devm_kasprintf)
    // - Main piu structure (devm_kzalloc)
}

// --- ID Table Includes Both Devices ---
static const struct hid_device_id piuio_ids[] = {
    { HID_USB_DEVICE(USB_VENDOR_ID_ANCHOR, USB_PRODUCT_ID_PYTHON2) },
    // Entry for the legacy device ID (0x1010)
    { HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_LEGACY) },
    // Entry for the new device ID (0x1020)
    { HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_NEW) },
    {} // Terminator entry
};
MODULE_DEVICE_TABLE(hid, piuio_ids); // Export table for module loading tools

// HID Driver Structure Definition
static struct hid_driver piuio_driver = {
    .name     = "piuio_hid", // Driver name visible in sysfs
    .id_table = piuio_ids,   // Table of supported device IDs
    .probe    = piuio_probe, // Probe function called for matching devices
    .remove   = piuio_remove,// Remove function called on device removal/driver unload
    .raw_event = piuio_raw_event, // Raw input event handler callback
    // Add .event if parsed input is needed, .feature_report for feature reports
    // Add power management hooks (.suspend, .resume) if needed
};

// Register the driver with the HID subsystem
module_hid_driver(piuio_driver);

// Module Metadata
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO HID interface with legacy compatibility");
MODULE_LICENSE("GPL v2");