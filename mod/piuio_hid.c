/*
 * PIUIO HID interface driver
 * Overrides generic Linux HID by matching PIUIO boards and provides
 * legacy /dev/piuio0 char-device compatibility for existing apps.
 *
 * Supports legacy (0x1010) and newer (0x1020) product IDs.
 * Input for 0x1020 is handled via interrupt endpoint and raw_event callback.
 * Input format for 0x1020 is assumed based on descriptor size (16 bytes)
 * and limited testing - format details might need refinement.
 * Input handling for 0x1010 is currently disabled pending investigation of its protocol.
 * Output handling (LEDs, legacy write) uses control transfers which may be incorrect
 * for the 0x1020 device based on its HID descriptor.
 *
 * Legacy implementation basis:
 * Copyright (C) 2012‑2014 Devin J. Pohly (djpohly+linux@gmail.com)
 *
 * HID integration, 0x1020 support, and modifications:
 * Copyright (C) 2025      Diego Acevedo <diego.acevedo.fernando@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>     // kzalloc, kfree, kasprintf
#include <linux/leds.h>     // LED framework
#include <linux/input.h>    // Input subsystem
#include <linux/usb.h>      // USB core
#include <linux/hid.h>      // USB HID core
#include <linux/miscdevice.h>// Misc device framework
#include <linux/fs.h>       // file_operations
#include <linux/uaccess.h>  // copy_from_user
#include <linux/spinlock.h> // spinlock
#include <linux/bitops.h>   // bit operations
#include <linux/string.h>   // memset, snprintf
#include <linux/jiffies.h>  // msecs_to_jiffies
#include <linux/bits.h>     // BITS_TO_LONGS, BITS_PER_LONG

// Include local definitions and structures
#include "piuio_hid.h"

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
 * Attempts to set LED states using the older control transfer method.
 * This may be incorrect for the 0x1020 device based on its HID descriptor.
 * Retained primarily for potential compatibility with the 0x1010 device.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int piuio_set_report_legacy_ctrl(struct piuio *piu, u8 rptid)
{
    int bank = rptid - PIUIO_RPT_OUT_BASE;
    u8 *buf;
    int i, ret;

    buf = kmalloc(PIUIO_OUTPUT_CHUNK + 1, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    buf[0] = rptid; // Report ID
    spin_lock(&piu->lock);
    for (i = 0; i < PIUIO_OUTPUT_CHUNK; i++) {
        int idx = bank * PIUIO_OUTPUT_CHUNK + i;
        buf[i+1] = (idx < PIUIO_MAX_LEDS && piu->led_shadow[idx]) ? 1 : 0;
    }
    spin_unlock(&piu->lock);

    ret = usb_control_msg(
        piu->udev,
        usb_sndctrlpipe(piu->udev, 0),
        HID_REQ_SET_REPORT,
        USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
        (HID_OUTPUT_REPORT << 8) | rptid,
        piu->iface,
        buf, PIUIO_OUTPUT_CHUNK + 1,
        msecs_to_jiffies(1000));

    kfree(buf);

    if (ret < 0) {
         hid_warn(piu->hdev, "SET_REPORT(ctrl) for ID 0x%02x failed: %d\n", rptid, ret);
         return ret;
    } else if (ret != (PIUIO_OUTPUT_CHUNK + 1)) {
        hid_warn(piu->hdev, "SET_REPORT(ctrl) for ID 0x%02x transferred %d bytes (expected %d)\n", rptid, ret, PIUIO_OUTPUT_CHUNK + 1);
        // Continue anyway
    }

    return 0; // Success
}


/**
 * piuio_raw_event - Callback for processing raw HID input reports.
 * @hdev: The HID device.
 * @report: The HID report structure.
 * @raw_data: Buffer containing the raw report data from the interrupt endpoint.
 * @size: The size of the data in raw_data.
 *
 * Parses input data based on Product ID and reports changes.
 * Input format for 1020 is assumed based on size (16 bytes) and limited testing.
 * Input format/mechanism for 1010 is unknown and currently ignored.
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
    const unsigned long *current_data_bits = NULL;
    int bit_index; // Loop variable for set bits

    // Determine expected format based on Product ID
    if (hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
        if (size != PIUIO_INPUT_SIZE_NEW) {
             hid_warn(hdev, "Ignoring 1020 input report with size %d (expected %d)\n", size, PIUIO_INPUT_SIZE_NEW);
             return 0;
        }
        // Assume 16 bytes received contain the input bits directly.
        current_data_bits = (unsigned long *)raw_data;
        max_inputs_in_report = size * 8;

    } else if (hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
        // Behavior and report format/size for the legacy device are unknown.
        hid_warn(hdev, "Ignoring input report for legacy 1010 device (size %d) - format unknown\n", size);
        return 0;

    } else {
        return 0; // Not a device we handle input for
    }

    // Limit processing based on report size and defined max inputs
    inputs_to_process = min(max_inputs_in_report, PIUIO_NUM_INPUTS);
    num_longs = BITS_TO_LONGS(inputs_to_process);

    // Compare current bits with previous state
    for (i = 0; i < num_longs; i++) {
        // Basic bounds check on reading current bits
        if (((void*)&current_data_bits[i]) >= ((void*)raw_data + size)) {
            hid_warn(hdev, "Bounds check failed reading input state (long %d)\n", i);
            break;
        }

        unsigned long current_bits = current_data_bits[i];
        unsigned long changed_bits = current_bits ^ piu->prev_inputs[i];

        if (changed_bits != 0) {
            state_changed = true;
            // Iterate over changed bits within this long
            for_each_set_bit(bit_index, &changed_bits, BITS_PER_LONG) {
                 int global_bit_index = i * BITS_PER_LONG + bit_index;

                 if (global_bit_index >= inputs_to_process) break;

                 // Assumes 0 = pressed / bit cleared
                 bool pressed = !test_bit(global_bit_index, current_data_bits);

                 // Report event to input subsystem
                 input_event(piu->idev, EV_MSC, MSC_SCAN, global_bit_index + 1);
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
 *
 * Return: 0 on success, negative error code on failure.
 */
static int piuio_led_set(struct led_classdev *cdev,
                          enum led_brightness b)
{
    struct piuio_led *ld = container_of(cdev, struct piuio_led, cdev);
    struct piuio *piu = ld->piu;
    int ret;

    if (ld->idx >= PIUIO_MAX_LEDS) return -EINVAL;

    spin_lock(&piu->lock);
    piu->led_shadow[ld->idx] = (b > 0);
    spin_unlock(&piu->lock);

    // Send update using the legacy control transfer method.
    // This might be incorrect for the 1020 device.
    ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (ld->idx / PIUIO_OUTPUT_CHUNK));

    return ret;
}


/**
 * piuio_dev_write - Handle writes to the legacy /dev/piuioX device.
 * @filp: File structure pointer.
 * @buf: User space buffer containing data to write.
 * @len: Number of bytes to write (must match PIUIO_LEGACY_SIZE).
 * @off: File offset (ignored).
 *
 * Assumes the written data is a bitmap for the first N LEDs.
 * Updates LEDs using the legacy control transfer method.
 *
 * Return: Number of bytes written on success, negative error code otherwise.
 */
static ssize_t piuio_dev_write(struct file *filp,
                               const char __user *buf,
                               size_t len, loff_t *off)
{
    struct piuio *piu = filp->private_data;
    u8 tmp[PIUIO_LEGACY_SIZE];
    int pin, ret = 0;
    int first_error = 0;

    if (len != PIUIO_LEGACY_SIZE)
        return -EINVAL;
    if (copy_from_user(tmp, buf, len))
        return -EFAULT;

    spin_lock(&piu->lock);
    // Clear and set only the LEDs controlled by the legacy interface
    memset(piu->led_shadow, 0, PIUIO_LEGACY_SIZE * 8);
    for (pin = 0; pin < PIUIO_LEGACY_SIZE*8 && pin < PIUIO_MAX_LEDS; pin++)
        piu->led_shadow[pin] = !!(tmp[pin/8] & (1 << (pin%8)));
    spin_unlock(&piu->lock);

    // Update all relevant output banks using legacy method
    for (pin = 0; pin < PIUIO_LEGACY_SIZE*8 && pin < PIUIO_MAX_LEDS; pin += PIUIO_OUTPUT_CHUNK) {
         ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (pin / PIUIO_OUTPUT_CHUNK));
         if (ret < 0 && first_error == 0) {
              first_error = ret; // Store first error
         }
    }

    return first_error ? first_error : len;
}

// File operations for the legacy /dev/piuioX device
static const struct file_operations piuio_fops = {
    .owner   = THIS_MODULE,
    .write   = piuio_dev_write,
    .open    = simple_open,
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
    idev->id.bustype = hdev->bus;
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
        return ret;
    }

    // --- LED class device setup ---
    piu->led = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS, sizeof(*piu->led), GFP_KERNEL);
    if (!piu->led) return -ENOMEM;

    for (i = 0; i < PIUIO_MAX_LEDS; i++) {
        piu->led[i].piu = piu;
        piu->led[i].idx = i;
        // Use unique name including device path segment
        piu->led[i].cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s::output%u", dev_name(&hdev->dev), i);
        if (!piu->led[i].cdev.name) return -ENOMEM;
        piu->led[i].cdev.brightness_set_blocking = piuio_led_set;
        piu->led[i].cdev.max_brightness = 1;

        // Register LED using device-managed function
        ret = devm_led_classdev_register(&hdev->dev, &piu->led[i].cdev);
        if (ret) {
            hid_err(hdev, "Failed to register LED %d: %d\n", i, ret);
            return ret;
        }
    }

    // --- Misc device setup (/dev/piuioX) ---
    piu->misc.minor  = MISC_DYNAMIC_MINOR;
    // Create a unique name like "piuio-0003:0D2F:1020.0001"
    piu->misc_name = kasprintf(GFP_KERNEL, "piuio-%s", dev_name(&hdev->dev));
    if (!piu->misc_name) return -ENOMEM;
    piu->misc.name = piu->misc_name;
    piu->misc.fops   = &piuio_fops;
    piu->misc.parent = &hdev->dev; // Associate with the HID device

    ret = misc_register(&piu->misc);
    if (ret) {
        hid_err(hdev, "Failed to register misc device '%s': %d\n", piu->misc.name, ret);
        kfree(piu->misc_name);
        return ret;
    }
    hid_info(hdev, "Registered misc device /dev/%s\n", piu->misc.name);


    // --- Start HID Hardware Layer ---
    // Connect default endpoints (HIDRAW etc) but prevent hid-input node creation
    ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_HIDINPUT);
    if (ret) {
        hid_err(hdev, "hid_hw_start failed: %d\n", ret);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    }

    // --- Parse HID Reports ---
    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "hid_parse failed: %d\n", ret);
        hid_hw_stop(hdev);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    }

    // --- Open HID Device Communication Channel ---
    ret = hid_hw_open(hdev);
    if (ret) {
        hid_err(hdev, "hid_hw_open failed: %d\n", ret);
        hid_hw_stop(hdev);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    }

    // --- Send SET_IDLE Request ---
    // Required by 1020 device to start sending input reports.
    ret = usb_control_msg(piu->udev,
                          usb_sndctrlpipe(piu->udev, 0),
                          HID_REQ_SET_IDLE,
                          USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                          0, // wValue (Report ID 0, Infinite Idle)
                          piu->iface, // wIndex (Interface)
                          NULL, 0, // No data
                          msecs_to_jiffies(1000));
    if (ret < 0 && ret != -EPIPE) {
        hid_err(hdev, "SET_IDLE failed: %d\n", ret);
        hid_hw_close(hdev);
        hid_hw_stop(hdev);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    } else if (ret == -EPIPE) {
         hid_warn(hdev, "SET_IDLE stalled (continuing)\n");
    } else {
         hid_info(hdev, "SET_IDLE sent successfully\n");
    }

    hid_info(hdev, "PIUIO HID Driver registered successfully\n");
    return 0; // Success!
}


/**
 * piuio_remove - Called when the HID device is removed or driver unloaded.
 * @hdev: Pointer to the HID device structure.
 *
 * Cleans up resources allocated in probe. Device-managed resources are freed automatically.
 */
static void piuio_remove(struct hid_device *hdev)
{
    struct piuio *piu = hid_get_drvdata(hdev);

    hid_info(hdev, "Unregistering PIUIO HID Driver\n");

    // Close HID hardware connection
    hid_hw_close(hdev);

    // Stop HID hardware layer
    hid_hw_stop(hdev);

    // De-register misc device (and free its allocated name)
    misc_deregister(&piu->misc);
    kfree(piu->misc_name);

    // devm_* functions automatically clean up other resources.
}

// --- ID Table Includes Both Devices ---
static const struct hid_device_id piuio_ids[] = {
    { HID_USB_DEVICE(USB_VENDOR_ID_ANCHOR, USB_PRODUCT_ID_PYTHON2) },
    { HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_LEGACY) }, // 0x1010
    { HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_NEW) },    // 0x1020
    {} // Terminator entry
};
MODULE_DEVICE_TABLE(hid, piuio_ids); // Export table for module loading tools

// HID Driver Structure Definition
static struct hid_driver piuio_driver = {
    .name     = "piuio_hid", // Driver name
    .id_table = piuio_ids,   // Supported device IDs
    .probe    = piuio_probe, // Probe function
    .remove   = piuio_remove,// Remove function
    .raw_event = piuio_raw_event, // Raw input event handler
};

// Register the driver with the HID subsystem
module_hid_driver(piuio_driver);

// Module Metadata
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO HID interface driver with legacy compatibility");
MODULE_LICENSE("GPL v2");