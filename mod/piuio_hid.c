/*
 * PIUIO HID interface driver
 * Overrides generic Linux HID by matching PIUIO boards and provides
 * legacy /dev/piuio0 char-device compatibility for existing apps.
 *
 * Supports legacy (0x1010) and newer (0x1020) product IDs.
 * Input is handled via timer polling using GET_REPORT control transfers.
 * Input format for 1020 assumed 16 bytes (needs verification). 1010 unknown.
 * Output for 1020 uses Interrupt OUT endpoint (16 bytes) based on descriptor/logs.
 * Output for 1010 attempts legacy control transfer method (unverified).
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
#include <linux/string.h>   // memset, snprintf, memcpy
#include <linux/jiffies.h>  // msecs_to_jiffies
#include <linux/bits.h>     // BITS_TO_LONGS, BITS_PER_LONG
#include <linux/hrtimer.h>  // High-resolution timer
#include <linux/ktime.h>    // ktime_t
#include <linux/mutex.h>    // mutex

// Include local definitions and structures
#include "piuio_hid.h"

// --- Module Parameters ---
static int poll_interval_ms = 4; // Default poll interval
module_param(poll_interval_ms, int, 0444);
MODULE_PARM_DESC(poll_interval_ms, "Polling interval in milliseconds for input reports");


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
 * piuio_get_input_report - Poll device for input using GET_REPORT control transfer.
 * @piu: Device instance data.
 * @buffer: Buffer to store the received report data.
 * @buf_len: Size of the provided buffer.
 *
 * Uses GET_REPORT (Input Report ID 1) based on logs/python script.
 *
 * Return: Number of bytes received on success, negative error code otherwise.
 */
static int piuio_get_input_report(struct piuio *piu, u8 *buffer, size_t buf_len)
{
    int ret;

    // Parameters from logs/python: reqType=0xA1, req=1, val=0x0301, idx=iface
    ret = usb_control_msg(
        piu->udev,
        usb_rcvctrlpipe(piu->udev, 0),
        HID_REQ_GET_REPORT, // 0x01
        USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // 0xA1
        (HID_INPUT_REPORT << 8) | PIUIO_INPUT_REPORT_ID, // wValue (Report Type = Input | Report ID = 1)
        piu->iface, // wIndex (Interface)
        buffer,     // data buffer
        buf_len,    // max length to read
        msecs_to_jiffies(50)); // Short timeout for polling

    if (ret < 0 && ret != -ESHUTDOWN && ret != -ENODEV && ret != -EPIPE && ret != -ETIMEDOUT) {
         // Log errors other than expected disconnect/stall/timeout conditions
         hid_warn(piu->hdev, "GET_REPORT failed: %d\n", ret);
    }

    return ret; // Return bytes read or error
}


/**
 * piuio_set_report_legacy_ctrl - Send output report using legacy SET_REPORT control transfer.
 * @piu: Device instance data.
 * @rptid: The Report ID (typically 0x80 + bank number).
 *
 * Attempts to set LED states using the older control transfer method.
 * Retained primarily for potential compatibility with the 0x1010 device.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int piuio_set_report_legacy_ctrl(struct piuio *piu, u8 rptid)
{
    int bank = rptid - PIUIO_RPT_OUT_BASE;
    u8 *buf;
    int i, ret;

    // Use GFP_ATOMIC if potentially called from atomic context (e.g., timer)
    buf = kmalloc(PIUIO_OUTPUT_CHUNK + 1, GFP_ATOMIC);
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
 * piuio_output_report_interrupt_complete - Completion handler for interrupt OUT URB.
 * @urb: The completed URB.
 */
static void piuio_output_report_interrupt_complete(struct urb *urb)
{
    struct piuio *piu = urb->context;
    int status = urb->status;

    if (status && status != -ENOENT && status != -ECONNRESET && status != -ESHUTDOWN) {
        hid_warn(piu->hdev, "Interrupt OUT URB completed with status %d\n", status);
    }
    // Release mutex allowing next submission
    mutex_unlock(&piu->output_mutex);
}


/**
 * piuio_send_output_interrupt - Send LED state via Interrupt OUT endpoint (for 1020).
 * @piu: Device instance data.
 *
 * Prepares and submits a 16-byte report reflecting the current led_shadow state
 * to the Interrupt OUT endpoint (0x02).
 *
 * Return: 0 on success, negative error code on failure.
 */
static int piuio_send_output_interrupt(struct piuio *piu)
{
    int i, ret = 0;
    unsigned int pipe;

    // Try to acquire lock, exit if already sending
    if (!mutex_trylock(&piu->output_mutex))
        return -EBUSY;

    // Format the 16-byte output report based on led_shadow
    // ASSUMPTION: Report ID is 0 (not included in buffer), 16 bytes data map to first 128 LEDs?
    // Or map first 48 LEDs into the 16 bytes? Let's assume first 48 bits.
    memset(piu->output_buf, 0, PIUIO_OUTPUT_SIZE_NEW);
    spin_lock(&piu->lock);
    for (i = 0; i < PIUIO_MAX_LEDS && i < (PIUIO_OUTPUT_SIZE_NEW * 8); i++) {
        if (piu->led_shadow[i]) {
            __set_bit(i, (unsigned long *)piu->output_buf);
        }
    }
    spin_unlock(&piu->lock);


    // Endpoint address 0x02 (OUT)
    pipe = usb_sndintpipe(piu->udev, 0x02);

    // Initialize and submit the URB
    usb_fill_int_urb(piu->output_urb, piu->udev, pipe,
                     piu->output_buf, PIUIO_OUTPUT_SIZE_NEW,
                     piuio_output_report_interrupt_complete, piu,
                     1); // Interval (ms) for FS devices

    ret = usb_submit_urb(piu->output_urb, GFP_ATOMIC);
    if (ret) {
        hid_err(piu->hdev, "Failed to submit interrupt OUT URB: %d\n", ret);
        mutex_unlock(&piu->output_mutex); // Release lock on error
    }
    // Mutex is unlocked in completion handler on success

    return ret;
}


/**
 * piuio_timer_cb - Timer callback function for polling input reports.
 * @timer: Pointer to the hrtimer structure.
 *
 * Polls the device using GET_REPORT control transfer and processes the result.
 *
 * Return: HRTIMER_RESTART to reschedule the timer.
 */
static enum hrtimer_restart piuio_timer_cb(struct hrtimer *timer)
{
    struct piuio *piu = container_of(timer, struct piuio, timer);
    int size, i, max_inputs_in_report = 0;
    int inputs_to_process = 0;
    int num_longs = 0;
    bool state_changed = false;
    const unsigned long *current_data_bits = NULL;
    int bit_index;

    size = piuio_get_input_report(piu, piu->in_buf, PIUIO_INPUT_BUF_SIZE);

    if (size < 0) {
        goto reschedule; // Error already logged
    }

    // Determine expected format based on Product ID
    if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
        if (size != PIUIO_INPUT_SIZE_NEW) {
             hid_dbg(piu->hdev, "Ignoring polled 1020 input with size %d (expected %d)\n", size, PIUIO_INPUT_SIZE_NEW);
             goto reschedule;
        }
        // Assume 16 bytes received contain the input bits directly, no offset.
        current_data_bits = (unsigned long *)piu->in_buf;
        max_inputs_in_report = size * 8;

    } else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
        hid_warn(piu->hdev, "Ignoring polled input for legacy 1010 device (size %d) - format unknown\n", size);
        goto reschedule;

    } else {
        goto reschedule; // Should not happen
    }

    inputs_to_process = min(max_inputs_in_report, PIUIO_NUM_INPUTS);
    num_longs = BITS_TO_LONGS(inputs_to_process);

    // Compare current bits with previous state
    for (i = 0; i < num_longs; i++) {
        if (((void*)&current_data_bits[i]) >= ((void*)piu->in_buf + size)) {
            hid_warn(piu->hdev, "Bounds check failed reading input state (long %d)\n", i);
            break;
        }
        unsigned long current_bits = current_data_bits[i];
        unsigned long changed_bits = current_bits ^ piu->prev_inputs[i];

        if (changed_bits != 0) {
            state_changed = true;
            for_each_set_bit(bit_index, &changed_bits, BITS_PER_LONG) {
                 int global_bit_index = i * BITS_PER_LONG + bit_index;
                 if (global_bit_index >= inputs_to_process) break;
                 bool pressed = !test_bit(global_bit_index, current_data_bits);
                 input_event(piu->idev, EV_MSC, MSC_SCAN, global_bit_index + 1);
                 input_report_key(piu->idev, piuio_keycode(global_bit_index), pressed);
            }
             piu->prev_inputs[i] = current_bits;
        }
    }

    if (state_changed) {
        input_sync(piu->idev);
    }

reschedule:
    hrtimer_forward_now(timer, ms_to_ktime(poll_interval_ms));
    return HRTIMER_RESTART;
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
    int ret = -EOPNOTSUPP; // Default to not supported

    if (ld->idx >= PIUIO_MAX_LEDS) return -EINVAL;

    spin_lock(&piu->lock);
    piu->led_shadow[ld->idx] = (b > 0);
    spin_unlock(&piu->lock);

    // Choose output method based on device
    if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
        // Use Interrupt OUT transfer
        ret = piuio_send_output_interrupt(piu);
    } else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
        // Attempt legacy control transfer method (unverified if correct)
        ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (ld->idx / PIUIO_OUTPUT_CHUNK));
    } else {
        hid_warn(piu->hdev, "LED set called for unknown product ID %04x\n", piu->hdev->product);
    }

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
 * Updates LEDs using the appropriate method for the device.
 *
 * Return: Number of bytes written on success, negative error code otherwise.
 */
static ssize_t piuio_dev_write(struct file *filp,
                               const char __user *buf,
                               size_t len, loff_t *off)
{
    struct piuio *piu = filp->private_data;
    const size_t legacy_bits = PIUIO_LEGACY_SIZE * 8;
    u8 tmp[PIUIO_LEGACY_SIZE];
    int pin, ret = 0;
    int first_error = 0;

    if (len != PIUIO_LEGACY_SIZE)
        return -EINVAL;
    if (copy_from_user(tmp, buf, len))
        return -EFAULT;

    spin_lock(&piu->lock);
    // Clear and set only the LEDs controlled by the legacy interface
    memset(piu->led_shadow, 0, min_t(size_t, legacy_bits, PIUIO_MAX_LEDS));
    for (pin = 0; pin < legacy_bits && pin < PIUIO_MAX_LEDS; pin++)
        piu->led_shadow[pin] = !!(tmp[pin/8] & (1 << (pin%8)));
    spin_unlock(&piu->lock);

    // Update LEDs using the appropriate method
    if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
        ret = piuio_send_output_interrupt(piu);
        if (ret < 0) first_error = ret;
    } else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
        // Update all relevant output banks using legacy control transfer method
        for (pin = 0; pin < legacy_bits && pin < PIUIO_MAX_LEDS; pin += PIUIO_OUTPUT_CHUNK) {
             ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (pin / PIUIO_OUTPUT_CHUNK));
             if (ret < 0 && first_error == 0) {
                  first_error = ret; // Store first error
             }
        }
    } else {
         first_error = -EOPNOTSUPP;
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
 * Sets up the driver instance, input device, LEDs, misc device,
 * sends the SET_IDLE command, and starts the input polling timer.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int piuio_probe(struct hid_device *hdev,
                       const struct hid_device_id *id)
{
    struct piuio *piu;
    struct input_dev *idev;
    struct usb_interface *intf;
    struct usb_endpoint_descriptor *ep_out = NULL;
    int i, ret;

    hid_info(hdev, "Probing PIUIO device %04X:%04X\n", id->vendor, id->product);

    intf = to_usb_interface(hdev->dev.parent);
    if (!intf) {
        hid_err(hdev,"Cannot get USB interface\n");
        return -ENODEV;
    }

    piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
    if (!piu) return -ENOMEM;
    hid_set_drvdata(hdev, piu);

    piu->hdev = hdev;
    piu->udev = interface_to_usbdev(intf);
    piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;
    spin_lock_init(&piu->lock);
    mutex_init(&piu->output_mutex);

    // --- Input device setup ---
    snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys);
    idev = devm_input_allocate_device(&hdev->dev);
    if (!idev) return -ENOMEM;
    piu->idev = idev;

    idev->name = hdev->name;
    idev->phys = piu->phys;
    idev->id.bustype = hdev->bus;
    idev->id.vendor  = hdev->vendor;
    idev->id.product = hdev->product;
    idev->id.version = hdev->version;
    idev->dev.parent = &hdev->dev;

    set_bit(EV_KEY, idev->evbit);
    for (i = 0; i < PIUIO_NUM_INPUTS; i++)
        set_bit(piuio_keycode(i), idev->keybit);
    set_bit(EV_MSC, idev->evbit);
    set_bit(MSC_SCAN, idev->mscbit);

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
        piu->led[i].cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s::output%u", dev_name(&hdev->dev), i);
        if (!piu->led[i].cdev.name) return -ENOMEM;
        piu->led[i].cdev.brightness_set_blocking = piuio_led_set;
        piu->led[i].cdev.max_brightness = 1;
        ret = devm_led_classdev_register(&hdev->dev, &piu->led[i].cdev);
        if (ret) {
            hid_err(hdev, "Failed to register LED %d: %d\n", i, ret);
            return ret;
        }
    }

    // --- Misc device setup (/dev/piuioX) ---
    piu->misc.minor  = MISC_DYNAMIC_MINOR;
    piu->misc_name = kasprintf(GFP_KERNEL, "piuio-%s", dev_name(&hdev->dev));
    if (!piu->misc_name) return -ENOMEM;
    piu->misc.name = piu->misc_name;
    piu->misc.fops   = &piuio_fops;
    piu->misc.parent = &hdev->dev;

    ret = misc_register(&piu->misc);
    if (ret) {
        hid_err(hdev, "Failed to register misc device '%s': %d\n", piu->misc.name, ret);
        kfree(piu->misc_name);
        return ret;
    }
    hid_info(hdev, "Registered misc device /dev/%s\n", piu->misc.name);

    // --- Setup for Interrupt OUT URB (only needed if 1020 support uses it) ---
    // Find the interrupt OUT endpoint descriptor
    for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
        struct usb_endpoint_descriptor *ep = &intf->cur_altsetting->endpoint[i].desc;
        if (usb_endpoint_is_int_out(ep)) {
            ep_out = ep;
            break;
        }
    }
    // If supporting interrupt OUT for 1020, check ep_out is found and endpoint addr matches 0x02
    if (id->product == USB_PRODUCT_ID_BTNBOARD_NEW) {
        if (!ep_out || ep_out->bEndpointAddress != 0x02) {
            hid_err(hdev, "Interrupt OUT endpoint 0x02 not found for 1020 device\n");
            // Continue without interrupt output support? Or fail probe? Let's continue for now.
            ep_out = NULL; // Indicate interrupt out is not available
        } else {
            piu->output_buf = devm_kmalloc(&hdev->dev, PIUIO_OUTPUT_SIZE_NEW, GFP_KERNEL);
            piu->output_urb = usb_alloc_urb(0, GFP_KERNEL);
            if (!piu->output_buf || !piu->output_urb) {
                // Fail probe if resources for required output method aren't available
                hid_err(hdev, "Failed to allocate output URB/buffer\n");
                usb_free_urb(piu->output_urb); // Free urb if buf failed
                misc_deregister(&piu->misc);
                kfree(piu->misc_name);
                // devm_* cleanup handles input/LEDs
                return -ENOMEM;
            }
             hid_info(hdev, "Using Interrupt OUT EP 0x%02x for output\n", ep_out->bEndpointAddress);
        }
    }

    // --- Start HID Hardware Layer ---
    ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_HIDINPUT);
    if (ret) {
        hid_err(hdev, "hid_hw_start failed: %d\n", ret);
        usb_free_urb(piu->output_urb); // Free if allocated
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    }

    // --- Parse HID Reports ---
    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "hid_parse failed: %d\n", ret);
        hid_hw_stop(hdev);
        usb_free_urb(piu->output_urb);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    }

    // --- Open HID Device Communication Channel ---
    ret = hid_hw_open(hdev);
    if (ret) {
        hid_err(hdev, "hid_hw_open failed: %d\n", ret);
        hid_hw_stop(hdev);
        usb_free_urb(piu->output_urb);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    }

    // --- Send SET_IDLE Request ---
    ret = usb_control_msg(piu->udev,
                          usb_sndctrlpipe(piu->udev, 0),
                          HID_REQ_SET_IDLE,
                          USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                          0, piu->iface, NULL, 0, msecs_to_jiffies(1000));
    if (ret < 0 && ret != -EPIPE) {
        hid_err(hdev, "SET_IDLE failed: %d\n", ret);
        hid_hw_close(hdev);
        hid_hw_stop(hdev);
        usb_free_urb(piu->output_urb);
        misc_deregister(&piu->misc);
        kfree(piu->misc_name);
        return ret;
    } else if (ret == -EPIPE) {
         hid_warn(hdev, "SET_IDLE stalled (continuing)\n");
    } else {
         hid_info(hdev, "SET_IDLE sent successfully\n");
    }

    // --- Initialize and Start Polling Timer ---
    hrtimer_init(&piu->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    piu->timer.function = piuio_timer_cb;
    hrtimer_start(&piu->timer, ms_to_ktime(poll_interval_ms), HRTIMER_MODE_REL);


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

    // Cancel and wait for timer to finish
    hrtimer_cancel(&piu->timer);

    // Kill pending output URB and free resources if they were allocated
    usb_kill_urb(piu->output_urb);
    usb_free_urb(piu->output_urb);
    // output_buf is freed by devm_kmalloc automatically

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
MODULE_DEVICE_TABLE(hid, piuio_ids);

// HID Driver Structure Definition
static struct hid_driver piuio_driver = {
    .name     = "piuio_hid", // Driver name
    .id_table = piuio_ids,   // Supported device IDs
    .probe    = piuio_probe, // Probe function
    .remove   = piuio_remove,// Remove function
    // Input is handled via timer polling, not HID events
};

// Register the driver with the HID subsystem
module_hid_driver(piuio_driver);

// Module Metadata
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO HID interface driver");
MODULE_LICENSE("GPL v2");