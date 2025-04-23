/*
 * PIUIO HID interface driver
 * (Based on analysis of 0x1020 device traffic)
 * Overrides generic Linux HID by matching PIUIO boards and provides
 * legacy /dev/piuio0 char-device compatibility for existing apps.
 *
 * Supports legacy (0x1010) and newer (0x1020) product IDs.
 * Input is handled via timer polling using GET_REPORT control transfers.
 * Input format for 1020 requires 32 bytes (Report ID 0x30).
 * Output for 1020 uses Interrupt OUT endpoint (16 bytes, Report ID 0x13).
 * Legacy (1010) device support is minimal/unverified.
 * LED support is conditional on CONFIG_LEDS_CLASS.
 *
 * Legacy implementation basis:
 * Copyright (C) 2012-2014 Devin J. Pohly (djpohly+linux@gmail.com)
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
#include <linux/slab.h>
#ifdef CONFIG_LEDS_CLASS
#include <linux/leds.h>
#endif
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/kconfig.h>
#include <linux/poll.h>

#include "piuio_hid.h"

static int poll_interval_ms = 4;
module_param(poll_interval_ms, int, 0444);
MODULE_PARM_DESC(poll_interval_ms, "Polling interval in milliseconds for input reports");

/* Prevent multiple device instances */
static atomic_t piuio_device_count = ATOMIC_INIT(0);

/* Map raw input bit index to key/button code */
static inline int piuio_keycode(unsigned pin)
{
    if (pin < (BTN_GAMEPAD - BTN_JOYSTICK))
        return PIUIO_BTN_REG + pin;
    if (pin >= PIUIO_NUM_INPUTS)
        return -EINVAL;
    return PIUIO_BTN_EXTRA + (pin - (BTN_GAMEPAD - BTN_JOYSTICK));
}

/* Poll device with GET_REPORT (0x30) */
static int piuio_get_input_report(struct piuio *piu, u8 *buffer, size_t buf_len)
{
    int ret;
    const u8 report_id = PIUIO_INPUT_REPORT_ID; // 0x30
    const u16 report_len = PIUIO_INPUT_SIZE_NEW; // 32
    const u16 wValue = (HID_INPUT_REPORT << 8) | report_id; // 0x0130

    if (!piu || !piu->udev || !piu->hdev || !buffer) {
        pr_err("piuio: Invalid arguments to piuio_get_input_report\n");
        return -EINVAL;
    }
    if (buf_len < report_len) {
         pr_err("piuio: Input buffer too small (%zu bytes) for expected report (%u bytes)\n",
               buf_len, report_len);
        return -EINVAL;
    }

    ret = usb_control_msg(
        piu->udev,
        usb_rcvctrlpipe(piu->udev, 0),
        HID_REQ_GET_REPORT,
        USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
        wValue,
        piu->iface,
        buffer,
        report_len,
        msecs_to_jiffies(50));

    if (ret < 0 && ret != -ESHUTDOWN && ret != -ENODEV && ret != -EPIPE && ret != -ETIMEDOUT) {
        hid_warn(piu->hdev, "GET_REPORT (ID 0x%02x) failed: %d\n", report_id, ret);
    } else if (ret >= 0 && ret != report_len) {
        hid_warn(piu->hdev, "GET_REPORT (ID 0x%02x) returned %d bytes (expected %u)\n",
                 report_id, ret, report_len);
        return -EIO;
    } else if (ret == report_len) {
        hid_dbg(piu->hdev, "GET_REPORT (ID 0x%02x) successful (%d bytes)\n", report_id, ret);
    }

    return ret;
}

/* Feed polled input into HID core */
static void piuio_process_polled_input(struct piuio *piu, u8 *data, int size)
{
    if (!piu || !piu->hdev || !piu->idev || !data) {
        pr_err("piuio: Invalid state/data in process_polled_input\n");
        return;
    }
    if (size != PIUIO_INPUT_SIZE_NEW) {
         hid_dbg(piu->hdev, "Ignoring input report with unexpected size %d (expected %d)\n",
                size, PIUIO_INPUT_SIZE_NEW);
        return;
    }
    hid_dbg(piu->hdev, "Submitting %d byte input report to HID core\n", size);
    hid_input_report(piu->hdev, HID_INPUT_REPORT, data, size, 0);
}

/* Timer callback for polling */
static enum hrtimer_restart piuio_timer_cb(struct hrtimer *timer)
{
    struct piuio *piu = container_of(timer, struct piuio, timer);
    int size;

    if (!piu || !piu->hdev || !piu->idev || !piu->udev)
        return HRTIMER_NORESTART;

    size = piuio_get_input_report(piu, piu->in_buf, PIUIO_INPUT_BUF_SIZE);

    if (size == PIUIO_INPUT_SIZE_NEW) {
        piuio_process_polled_input(piu, piu->in_buf, size);
    } else if (size >= 0) {
        hid_dbg(piu->hdev, "Ignoring input report fragment (%d bytes)\n", size);
    } else if (size == -ENODEV || size == -ESHUTDOWN) {
        return HRTIMER_NORESTART;
    } else {
        hid_dbg(piu->hdev, "Polling failed with error %d, continuing timer.\n", size);
    }

    hrtimer_forward_now(timer, ms_to_ktime(poll_interval_ms));
    return HRTIMER_RESTART;
}

#ifdef CONFIG_LEDS_CLASS
/* Completion for interrupt OUT URB */
static void piuio_output_report_interrupt_complete(struct urb *urb)
{
    struct piuio *piu = urb->context;
    unsigned long flags;

    if (!piu) {
        pr_err("piuio: Invalid context in output URB completion!\n");
        return;
    }

    spin_lock_irqsave(&piu->output_submit_lock, flags);
    atomic_set(&piu->output_active, 0);
    spin_unlock_irqrestore(&piu->output_submit_lock, flags);

    if (urb->status && urb->status != -ENOENT && urb->status != -ECONNRESET && urb->status != -ESHUTDOWN)
        hid_warn(piu->hdev, "Interrupt OUT URB status %d\n", urb->status);
}

/* Send LED state via interrupt OUT */
static int piuio_send_output_interrupt(struct piuio *piu)
{
    int i, ret = 0;
    unsigned long flags_led, flags_submit;

    if (!piu || !piu->output_urb || !piu->output_buf || !piu->udev || !piu->hdev) {
        pr_err("piuio: Invalid state in piuio_send_output_interrupt\n");
        return -EFAULT;
    }
    if (!piu->output_pipe) {
        hid_warn_once(piu->hdev, "Cannot send output report, pipe not initialized\n");
        return -ENODEV;
    }

    spin_lock_irqsave(&piu->output_submit_lock, flags_submit);
    if (atomic_read(&piu->output_active)) {
        spin_unlock_irqrestore(&piu->output_submit_lock, flags_submit);
        return 0; // Busy, not an error
    }
    atomic_set(&piu->output_active, 1);
    spin_unlock_irqrestore(&piu->output_submit_lock, flags_submit);

    memset(piu->output_buf, 0, PIUIO_OUTPUT_SIZE_NEW);
    piu->output_buf[0] = PIUIO_OUTPUT_REPORT_ID; // Set Report ID 0x13

    spin_lock_irqsave(&piu->led_lock, flags_led);
    memset(piu->output_buf + 1, 0, PIUIO_OUTPUT_SIZE_NEW - 1); // Clear payload
    for (i = 0; i < PIUIO_MAX_LEDS; i++) {
        if (piu->led_shadow[i]) {
            int byte_idx = 1 + (i >> 3); // Byte index 1-15
            int bit_idx = i & 7;         // Bit index 0-7
            if (byte_idx < PIUIO_OUTPUT_SIZE_NEW)
                piu->output_buf[byte_idx] |= (1 << bit_idx);
        }
    }
    spin_unlock_irqrestore(&piu->led_lock, flags_led);

    usb_fill_int_urb(piu->output_urb, piu->udev, piu->output_pipe,
                     piu->output_buf, PIUIO_OUTPUT_SIZE_NEW,
                     piuio_output_report_interrupt_complete, piu, 1);

    ret = usb_submit_urb(piu->output_urb, GFP_ATOMIC);
    if (ret) {
        unsigned long flags; // Need flags again for lock
        hid_err(piu->hdev, "Failed to submit URB: %d\n", ret);
        spin_lock_irqsave(&piu->output_submit_lock, flags);
        atomic_set(&piu->output_active, 0);
        spin_unlock_irqrestore(&piu->output_submit_lock, flags);
    }
    return ret;
}

/* LED brightness callback */
static int piuio_led_set(struct led_classdev *cdev, enum led_brightness b)
{
    struct piuio_led *ld = container_of(cdev, struct piuio_led, cdev);
    struct piuio *piu = ld->piu;
    unsigned long flags;
    int ret = -EOPNOTSUPP;

    if (!piu || !piu->hdev) return -ENODEV;
    if (ld->idx >= PIUIO_MAX_LEDS) return -EINVAL;

    spin_lock_irqsave(&piu->led_lock, flags);
    piu->led_shadow[ld->idx] = (b > 0);
    spin_unlock_irqrestore(&piu->led_lock, flags);

    if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW)
        ret = piuio_send_output_interrupt(piu);
    else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY)
        hid_warn_once(piu->hdev, "LED set for legacy PIUIO not implemented\n");
    else
        hid_warn(piu->hdev, "LED set for unknown product ID %04x\n", piu->hdev->product);


    return ret < 0 ? ret : 0;
}
#endif /* CONFIG_LEDS_CLASS */

/* Legacy write handler (/dev/piuioX) */
static ssize_t piuio_dev_write(struct file *filp,
                               const char __user *buf,
                               size_t len, loff_t *off)
{
    struct miscdevice *misc = filp->private_data;
    struct piuio *piu; // Retrieve below
#ifdef CONFIG_LEDS_CLASS
    unsigned long flags;
    u8 *tmp;
    int pin, ret = 0;
    int first_error = 0;
#endif

    if (len != PIUIO_LEGACY_SIZE)
        return -EINVAL;

    // Check misc pointer *before* dereferencing it
    if (!misc || !misc->this_device) {
         pr_err("piuio: Invalid misc device data in write\n");
         return -ENODEV;
    }
    piu = dev_get_drvdata(misc->this_device);
    if (!piu || !piu->hdev) {
         pr_err("piuio: Invalid driver state in write for %s\n", misc->name);
         return -ENODEV;
    }

#ifndef CONFIG_LEDS_CLASS
    return len; // Accept write as no-op if LEDs disabled
#else
    tmp = kmalloc(len, GFP_KERNEL);
    if (!tmp)
        return -ENOMEM;
    if (copy_from_user(tmp, buf, len)) {
        kfree(tmp);
        return -EFAULT;
    }

    spin_lock_irqsave(&piu->led_lock, flags);
    for (pin = 0; (pin < PIUIO_LEGACY_SIZE*8) && (pin < PIUIO_MAX_LEDS); pin++)
        piu->led_shadow[pin] = !!(tmp[pin>>3] & (1 << (pin&7)));
    spin_unlock_irqrestore(&piu->led_lock, flags);
    kfree(tmp);

    if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) {
        ret = piuio_send_output_interrupt(piu);
        if (ret < 0) first_error = ret;
    } else {
        hid_warn_once(piu->hdev, "Legacy write for non-1020 device not implemented\n");
        first_error = -EOPNOTSUPP;
    }

    return first_error < 0 ? first_error : len;
#endif
}

static const struct file_operations piuio_fops = {
    .owner   = THIS_MODULE,
    .write   = piuio_dev_write,
    .open    = simple_open,
    .llseek  = no_llseek,
};

/* Probe function */
static int piuio_probe(struct hid_device *hdev,
                       const struct hid_device_id *id)
{
    struct piuio *piu;
    struct input_dev *idev = NULL; // Initialize to NULL
    struct usb_interface *intf;
    struct usb_endpoint_descriptor *ep_out = NULL;
    int i, ret;
#ifdef CONFIG_LEDS_CLASS
    int registered_leds = 0;
#endif

    /* Prevent multiple instances */
    if (atomic_inc_return(&piuio_device_count) > 1) {
        hid_warn(hdev, "PIUIO device already handled, skipping\n");
        atomic_dec(&piuio_device_count); // Decrement if skipping
        return -ENODEV;
    }

    hid_info(hdev, "Probing PIUIO %04X:%04X\n", id->vendor, id->product);

    intf = to_usb_interface(hdev->dev.parent);
    if (!intf) {
        ret = -ENODEV; goto err_count;
    }

    piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
    if (!piu) { ret = -ENOMEM; goto err_count; }

    /* Associate piu struct with hdev early for cleanup */
    hid_set_drvdata(hdev, piu);

    piu->hdev  = hdev;
    piu->udev  = interface_to_usbdev(intf);
    piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;
#ifdef CONFIG_LEDS_CLASS
    spin_lock_init(&piu->led_lock);
#endif
    spin_lock_init(&piu->output_submit_lock);
    atomic_set(&piu->output_active, 0);

    if (!piu->udev) {
        hid_err(hdev, "Cannot get USB device\n");
        ret = -ENODEV;
        goto err_count; // Use devm cleanup
    }

    /* Find OUT endpoint */
    for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
        struct usb_endpoint_descriptor *ep =
            &intf->cur_altsetting->endpoint[i].desc;
        if (!ep_out && usb_endpoint_is_int_out(ep)) { ep_out = ep; break; }
    }
    if (ep_out) {
        hid_info(hdev, "Found Interrupt OUT endpoint: addr=0x%02x, size=%d\n",
                     ep_out->bEndpointAddress, usb_endpoint_maxp(ep_out)); // Use ep_out here
    } else {
        hid_warn(hdev, "No Interrupt OUT endpoint found\n");
    }


    /* Setup input device */
    snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys);
    idev = devm_input_allocate_device(&hdev->dev);
    if (!idev) { ret = -ENOMEM; goto err_count; }
    piu->idev = idev; // Store idev pointer

    idev->name = hdev->name;
    idev->phys = piu->phys;
    idev->id.bustype = hdev->bus;
    idev->id.vendor  = hdev->vendor;
    idev->id.product = hdev->product;
    idev->id.version = hdev->version;
    idev->dev.parent = &hdev->dev;

    /* Set input capabilities */
    set_bit(EV_KEY, idev->evbit);
    for (i = 0; i < PIUIO_NUM_INPUTS; i++) {
        int keycode = piuio_keycode(i);
        if (keycode >= 0)
            set_bit(keycode, idev->keybit);
    }
    set_bit(EV_MSC, idev->evbit);
    set_bit(MSC_SCAN, idev->mscbit);

    /* Register input device */
    ret = input_register_device(idev);
    if (ret) {
        hid_err(hdev, "Failed to register input device: %d\n", ret);
        // idev allocated by devm is freed automatically
        goto err_count;
    }
    hid_info(hdev, "Registered input device %s", idev->name);


#ifdef CONFIG_LEDS_CLASS
    /* Setup LEDs */
    piu->led = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS, sizeof(*piu->led), GFP_KERNEL);
    if (!piu->led) { ret = -ENOMEM; goto err_unregister_input; }
    hid_info(hdev, "Allocated %d LED structures", PIUIO_MAX_LEDS);

    for (i = 0; i < PIUIO_MAX_LEDS; i++) {
        piu->led[i].piu = piu;
        piu->led[i].idx = i;
        piu->led[i].cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s::output%u", dev_name(&hdev->dev), i);
        if (!piu->led[i].cdev.name) { ret = -ENOMEM; registered_leds = i; goto err_unregister_leds; }
        piu->led[i].cdev.brightness_set_blocking = piuio_led_set;
        piu->led[i].cdev.max_brightness = 1;
        piu->led[i].cdev.flags = 0;

        ret = led_classdev_register(&hdev->dev, &piu->led[i].cdev);
        if (ret) {
            hid_err(hdev, "Failed to register LED %d (%s): %d\n", i, piu->led[i].cdev.name, ret);
            registered_leds = i;
            goto err_unregister_leds;
        }
    }
    registered_leds = PIUIO_MAX_LEDS;
    hid_info(hdev, "Registered %d LED class devices", registered_leds);
#endif // CONFIG_LEDS_CLASS


    /* Setup for Interrupt OUT URB (1020) */
    if (id->product == USB_PRODUCT_ID_BTNBOARD_NEW) {
        piu->output_buf = devm_kzalloc(&hdev->dev, PIUIO_OUTPUT_SIZE_NEW, GFP_KERNEL);
        piu->output_urb = usb_alloc_urb(0, GFP_KERNEL);
        if (!piu->output_buf || !piu->output_urb) {
            hid_err(hdev, "Failed to allocate output URB/buffer\n");
            usb_free_urb(piu->output_urb);
            ret = -ENOMEM;
            goto err_unregister_leds_or_input;
        }
        if (ep_out) {
            piu->output_pipe = usb_sndintpipe(piu->udev, ep_out->bEndpointAddress);
            hid_info(hdev, "Allocated URB %p and buffer %p for Interrupt OUT EP 0x%02x\n",
                     piu->output_urb, piu->output_buf, ep_out->bEndpointAddress);
        } else {
            hid_warn(hdev, "Output URB/buffer allocated, but no valid endpoint found.\n");
            piu->output_pipe = 0; // Mark pipe as invalid
        }
    }

    /* Legacy Workqueue setup REMOVED */

    /* Start HID Hardware Layer (HIDRAW) */
    ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
    if (ret) {
        hid_err(hdev, "hid_hw_start failed: %d\n", ret);
        goto err_free_output_urb; // Cleanup URB if allocated
    }
    hid_info(hdev, "hid_hw_start successful with HID_CONNECT_HIDRAW");


    /* Parse HID Reports */
    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "hid_parse failed: %d\n", ret);
        goto err_stop_hid;
    }
    hid_info(hdev, "hid_parse successful");

    /* Open HID Device Communication Channel */
    ret = hid_hw_open(hdev);
    if (ret) {
        hid_err(hdev, "hid_hw_open failed: %d\n", ret);
        goto err_stop_hid;
    }
    hid_info(hdev, "hid_hw_open successful");

    /* Send SET_IDLE Request */
    ret = usb_control_msg(piu->udev, usb_sndctrlpipe(piu->udev, 0),
                          HID_REQ_SET_IDLE,
                          USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                          0, piu->iface, NULL, 0, msecs_to_jiffies(100));
    if (ret < 0 && ret != -EPIPE) {
        hid_warn(hdev, "SET_IDLE failed: %d\n", ret);
    } else {
        hid_info(hdev, "SET_IDLE sent\n");
    }
    /* Don't fail probe for SET_IDLE failure */
    ret = 0;


    /* Initialize and Start Polling Timer */
    hrtimer_init(&piu->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    piu->timer.function = piuio_timer_cb;
    hrtimer_start(&piu->timer, ms_to_ktime(poll_interval_ms), HRTIMER_MODE_REL);
    hid_info(hdev, "Started polling timer (interval %d ms)\n", poll_interval_ms);


    /* Misc device setup */
    piu->misc.minor = MISC_DYNAMIC_MINOR;
    piu->misc_name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "piuio-%s", dev_name(&hdev->dev));
    if (!piu->misc_name) { ret = -ENOMEM; goto err_cancel_timer; }
    piu->misc.name = piu->misc_name;
    piu->misc.fops = &piuio_fops;
    piu->misc.parent = &hdev->dev;

    ret = misc_register(&piu->misc);
    if (ret) { hid_err(hdev, "misc_register failed: %d\n", ret); goto err_cancel_timer; }
    /* Associate piu struct with the misc device for write() */
    /* Using non-& version assuming this_device is pointer, based on prior errors */
    /* If this causes issues on standard kernels, needs revisiting */
    dev_set_drvdata(piu->misc.this_device, piu);
    hid_info(hdev, "Registered misc device /dev/%s\n", piu->misc.name);

    hid_info(hdev, "Probe successful\n");
    return 0; // Success!

/* --- Error Handling Cleanup --- */
err_cancel_timer:
    hrtimer_cancel(&piu->timer);
err_stop_hid:
    hid_hw_close(hdev);
    hid_hw_stop(hdev);
// err_destroy_workqueue: // REMOVED
err_free_output_urb:
    if (piu->output_urb) usb_free_urb(piu->output_urb);
    /* devm handles piu->output_buf */
err_unregister_leds_or_input: // Label used if URB/buf alloc fails or hid_hw_start fails
#ifdef CONFIG_LEDS_CLASS
err_unregister_leds:
    hid_info(hdev, "Unregistering %d LEDs due to probe error\n", registered_leds);
    for (i = 0; i < registered_leds; i++) {
        if (piu->led && piu->led[i].cdev.name) {
            led_classdev_unregister(&piu->led[i].cdev);
        }
    }
    /* devm handles piu->led and names */
    // Fall through
#endif
err_unregister_input:
    if (piu->idev) {
        input_unregister_device(piu->idev);
    }
    /* devm handles idev */
err_count:
    atomic_dec(&piuio_device_count); // Decrement count on error
err_free_piu:
    /* devm handles piu */
    hid_info(hdev, "Probe failed (%d)\n", ret);
    return ret;
}

/* Remove */
static void piuio_remove(struct hid_device *hdev)
{
    /* Retrieve piu using hid_get_drvdata */
    struct piuio *piu = hid_get_drvdata(hdev);
#ifdef CONFIG_LEDS_CLASS
    int i;
#endif

    if (!piu) return;

    hid_info(hdev, "Removing PIUIO device /dev/%s\n", piu->misc.name ? piu->misc.name : "?");

#ifdef CONFIG_LEDS_CLASS
    /* 1. Unregister LEDs */
    if (piu->led) {
        for (i = 0; i < PIUIO_MAX_LEDS; i++) {
            if (piu->led[i].cdev.name) { // Check if registered
                led_classdev_unregister(&piu->led[i].cdev);
            }
        }
    }
#endif

    /* 2. Deregister misc device */
    /* misc_register checks for null, but check name as proxy for registration */
    if (piu->misc.name) {
        misc_deregister(&piu->misc);
    }

    /* 3. Cancel timer */
    hrtimer_cancel(&piu->timer);

    /* 4. Kill/free output URB */
    if (piu->output_urb) {
        usb_kill_urb(piu->output_urb);
        usb_free_urb(piu->output_urb);
    }

    /* 5. Legacy WQ removed */

    /* 6. Close/stop HID */
    hid_hw_close(hdev);
    hid_hw_stop(hdev);

    /* 7. Unregister input device */
    if (piu->idev) {
        input_unregister_device(piu->idev);
    }

    /* 8. Decrement device count */
    atomic_dec(&piuio_device_count);

    hid_info(hdev, "Remove complete\n");
    /* devm handles piu, led array, names, output_buf, misc_name, idev alloc */
}


static const struct hid_device_id piuio_ids[] = {
    { HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_LEGACY) },
    { HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_NEW) },
    { }
};
MODULE_DEVICE_TABLE(hid, piuio_ids);

static struct hid_driver piuio_driver = {
    .name       = "piuio_hid",
    .id_table   = piuio_ids,
    .probe      = piuio_probe,
    .remove     = piuio_remove,
};
module_hid_driver(piuio_driver);

MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO HID interface driver");
MODULE_LICENSE("GPL v2");