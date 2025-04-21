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
#include <linux/slab.h>     // kzalloc, kfree, kasprintf
#ifdef CONFIG_LEDS_CLASS
#include <linux/leds.h>     // LED framework
#endif
#include <linux/input.h>    // Input subsystem
#include <linux/usb.h>      // USB core
#include <linux/hid.h>      // USB HID core
#include <linux/miscdevice.h>// Misc device framework
#include <linux/fs.h>       // file_operations
#include <linux/uaccess.h>  // copy_from_user
#include <linux/spinlock.h> // spinlock API
#include <linux/bitops.h>   // bit operations
#include <linux/string.h>   // memset, snprintf, memcpy
#include <linux/jiffies.h>  // msecs_to_jiffies
#include <linux/hrtimer.h>  // High-resolution timer
#include <linux/ktime.h>    // ktime_t
#include <linux/atomic.h>   // Atomic operations
#include <linux/device.h>   // For device data functions
#include <linux/kconfig.h>  // For IS_ENABLED macro
#include <linux/poll.h>     // For hid_input_report potentially

// Include local definitions and structures
#include "piuio_hid.h"

// --- Module Parameters ---
static int poll_interval_ms = 4; // Default poll interval
module_param(poll_interval_ms, int, 0444);
MODULE_PARM_DESC(poll_interval_ms, "Polling interval in milliseconds for input reports");


/**
 * piuio_keycode - Map raw input bit index to a kernel key/button code.
 */
static inline int piuio_keycode(unsigned pin)
{
	if (pin < (BTN_GAMEPAD - BTN_JOYSTICK))
		return PIUIO_BTN_REG + pin;
	// Ensure pin doesn't exceed max mapped inputs
	if (pin >= PIUIO_NUM_INPUTS)
	    return -EINVAL;
	return PIUIO_BTN_EXTRA + (pin - (BTN_GAMEPAD - BTN_JOYSTICK));
}

/**
 * piuio_get_input_report - Poll device for input using GET_REPORT control transfer.
 * Uses Report ID 0x30, expects 32 bytes, based on USB trace analysis.
 */
static int piuio_get_input_report(struct piuio *piu, u8 *buffer, size_t buf_len)
{
	int ret;
	const u8 report_id = PIUIO_INPUT_REPORT_ID; // 0x30
	const u16 report_len = PIUIO_INPUT_SIZE_NEW; // 32
	const u16 wValue = (HID_INPUT_REPORT << 8) | report_id; // 0x0130

	// Basic null check
	if (!piu || !piu->udev || !piu->hdev || !buffer) {
		pr_err("piuio: Invalid arguments to piuio_get_input_report\n");
		return -EINVAL;
	}
	// Ensure buffer is large enough for expected report
	if (buf_len < report_len) {
		pr_err("piuio: Input buffer too small (%zu bytes) for expected report (%u bytes)\n",
		       buf_len, report_len);
		return -EINVAL; // Prevent overflow
	}

	// Parameters based on sniff: reqType=0xA1, req=1 (GET_REPORT), val=0x0130, idx=iface, len=32
	ret = usb_control_msg(
		piu->udev,
		usb_rcvctrlpipe(piu->udev, 0),
		HID_REQ_GET_REPORT, // 0x01
		USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // 0xA1
		wValue,     // Report Type (Input) | Report ID (0x30)
		piu->iface, // wIndex (Interface 0)
		buffer,     // data buffer
		report_len, // wLength (request exactly 32 bytes)
		msecs_to_jiffies(50)); // Short timeout for polling

	if (ret < 0 && ret != -ESHUTDOWN && ret != -ENODEV && ret != -EPIPE && ret != -ETIMEDOUT) {
		 hid_warn(piu->hdev, "GET_REPORT (ID 0x%02x) failed: %d\n", report_id, ret);
	} else if (ret >= 0 && ret != report_len) {
        hid_warn(piu->hdev, "GET_REPORT (ID 0x%02x) returned %d bytes (expected %u)\n",
                 report_id, ret, report_len);
        return -EIO; // Treat partial reads as errors
    } else if (ret == report_len) {
		hid_dbg(piu->hdev, "GET_REPORT (ID 0x%02x) successful (%d bytes)\n", report_id, ret);
	}

	return ret; // Return bytes read (32) or negative error code
}

/**
 * piuio_process_polled_input - Process a received input report buffer.
 * Feeds the raw report buffer (assumed 32 bytes) to the HID core via hid_input_report.
 */
static void piuio_process_polled_input(struct piuio *piu, u8 *data, int size)
{
	// Check pointers passed from timer callback
	if (!piu || !piu->hdev || !piu->idev || !data) {
		pr_err("piuio: Invalid state/data in process_polled_input\n");
		return;
	}

    // We expect exactly PIUIO_INPUT_SIZE_NEW (32) bytes
    if (size != PIUIO_INPUT_SIZE_NEW) {
        hid_dbg(piu->hdev, "Ignoring input report with unexpected size %d (expected %d)\n",
                size, PIUIO_INPUT_SIZE_NEW);
        return;
    }

	hid_dbg(piu->hdev, "Submitting %d byte input report to HID core\n", size);

	// Feed the raw report data to the HID core.
	// The '0' indicates this data did not come from an interrupt pipe.
	hid_input_report(piu->hdev, HID_INPUT_REPORT, data, size, 0);
}


/**
 * piuio_timer_cb - Timer callback function for polling input reports.
 */
static enum hrtimer_restart piuio_timer_cb(struct hrtimer *timer)
{
	struct piuio *piu = container_of(timer, struct piuio, timer);
	int size;

	// Check critical pointers
	if (!piu || !piu->hdev || !piu->idev || !piu->udev) {
		pr_err("piuio: Invalid state in timer callback (piu=%p)\n", piu);
		return HRTIMER_NORESTART; // Stop timer
	}

	size = piuio_get_input_report(piu, piu->in_buf, PIUIO_INPUT_BUF_SIZE);

	if (size == PIUIO_INPUT_SIZE_NEW) { // Check for exact expected size on success
		// Process if data was received successfully
		piuio_process_polled_input(piu, piu->in_buf, size);
	} else if (size >= 0) {
        hid_dbg(piu->hdev, "Ignoring input report fragment (%d bytes)\n", size);
    } else {
		// Error occurred during polling
		if (size == -ENODEV || size == -ESHUTDOWN) {
			hid_info(piu->hdev, "Device disconnected, stopping timer.\n");
			return HRTIMER_NORESTART; // Stop timer if device gone
		}
		hid_dbg(piu->hdev, "Polling failed with error %d, continuing timer.\n", size);
	}

	// Reschedule the timer for the next poll
	hrtimer_forward_now(timer, ms_to_ktime(poll_interval_ms));
	return HRTIMER_RESTART;
}


#ifdef CONFIG_LEDS_CLASS // --- Wrap LED related functions ---

/**
 * piuio_output_report_interrupt_complete - Completion handler for interrupt OUT URB.
 */
static void piuio_output_report_interrupt_complete(struct urb *urb)
{
	struct piuio *piu = urb->context;
	int status = urb->status;
	unsigned long flags;

	if (!piu) {
		pr_err("piuio: Invalid context in output URB completion!\n");
		return;
	}

	// Clear the output active flag under lock
	spin_lock_irqsave(&piu->output_submit_lock, flags);
	atomic_set(&piu->output_active, 0);
	spin_unlock_irqrestore(&piu->output_submit_lock, flags);

	hid_dbg(piu->hdev, "Interrupt OUT URB completed with status %d\n", status);

	// Log unexpected errors
	if (status && status != -ENOENT && status != -ECONNRESET && status != -ESHUTDOWN) {
		hid_warn(piu->hdev, "Interrupt OUT URB unexpected status %d\n", status);
	}
}

/**
 * piuio_send_output_interrupt - Send LED state via Interrupt OUT endpoint (for 1020).
 * Sends Report ID 0x13, total 16 bytes.
 */
static int piuio_send_output_interrupt(struct piuio *piu)
{
	int ret = 0;
	int i;
	unsigned long flags_led;
	unsigned long flags_submit;

	// Basic validation
	if (!piu || !piu->output_urb || !piu->output_buf || !piu->udev || !piu->hdev) {
		pr_err("piuio: Invalid state in piuio_send_output_interrupt (piu=%p, urb=%p, buf=%p)\n",
			   piu, piu ? piu->output_urb : NULL, piu ? piu->output_buf : NULL);
		return -EFAULT;
	}
    // Check if output pipe was successfully initialized
    if (!piu->output_pipe) {
        hid_warn_once(piu->hdev, "Cannot send output report, output pipe not initialized (likely missing endpoint)\n");
        return -ENODEV;
    }

	hid_dbg(piu->hdev, "Attempting to send Interrupt OUT report (ID 0x%02x)\n", PIUIO_OUTPUT_REPORT_ID);

	// Protect against concurrent submissions using spinlock and atomic flag
	spin_lock_irqsave(&piu->output_submit_lock, flags_submit);
	if (atomic_read(&piu->output_active)) {
		// Already active, skip this submission
		spin_unlock_irqrestore(&piu->output_submit_lock, flags_submit);
		hid_dbg(piu->hdev, "Output URB already active, skipping send\n");
		return 0; // Not an error, just busy
	}
	// Mark as active *before* unlocking to prevent race
	atomic_set(&piu->output_active, 1);
	spin_unlock_irqrestore(&piu->output_submit_lock, flags_submit);

	// Prepare the 16-byte buffer
	memset(piu->output_buf, 0, PIUIO_OUTPUT_SIZE_NEW);

	// Set Report ID
	piu->output_buf[0] = PIUIO_OUTPUT_REPORT_ID; // 0x13

	// Map LED state to bytes 1-15
	spin_lock_irqsave(&piu->led_lock, flags_led);
    memset(piu->output_buf + 1, 0, PIUIO_OUTPUT_SIZE_NEW - 1); // Clear payload
    for (i = 0; i < PIUIO_MAX_LEDS; i++) {
        if (piu->led_shadow[i]) {
            int byte_idx = 1 + (i / 8); // Target byte index (1 to 15)
            int bit_idx = i % 8;        // Target bit index (0 to 7)
            if (byte_idx < PIUIO_OUTPUT_SIZE_NEW) { // Check within buffer bounds
                 piu->output_buf[byte_idx] |= (1 << bit_idx);
            } else {
                 hid_warn_once(piu->hdev, "LED index %d maps out of output buffer bounds\n", i);
            }
        }
    }
	spin_unlock_irqrestore(&piu->led_lock, flags_led);


	// Fill and submit the URB
	usb_fill_int_urb(piu->output_urb, piu->udev, piu->output_pipe,
					 piu->output_buf, PIUIO_OUTPUT_SIZE_NEW, // Send all 16 bytes
					 piuio_output_report_interrupt_complete, piu,
					 1); // Interval ignored for OUT

	hid_dbg(piu->hdev, "Submitting Interrupt OUT URB (size %d)\n", PIUIO_OUTPUT_SIZE_NEW);
	ret = usb_submit_urb(piu->output_urb, GFP_ATOMIC);
	if (ret) {
		hid_err(piu->hdev, "Failed to submit interrupt OUT URB: %d\n", ret);
		// Clear active flag on submission failure
		spin_lock_irqsave(&piu->output_submit_lock, flags_submit);
		atomic_set(&piu->output_active, 0);
		spin_unlock_irqrestore(&piu->output_submit_lock, flags_submit);
	} else {
		hid_dbg(piu->hdev, "Interrupt OUT URB submitted successfully\n");
	}

	return ret;
}

/**
 * piuio_led_set - Callback function to set LED brightness.
 */
static int piuio_led_set(struct led_classdev *cdev,
						  enum led_brightness b)
{
	struct piuio_led *ld = container_of(cdev, struct piuio_led, cdev);
	struct piuio *piu = ld->piu;
	int ret = -EOPNOTSUPP;
	unsigned long flags;

	// Check piu and associated hid device
	if (!piu || !piu->hdev) {
		pr_err("piuio: Invalid state in led_set (piu=%p)\n", piu);
		return -ENODEV;
	}

	// Check LED index bounds
	if (ld->idx >= PIUIO_MAX_LEDS) {
		hid_warn(piu->hdev, "LED index %d out of range\n", ld->idx);
		return -EINVAL;
	}

	hid_dbg(piu->hdev, "Setting LED %d to brightness %d\n", ld->idx, b);

	// Update shadow state under lock
	spin_lock_irqsave(&piu->led_lock, flags);
	piu->led_shadow[ld->idx] = (b > 0);
	spin_unlock_irqrestore(&piu->led_lock, flags);

	// Trigger appropriate output mechanism based on device
	if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
		hid_dbg(piu->hdev, "Using Interrupt OUT for LED set\n");
		ret = piuio_send_output_interrupt(piu);
	} else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
		hid_warn_once(piu->hdev, "Legacy output for 1010 device not implemented.\n");
        ret = -EOPNOTSUPP;
	} else {
		hid_warn(piu->hdev, "LED set called for unknown product ID %04x\n", piu->hdev->product);
		ret = -ENODEV;
	}

	hid_dbg(piu->hdev, "LED set for index %d returned %d\n", ld->idx, ret);
	// Return 0 for success or if busy/queued, error otherwise
	return (ret < 0) ? ret : 0;
}
#endif // CONFIG_LEDS_CLASS


/**
 * piuio_dev_write - Handle writes to the legacy /dev/piuioX device.
 */
static ssize_t piuio_dev_write(struct file *filp,
							   const char __user *buf,
							   size_t len, loff_t *off)
{
	// Get misc device struct from file pointer
	struct miscdevice *misc = filp->private_data;
	// Retrieve the associated piu struct using the underlying device data function
	struct piuio *piu = dev_get_drvdata(misc->this_device);
#ifdef CONFIG_LEDS_CLASS // Variables only needed if LEDs enabled
	const size_t legacy_bits = PIUIO_LEGACY_SIZE * 8;
	unsigned long flags;
	u8 *tmp = NULL;
	int pin;
	int ret = 0;
	int first_error = 0;
#endif

	// Check misc first, as piu depends on it being valid
	if (!misc || !misc->this_device) {
		pr_err("piuio: Invalid misc device data in dev_write (filp->private_data or this_device is NULL)\n");
		return -ENODEV;
	}
	// Check piu and associated hid device
	if (!piu || !piu->hdev) {
		pr_err("piuio: Invalid state in dev_write (piu=%p, hdev=%p) for device %s\n",
		       piu, piu ? piu->hdev : NULL, misc->name);
		return -ENODEV;
	}

	hid_dbg(piu->hdev, "Write to legacy device /dev/%s, len=%zu\n", piu->misc.name, len);

	// Check for expected legacy write size
	if (len != PIUIO_LEGACY_SIZE) {
		hid_err(piu->hdev, "Legacy write invalid length %zu (expected %d)\n", len, PIUIO_LEGACY_SIZE);
		return -EINVAL;
	}

#ifndef CONFIG_LEDS_CLASS
    hid_warn_once(piu->hdev, "Legacy write accepted but ignored (CONFIG_LEDS_CLASS disabled)\n");
    return len; // Pretend success
#else
	// Allocate temporary buffer
	tmp = kmalloc(PIUIO_LEGACY_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	// Copy data from userspace
	if (copy_from_user(tmp, buf, len)) {
		kfree(tmp);
		return -EFAULT;
	}

	// Update shadow state under lock based on received bitmap
	spin_lock_irqsave(&piu->led_lock, flags);
	for (pin = 0; pin < legacy_bits && pin < PIUIO_MAX_LEDS; pin++) {
		piu->led_shadow[pin] = !!(tmp[pin / 8] & (1 << (pin % 8)));
	}
	spin_unlock_irqrestore(&piu->led_lock, flags);

	kfree(tmp); // Free temporary buffer

	// Trigger appropriate output mechanism
	if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
		hid_dbg(piu->hdev, "Updating LEDs via Interrupt OUT due to legacy write\n");
		ret = piuio_send_output_interrupt(piu);
		if (ret < 0)
			first_error = ret;
	} else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
		hid_warn_once(piu->hdev, "Legacy output for 1010 device not implemented.\n");
        first_error = -EOPNOTSUPP;
	} else {
		hid_err(piu->hdev, "Cannot update LEDs for unknown product ID %04x via legacy write\n", piu->hdev->product);
		first_error = -ENODEV;
	}

	hid_dbg(piu->hdev, "Legacy write processing finished, result=%d\n", first_error ? first_error : (int)len);
	// Return length on success, or the first error encountered
	return first_error ? first_error : len;
#endif // CONFIG_LEDS_CLASS
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
 */
static int piuio_probe(struct hid_device *hdev,
					   const struct hid_device_id *id)
{
	struct piuio *piu;
	struct input_dev *idev;
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *ep_out = NULL;
	int i, ret;
#ifdef CONFIG_LEDS_CLASS
	int registered_leds = 0;
#endif

	hid_info(hdev, "Probing PIUIO device %04X:%04X\n", id->vendor, id->product);

	intf = to_usb_interface(hdev->dev.parent);
	if (!intf) {
		hid_err(hdev, "Cannot get USB interface\n");
		return -ENODEV;
	}

	piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
	if (!piu) {
		hid_err(hdev, "Failed to allocate piu struct\n");
		return -ENOMEM;
	}
	hid_set_drvdata(hdev, piu);
	hid_info(hdev, "Allocated piu struct: %p (size %zu)", piu, sizeof(*piu));

	piu->hdev = hdev;
	piu->udev = interface_to_usbdev(intf);
	if (!piu->udev) {
		hid_err(hdev, "Cannot get USB device\n");
		ret = -ENODEV;
		goto err_free_piu;
	}
	piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;
#ifdef CONFIG_LEDS_CLASS
	spin_lock_init(&piu->led_lock);
#endif
	spin_lock_init(&piu->output_submit_lock);
	atomic_set(&piu->output_active, 0);

	// Find Interrupt OUT Endpoint
	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *ep = &intf->cur_altsetting->endpoint[i].desc;
		if (!ep_out && usb_endpoint_is_int_out(ep)) {
			ep_out = ep;
			hid_info(hdev, "Found Interrupt OUT endpoint: addr=0x%02x, size=%d\n",
					 ep->bEndpointAddress, usb_endpoint_maxp(ep));
			break;
		}
	}

	// Input device setup
	snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys);
	idev = devm_input_allocate_device(&hdev->dev);
	if (!idev) {
		hid_err(hdev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_free_piu;
	}
	piu->idev = idev;
	hid_info(hdev, "Allocated input device: %p", idev);

	idev->name = hdev->name;
	idev->phys = piu->phys;
	idev->id.bustype = hdev->bus;
	idev->id.vendor  = hdev->vendor;
	idev->id.product = hdev->product;
	idev->id.version = hdev->version;
	idev->dev.parent = &hdev->dev;

	set_bit(EV_KEY, idev->evbit);
	for (i = 0; i < PIUIO_NUM_INPUTS; i++) {
	    int keycode = piuio_keycode(i);
	    if (keycode >= 0)
		    set_bit(keycode, idev->keybit);
    }
	set_bit(EV_MSC, idev->evbit);
	set_bit(MSC_SCAN, idev->mscbit);

	ret = input_register_device(idev);
	if (ret) {
		hid_err(hdev, "Failed to register input device: %d\n", ret);
		goto err_free_piu;
	}
	hid_info(hdev, "Registered input device %s", idev->name);

#ifdef CONFIG_LEDS_CLASS // --- LED setup block ---
	piu->led = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS, sizeof(*piu->led), GFP_KERNEL);
	if (!piu->led) {
		hid_err(hdev, "Failed to allocate LED structures\n");
		ret = -ENOMEM;
		goto err_unregister_input;
	}
	hid_info(hdev, "Allocated %d LED structures", PIUIO_MAX_LEDS);

	for (i = 0; i < PIUIO_MAX_LEDS; i++) {
		piu->led[i].piu = piu;
		piu->led[i].idx = i;
		piu->led[i].cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s::output%u", dev_name(&hdev->dev), i);
		if (!piu->led[i].cdev.name) {
			hid_err(hdev, "Failed to allocate name for LED %d\n", i);
			ret = -ENOMEM;
			registered_leds = i;
			goto err_unregister_leds;
		}
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


	// Setup for Interrupt OUT URB (1020)
	if (id->product == USB_PRODUCT_ID_BTNBOARD_NEW) {
		if (!ep_out) {
			hid_warn(hdev, "Interrupt OUT endpoint not found for 1020 device.\n");
		} else if (IS_ENABLED(CONFIG_LEDS_CLASS) && ep_out->bEndpointAddress != PIUIO_INT_OUT_EP) {
            hid_warn(hdev, "Interrupt OUT endpoint 0x%02x found, expected 0x%02x for 1020 device LED output.\n",
                     ep_out->bEndpointAddress, PIUIO_INT_OUT_EP);
        }
		piu->output_buf = devm_kzalloc(&hdev->dev, PIUIO_OUTPUT_SIZE_NEW, GFP_KERNEL);
		piu->output_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!piu->output_buf || !piu->output_urb) {
			hid_err(hdev, "Failed to allocate output URB/buffer\n");
			usb_free_urb(piu->output_urb);
			ret = -ENOMEM;
			#ifdef CONFIG_LEDS_CLASS
			goto err_unregister_leds;
			#else
			goto err_unregister_input;
			#endif
		}
		if (ep_out) {
			piu->output_pipe = usb_sndintpipe(piu->udev, ep_out->bEndpointAddress);
			hid_info(hdev, "Allocated URB %p and buffer %p for Interrupt OUT EP 0x%02x\n",
					 piu->output_urb, piu->output_buf, ep_out->bEndpointAddress);
		} else {
			hid_warn(hdev, "Output URB/buffer allocated, but no valid endpoint found.\n");
            piu->output_pipe = 0; // Indicate invalid pipe
		}
	}

	// Legacy Workqueue setup REMOVED

	// Start HID Hardware Layer (HIDRAW)
	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "hid_hw_start failed with HID_CONNECT_HIDRAW: %d\n", ret);
		if (piu->output_urb) goto err_free_output_urb;
		#ifdef CONFIG_LEDS_CLASS
		goto err_unregister_leds;
		#else
		goto err_unregister_input;
		#endif
	}
	hid_info(hdev, "hid_hw_start successful with HID_CONNECT_HIDRAW");


	// Parse HID Reports
	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "hid_parse failed: %d\n", ret);
		goto err_stop_hid;
	}
	hid_info(hdev, "hid_parse successful");

	// Open HID Device Communication Channel
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hid_hw_open failed: %d\n", ret);
		goto err_stop_hid;
	}
	hid_info(hdev, "hid_hw_open successful");

	// Send SET_IDLE Request
	ret = usb_control_msg(piu->udev,
						  usb_sndctrlpipe(piu->udev, 0),
						  HID_REQ_SET_IDLE,
						  USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
						  0, // Duration = 0 (infinite)
						  piu->iface, // Interface
						  NULL, 0, // No data
						  msecs_to_jiffies(100));
	if (ret < 0 && ret != -EPIPE) {
		hid_warn(hdev, "SET_IDLE failed: %d (continuing)\n", ret);
		ret = 0;
	} else {
		hid_info(hdev, "SET_IDLE sent successfully (or stalled)\n");
		ret = 0;
	}


	// Initialize and Start Polling Timer
	hrtimer_init(&piu->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	piu->timer.function = piuio_timer_cb;
	hrtimer_start(&piu->timer, ms_to_ktime(poll_interval_ms), HRTIMER_MODE_REL);
	hid_info(hdev, "Started input polling timer (interval %d ms)\n", poll_interval_ms);


	// Misc device setup
	piu->misc.minor = MISC_DYNAMIC_MINOR;
	piu->misc_name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "piuio-%s", dev_name(&hdev->dev));
	if (!piu->misc_name) {
		hid_err(hdev, "Failed to allocate misc device name string\n");
		ret = -ENOMEM;
		goto err_cancel_timer;
	}
	piu->misc.name = piu->misc_name;
	piu->misc.fops = &piuio_fops;
	piu->misc.parent = &hdev->dev;

	ret = misc_register(&piu->misc);
	if (ret) {
		hid_err(hdev, "Failed to register misc device '%s': %d\n", piu->misc.name, ret);
		goto err_cancel_timer;
	}
	dev_set_drvdata(piu->misc.this_device, piu);
	hid_info(hdev, "Registered misc device /dev/%s\n", piu->misc.name);

	hid_info(hdev, "PIUIO HID Driver probe completed successfully for /dev/%s\n", piu->misc.name);
	return 0; // Success!

/* --- Error Handling Cleanup: Unwind in reverse order --- */
err_cancel_timer:
	hrtimer_cancel(&piu->timer);
err_stop_hid:
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
// err_destroy_workqueue: // REMOVED
err_free_output_urb:
	usb_free_urb(piu->output_urb);
#ifdef CONFIG_LEDS_CLASS
err_unregister_leds:
	hid_info(hdev, "Unregistering %d LEDs due to probe error\n", registered_leds);
	for (i = 0; i < registered_leds; i++) {
		if (piu->led && piu->led[i].cdev.name) {
			led_classdev_unregister(&piu->led[i].cdev);
		}
	}
	// Fall through
#endif
err_unregister_input:
	if (piu->idev) {
		input_unregister_device(piu->idev);
	}
err_free_piu:
	hid_info(hdev, "PIUIO HID Driver probe failed (%d)\n", ret);
	return ret;
}


/**
 * piuio_remove - Called when the HID device is removed or driver unloaded.
 */
static void piuio_remove(struct hid_device *hdev)
{
	struct piuio *piu = hid_get_drvdata(hdev);
#ifdef CONFIG_LEDS_CLASS
	int i;
#endif

	if (!piu) {
		hid_warn(hdev, "piuio_remove called with NULL drvdata\n");
		return;
	}

	if (piu->misc.name) {
		hid_info(hdev, "Unregistering PIUIO HID Driver for %s (/dev/%s)\n",
		         dev_name(&hdev->dev), piu->misc.name);
	} else {
	    hid_info(hdev, "Unregistering PIUIO HID Driver for %s (misc name unknown)\n",
	             dev_name(&hdev->dev));
	}

#ifdef CONFIG_LEDS_CLASS // --- Wrap LED unregistration ---
	// Unregister LEDs
    hid_info(hdev, "Unregistering LED class devices...");
	if (piu->led) {
		for (i = 0; i < PIUIO_MAX_LEDS; i++) {
			if (piu->led[i].cdev.name) {
				led_classdev_unregister(&piu->led[i].cdev);
			}
		}
	}
	hid_info(hdev, "LEDs unregistered.");
#endif // CONFIG_LEDS_CLASS

	// Deregister misc device
    if (piu->misc.name) {
		hid_info(hdev, "Deregistering misc device /dev/%s...", piu->misc.name);
		misc_deregister(&piu->misc);
		hid_info(hdev, "Misc device deregistered.");
	}

	// Cancel timer
    hid_info(hdev, "Cancelling input timer...");
	hrtimer_cancel(&piu->timer);
	hid_info(hdev, "Input timer cancelled.");

	// Kill pending output URB and free it
    if (piu->output_urb) {
		hid_info(hdev, "Killing and freeing output URB %p...", piu->output_urb);
		usb_kill_urb(piu->output_urb);
		usb_free_urb(piu->output_urb);
		piu->output_urb = NULL;
		hid_info(hdev, "Output URB killed and freed.");
	}

	// Destroy workqueue - REMOVED

	// Close and stop HID hardware layer
    hid_info(hdev, "Closing HID hardware...");
	hid_hw_close(hdev);
	hid_info(hdev, "HID hardware closed.");

	hid_info(hdev, "Stopping HID hardware...");
	hid_hw_stop(hdev);
	hid_info(hdev, "HID hardware stopped.");

	// Unregister input device
    if (piu->idev) {
		hid_info(hdev, "Unregistering input device...");
		input_unregister_device(piu->idev);
		piu->idev = NULL;
		hid_info(hdev, "Input device unregistered.");
	}

	// Remaining devm resources cleaned up automatically
	hid_info(hdev, "PIUIO HID Driver remove finished for %s.\n", dev_name(&hdev->dev));
}

// --- ID Table Includes Both Devices ---
static const struct hid_device_id piuio_ids[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_LEGACY) }, // 0x0d2f:0x1010
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_NEW) },    // 0x0d2f:0x1020
	{ } // Terminator entry
};
MODULE_DEVICE_TABLE(hid, piuio_ids);

// --- HID Driver Structure Definition ---
static struct hid_driver piuio_driver = {
	.name     = "piuio_hid", // Driver name
	.id_table = piuio_ids,   // Supported device IDs
	.probe    = piuio_probe, // Probe function
	.remove   = piuio_remove,// Remove function
};

// Register the driver with the HID subsystem
module_hid_driver(piuio_driver);

// Module Metadata
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO HID interface driver");
MODULE_LICENSE("GPL v2");