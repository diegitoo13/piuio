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

	// NOTE: Removed check for !buffer as it's likely an array within piu

	// Parameters from logs/python: reqType=0xA1, req=1, val=0x0301, idx=iface
	ret = usb_control_msg(
		piu->udev,
		usb_rcvctrlpipe(piu->udev, 0),
		HID_REQ_GET_REPORT, // 0x01
		USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // 0xA1
		(HID_INPUT_REPORT << 8) | PIUIO_INPUT_REPORT_ID, // wValue (Report Type = Input | Report ID = 1)
		piu->iface, // wIndex (Interface)
		buffer,     // data buffer (address of embedded array)
		buf_len,    // max length to read
		msecs_to_jiffies(50)); // Short timeout for polling

	if (ret < 0 && ret != -ESHUTDOWN && ret != -ENODEV && ret != -EPIPE && ret != -ETIMEDOUT) {
		 hid_warn(piu->hdev, "GET_REPORT failed: %d\n", ret);
	} else if (ret >= 0) {
		hid_dbg(piu->hdev, "GET_REPORT received %d bytes\n", ret);
	}

	return ret;
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

	hid_dbg(piu->hdev, "Sending SET_REPORT(ctrl) for ID 0x%02x (bank %d)\n", rptid, bank);

	buf = kmalloc(PIUIO_OUTPUT_CHUNK + 1, GFP_ATOMIC);
	if (!buf) {
		hid_err(piu->hdev, "Failed to allocate buffer for SET_REPORT(ctrl)\n");
		return -ENOMEM;
	}

	buf[0] = rptid; // Report ID
	spin_lock(&piu->lock);
	// Assume led_shadow is valid array if piu is valid
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
	}

	return 0;
}

/**
 * piuio_output_report_interrupt_complete - Completion handler for interrupt OUT URB.
 * @urb: The completed URB.
 */
static void piuio_output_report_interrupt_complete(struct urb *urb)
{
	struct piuio *piu = urb->context;
	int status = urb->status;

	if (!piu) {
		pr_err("piuio: Invalid context in output URB completion!\n");
		return;
	}

	hid_dbg(piu->hdev, "Interrupt OUT URB completed with status %d\n", status);

	if (status && status != -ENOENT && status != -ECONNRESET && status != -ESHUTDOWN) {
		hid_warn(piu->hdev, "Interrupt OUT URB unexpected status %d\n", status);
	}

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

	hid_dbg(piu->hdev, "Attempting to send Interrupt OUT report\n");

	// Check only pointers that *might* be NULL (output_urb isn't devm allocated)
	if (!piu || !piu->output_urb) {
		pr_err("piuio: Invalid state in piuio_send_output_interrupt (piu=%p, urb=%p)\n",
			   piu, piu ? piu->output_urb : NULL);
		// Cannot proceed without URB if needed
		return -EFAULT;
	}
	// output_buf and led_shadow are likely arrays, assume valid if piu is valid

	if (!mutex_trylock(&piu->output_mutex)) {
		hid_dbg(piu->hdev, "Output mutex busy, skipping send\n");
		return -EBUSY;
	}

	// output_buf is assumed to be an array within piu now
	memset(piu->output_buf, 0, PIUIO_OUTPUT_SIZE_NEW);
	spin_lock(&piu->lock);
	// led_shadow is assumed to be an array within piu now
	for (i = 0; i < PIUIO_MAX_LEDS && i < (PIUIO_OUTPUT_SIZE_NEW * 8); i++) {
		if (piu->led_shadow[i]) {
			__set_bit(i, (unsigned long *)piu->output_buf);
		}
	}
	spin_unlock(&piu->lock);


	pipe = usb_sndintpipe(piu->udev, 0x02);

	usb_fill_int_urb(piu->output_urb, piu->udev, pipe,
					 piu->output_buf, PIUIO_OUTPUT_SIZE_NEW, // Pass address of array
					 piuio_output_report_interrupt_complete, piu,
					 1);

	hid_dbg(piu->hdev, "Submitting Interrupt OUT URB (size %d)\n", PIUIO_OUTPUT_SIZE_NEW);
	ret = usb_submit_urb(piu->output_urb, GFP_ATOMIC);
	if (ret) {
		hid_err(piu->hdev, "Failed to submit interrupt OUT URB: %d\n", ret);
		mutex_unlock(&piu->output_mutex);
	} else {
		hid_dbg(piu->hdev, "Interrupt OUT URB submitted successfully\n");
	}

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
	// Use pointer directly to embedded array
	const unsigned long *current_data_bits = (unsigned long *)piu->in_buf;
	int bit_index;

	// Check only piu and pointers known to be potentially NULL (idev?)
	if (!piu || !piu->hdev || !piu->idev) {
		pr_err("piuio: Invalid state in timer callback! Aborting poll. (piu=%p, hdev=%p, idev=%p)\n",
			   piu, piu ? piu->hdev : NULL, piu ? piu->idev : NULL);
		return HRTIMER_NORESTART;
	}
	// Assume in_buf and prev_inputs are valid arrays if piu is valid

	hid_dbg(piu->hdev, "Timer callback entered\n");

	size = piuio_get_input_report(piu, piu->in_buf, PIUIO_INPUT_BUF_SIZE);

	if (size < 0) {
		goto reschedule;
	}
	if (size == 0) {
		hid_dbg(piu->hdev, "Timer poll received 0 bytes\n");
		goto reschedule;
	}

	if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
		if (size != PIUIO_INPUT_SIZE_NEW) {
			 hid_dbg(piu->hdev, "Ignoring polled 1020 input with size %d (expected %d)\n", size, PIUIO_INPUT_SIZE_NEW);
			 goto reschedule;
		}
		max_inputs_in_report = size * 8;
		hid_dbg(piu->hdev, "Processing 1020 input (%d bytes)\n", size);

	} else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
		hid_warn(piu->hdev, "Ignoring polled input for legacy 1010 device (size %d) - format unknown\n", size);
		goto reschedule;

	} else {
		hid_warn(piu->hdev, "Ignoring polled input for unknown device %04X:%04X (size %d)\n",
				 piu->hdev->vendor, piu->hdev->product, size);
		goto reschedule;
	}


	inputs_to_process = min(max_inputs_in_report, PIUIO_NUM_INPUTS);
	num_longs = BITS_TO_LONGS(inputs_to_process);

	hid_dbg(piu->hdev, "Comparing %d input bits (%d longs)\n", inputs_to_process, num_longs);

	for (i = 0; i < num_longs; i++) {
		// Bounds check
		if (((void*)piu->in_buf + (i * sizeof(unsigned long))) >= ((void*)piu->in_buf + size)) {
			hid_warn(piu->hdev, "Bounds check failed reading input state (long %d)\n", i);
			break;
		}
		unsigned long current_bits = current_data_bits[i];
		unsigned long changed_bits = current_bits ^ piu->prev_inputs[i]; // Access embedded arrays

		if (changed_bits != 0) {
			hid_dbg(piu->hdev, "Input change detected in long %d (changed=0x%lx)\n", i, changed_bits);
			state_changed = true;
			for_each_set_bit(bit_index, &changed_bits, BITS_PER_LONG) {
				 int global_bit_index = i * BITS_PER_LONG + bit_index;
				 if (global_bit_index >= inputs_to_process) break;

				 bool pressed = !!test_bit(global_bit_index, current_data_bits);
				 hid_dbg(piu->hdev, "Input bit %d changed to %d\n", global_bit_index, pressed);

				 input_event(piu->idev, EV_MSC, MSC_SCAN, global_bit_index + 1);
				 input_report_key(piu->idev, piuio_keycode(global_bit_index), pressed);
			}
			 piu->prev_inputs[i] = current_bits; // Update embedded array
		}
	}

	if (state_changed) {
		hid_dbg(piu->hdev, "Sending input_sync\n");
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
	int ret = -EOPNOTSUPP;

	if (!piu || !piu->hdev) {
		pr_err("piuio: Invalid state in led_set (piu=%p)\n", piu);
		return -ENODEV;
	}

	if (ld->idx >= PIUIO_MAX_LEDS) {
		hid_warn(piu->hdev, "LED index %d out of range\n", ld->idx);
		return -EINVAL;
	}

	hid_dbg(piu->hdev, "Setting LED %d to brightness %d\n", ld->idx, b);

	spin_lock(&piu->lock);
	// Assume led_shadow is a valid embedded array
	piu->led_shadow[ld->idx] = (b > 0);
	spin_unlock(&piu->lock);

	if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
		hid_dbg(piu->hdev, "Using Interrupt OUT for LED set\n");
		ret = piuio_send_output_interrupt(piu);
	} else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
		hid_dbg(piu->hdev, "Using legacy control transfer for LED set\n");
		ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (ld->idx / PIUIO_OUTPUT_CHUNK));
	} else {
		hid_warn(piu->hdev, "LED set called for unknown product ID %04x\n", piu->hdev->product);
	}

	hid_dbg(piu->hdev, "LED set for index %d returned %d\n", ld->idx, ret);
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
	u8 *tmp = NULL;
	int pin, ret = 0;
	int first_error = 0;

	if (!piu || !piu->hdev) {
		pr_err("piuio: Invalid state in dev_write (piu=%p)\n", piu);
		return -ENODEV;
	}
	// Assume led_shadow array is valid if piu is valid

	hid_dbg(piu->hdev, "Write to legacy device /dev/%s, len=%zu\n", piu->misc.name, len);

	if (len != PIUIO_LEGACY_SIZE) {
		hid_err(piu->hdev, "Legacy write invalid length %zu (expected %d)\n", len, PIUIO_LEGACY_SIZE);
		return -EINVAL;
	}

	tmp = kmalloc(PIUIO_LEGACY_SIZE, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	if (copy_from_user(tmp, buf, len)) {
		kfree(tmp);
		return -EFAULT;
	}

	spin_lock(&piu->lock);
	memset(piu->led_shadow, 0, min_t(size_t, legacy_bits, PIUIO_MAX_LEDS));
	for (pin = 0; pin < legacy_bits && pin < PIUIO_MAX_LEDS; pin++)
		piu->led_shadow[pin] = !!(tmp[pin/8] & (1 << (pin%8)));
	spin_unlock(&piu->lock);

	kfree(tmp);

	if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
		hid_dbg(piu->hdev, "Updating LEDs via Interrupt OUT due to legacy write\n");
		ret = piuio_send_output_interrupt(piu);
		if (ret < 0) first_error = ret;
	} else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
		hid_dbg(piu->hdev, "Updating LEDs via legacy control transfer due to legacy write\n");
		for (pin = 0; pin < legacy_bits && pin < PIUIO_MAX_LEDS; pin += PIUIO_OUTPUT_CHUNK) {
			 ret = piuio_set_report_legacy_ctrl(piu, PIUIO_RPT_OUT_BASE + (pin / PIUIO_OUTPUT_CHUNK));
			 if (ret < 0 && first_error == 0) {
				  first_error = ret;
			 }
		}
	} else {
		 hid_err(piu->hdev, "Cannot update LEDs for unknown product ID %04x via legacy write\n", piu->hdev->product);
		 first_error = -EOPNOTSUPP;
	}

	hid_dbg(piu->hdev, "Legacy write processing finished, result=%d\n", first_error ? first_error : (int)len);
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
	// Removed calculation for prev_inputs_size as it's an array now

	hid_info(hdev, "Probing PIUIO device %04X:%04X\n", id->vendor, id->product);

	intf = to_usb_interface(hdev->dev.parent);
	if (!intf) {
		hid_err(hdev,"Cannot get USB interface\n");
		return -ENODEV;
	}

	// Allocate the main struct, including embedded arrays
	piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
	if (!piu) {
		hid_err(hdev, "Failed to allocate piu struct\n");
		return -ENOMEM;
	}
	hid_set_drvdata(hdev, piu);
	hid_info(hdev, "Allocated piu struct: %p (size %zu)", piu, sizeof(*piu));

	piu->hdev = hdev;
	piu->udev = interface_to_usbdev(intf);
	piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;
	spin_lock_init(&piu->lock);
	mutex_init(&piu->output_mutex);

	// --- Buffers in_buf and prev_inputs are now embedded arrays ---
	// --- No separate allocation needed ---
	hid_info(hdev, "Using embedded input buffer: %p (size %d)", piu->in_buf, PIUIO_INPUT_BUF_SIZE);
	hid_info(hdev, "Using embedded prev_inputs buffer: %p (size %zu longs)", piu->prev_inputs, BITS_TO_LONGS(PIUIO_NUM_INPUTS));


	// --- Input device setup ---
	snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys);
	idev = devm_input_allocate_device(&hdev->dev);
	if (!idev) {
		hid_err(hdev, "Failed to allocate input device\n");
		return -ENOMEM;
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
	for (i = 0; i < PIUIO_NUM_INPUTS; i++)
		set_bit(piuio_keycode(i), idev->keybit);
	set_bit(EV_MSC, idev->evbit);
	set_bit(MSC_SCAN, idev->mscbit);

	ret = input_register_device(idev);
	if (ret) {
		hid_err(hdev, "Failed to register input device: %d\n", ret);
		return ret;
	}
	hid_info(hdev, "Registered input device %s", idev->name);


	// --- LED class device setup ---
	piu->led = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS, sizeof(*piu->led), GFP_KERNEL);
	if (!piu->led) {
		hid_err(hdev, "Failed to allocate LED structures\n");
		return -ENOMEM;
	}
	// NOTE: Assumes led_shadow is also an embedded array within piu
	// If it's a pointer, it needs allocation here like we tried before!
	// Check piuio_hid.h for definition of struct piuio { ... u8 led_shadow[]; ... }
	hid_info(hdev, "Allocated %d LED structures", PIUIO_MAX_LEDS);
	hid_info(hdev, "Using embedded LED shadow buffer: %p (size %d)", piu->led_shadow, PIUIO_MAX_LEDS);


	for (i = 0; i < PIUIO_MAX_LEDS; i++) {
		piu->led[i].piu = piu;
		piu->led[i].idx = i;
		piu->led[i].cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s::output%u", dev_name(&hdev->dev), i);
		if (!piu->led[i].cdev.name) {
			hid_err(hdev, "Failed to allocate name for LED %d\n", i);
			return -ENOMEM;
		}
		piu->led[i].cdev.brightness_set_blocking = piuio_led_set;
		piu->led[i].cdev.max_brightness = 1;
		ret = devm_led_classdev_register(&hdev->dev, &piu->led[i].cdev);
		if (ret) {
			hid_err(hdev, "Failed to register LED %d (%s): %d\n", i, piu->led[i].cdev.name, ret);
			return ret;
		}
	}
	hid_info(hdev, "Registered %d LED class devices", PIUIO_MAX_LEDS);


	// --- Misc device setup (/dev/piuioX) ---
	piu->misc.minor  = MISC_DYNAMIC_MINOR;
	piu->misc_name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "piuio-%s", dev_name(&hdev->dev));
	if (!piu->misc_name) {
		hid_err(hdev, "Failed to allocate misc device name string\n");
		return -ENOMEM;
	}
	piu->misc.name = piu->misc_name;
	piu->misc.fops   = &piuio_fops;
	piu->misc.parent = &hdev->dev;

	ret = misc_register(&piu->misc);
	if (ret) {
		hid_err(hdev, "Failed to register misc device '%s': %d\n", piu->misc.name, ret);
		return ret;
	}
	misc_set_drvdata(&piu->misc, piu);
	hid_info(hdev, "Registered misc device /dev/%s\n", piu->misc.name);


	// --- Setup for Interrupt OUT URB (only needed if 1020 support uses it) ---
	piu->output_urb = NULL;
	// output_buf is assumed to be an embedded array now
	// piu->output_buf = NULL; // Remove this if it's an array

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *ep = &intf->cur_altsetting->endpoint[i].desc;
		if (usb_endpoint_is_int_out(ep)) {
			ep_out = ep;
			hid_info(hdev, "Found Interrupt OUT endpoint: addr=0x%02x", ep->bEndpointAddress);
			break;
		}
	}

	if (id->product == USB_PRODUCT_ID_BTNBOARD_NEW) {
		if (!ep_out || ep_out->bEndpointAddress != 0x02) {
			hid_warn(hdev, "Interrupt OUT endpoint 0x02 not found or incorrect for 1020 device. Output may not work.\n");
		} else {
			// output_buf is assumed embedded, check size if needed
			// Size check example: if (sizeof(piu->output_buf) < PIUIO_OUTPUT_SIZE_NEW) ... error ...
			piu->output_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!piu->output_urb) {
				hid_err(hdev, "Failed to allocate output URB\n");
				misc_deregister(&piu->misc);
				return -ENOMEM;
			}
			 hid_info(hdev, "Allocated URB %p for Interrupt OUT EP 0x%02x\n",
					  piu->output_urb, ep_out->bEndpointAddress);
		}
	}


	// --- Start HID Hardware Layer ---
	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_HIDINPUT);
	if (ret) {
		hid_err(hdev, "hid_hw_start failed: %d\n", ret);
		usb_free_urb(piu->output_urb);
		misc_deregister(&piu->misc);
		return ret;
	}
	hid_info(hdev, "hid_hw_start successful");


	// --- Parse HID Reports ---
	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "hid_parse failed: %d\n", ret);
		hid_hw_stop(hdev);
		usb_free_urb(piu->output_urb);
		misc_deregister(&piu->misc);
		return ret;
	}
	hid_info(hdev, "hid_parse successful");


	// --- Open HID Device Communication Channel ---
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hid_hw_open failed: %d\n", ret);
		hid_hw_stop(hdev);
		usb_free_urb(piu->output_urb);
		misc_deregister(&piu->misc);
		return ret;
	}
	hid_info(hdev, "hid_hw_open successful");


	// --- Send SET_IDLE Request ---
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
	} else {
		 hid_info(hdev, "SET_IDLE sent successfully (or stalled)\n");
	}

	// --- Initialize and Start Polling Timer ---
	hrtimer_init(&piu->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	piu->timer.function = piuio_timer_cb;
	hrtimer_start(&piu->timer, ms_to_ktime(poll_interval_ms), HRTIMER_MODE_REL);
	hid_info(hdev, "Started polling timer with interval %d ms", poll_interval_ms);


	hid_info(hdev, "PIUIO HID Driver probe completed successfully for /dev/%s\n", piu->misc.name);
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

	if (!piu) {
		hid_warn(hdev, "piuio_remove called with NULL drvdata\n");
		return;
	}

	hid_info(hdev, "Unregistering PIUIO HID Driver for %s\n", piu->misc.name ? piu->misc.name : dev_name(&hdev->dev));

	hid_info(hdev, "Cancelling hrtimer...\n");
	hrtimer_cancel(&piu->timer);
	hid_info(hdev, "Hrtimer cancelled.");

	if (piu->output_urb) {
		hid_info(hdev, "Killing and freeing output URB %p...\n", piu->output_urb);
		usb_kill_urb(piu->output_urb);
		usb_free_urb(piu->output_urb);
		hid_info(hdev, "Output URB freed.");
	}

	hid_info(hdev, "Closing HID hardware...");
	hid_hw_close(hdev);
	hid_info(hdev, "HID hardware closed.");


	hid_info(hdev, "Stopping HID hardware...");
	hid_hw_stop(hdev);
	hid_info(hdev, "HID hardware stopped.");


	hid_info(hdev, "Deregistering misc device /dev/%s...", piu->misc.name);
	misc_deregister(&piu->misc);
	hid_info(hdev, "Misc device deregistered.");


	// devm_* functions handle cleanup automatically
	hid_info(hdev, "PIUIO HID Driver remove finished.\n");
}

// --- ID Table Includes Both Devices ---
static const struct hid_device_id piuio_ids[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_LEGACY) }, // 0x0DEF:0x1010
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_NEW) },    // 0x0DEF:0x1020
	{} // Terminator entry
};
MODULE_DEVICE_TABLE(hid, piuio_ids);

// HID Driver Structure Definition
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

