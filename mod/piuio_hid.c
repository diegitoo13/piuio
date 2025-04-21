/*
 * PIUIO HID interface driver
 * Overrides generic Linux HID by matching PIUIO boards and provides
 * legacy /dev/piuio0 char-device compatibility for existing apps.
 *
 * Supports legacy (0x1010) and newer (0x1020) product IDs.
 * Input is handled via timer polling using GET_REPORT control transfers.
 * Input format for 1020 assumed 16 bytes (needs verification). 1010 unknown.
 * Output for 1020 uses Interrupt OUT endpoint (16 bytes) based on descriptor/logs.
 * Output for 1010 attempts legacy control transfer method (unverified) via workqueue.
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
#include <linux/spinlock.h> // spinlock API
#include <linux/bitops.h>   // bit operations
#include <linux/string.h>   // memset, snprintf, memcpy
#include <linux/jiffies.h>  // msecs_to_jiffies
#include <linux/bits.h>     // BITS_TO_LONGS, BITS_PER_LONG
#include <linux/hrtimer.h>  // High-resolution timer
#include <linux/ktime.h>    // ktime_t
#include <linux/workqueue.h> // Workqueues for legacy output
#include <linux/mutex.h>    // Mutex API
#include <linux/atomic.h>   // Atomic operations
#include <linux/device.h>   // For misc_get/set_drvdata prototypes

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

	// Basic null check
	if (!piu ||!piu->udev ||!piu->hdev ||!buffer) {
		pr_err("piuio: Invalid arguments to piuio_get_input_report\n");
		return -EINVAL;
	}

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
 * piuio_process_polled_input - Process a received input report buffer.
 * @piu: Device instance data.
 * @data: Buffer containing the received report data.
 * @size: Actual size of the data received.
 *
 * Parses input data based on Product ID and reports changes.
 * Format for 1020 is assumed 16 bytes, sequential bits, 0=pressed.
 * Format for 1010 is unknown and ignored.
 */
static void piuio_process_polled_input(struct piuio *piu, u8 *data, int size)
{
	int i, max_inputs_in_report = 0;
	int inputs_to_process = 0;
	int num_longs = 0;
	bool state_changed = false;
	const unsigned long *current_data_bits = NULL;
	int bit_index;

	// Check pointers passed from timer callback
	if (!piu ||!piu->idev ||!data) {
		pr_err("piuio: Invalid state/data in process_polled_input\n");
		return;
	}

	// Handle input based on device type
	if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
		if (size!= PIUIO_INPUT_SIZE_NEW) {
			hid_dbg(piu->hdev, "Ignoring polled input report with size %d (expected %d)\n",
					size, PIUIO_INPUT_SIZE_NEW);
			return;
		}
		// Assume 16 bytes received contain the input bits directly, no offset.
		current_data_bits = (unsigned long *)data;
		max_inputs_in_report = size * 8; // 128 bits
		hid_dbg(piu->hdev, "Processing polled 1020 input (%d bytes)\n", size);

	} else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
		// Input format for legacy device is unknown/unsupported
		hid_warn_once(piu->hdev, "Ignoring polled input for legacy device %04X:%04X (size %d) - format unknown\n",
					  piu->hdev->vendor, piu->hdev->product, size);
		return;
	} else {
		// Should not happen
		hid_warn_once(piu->hdev, "Ignoring polled input for unknown device %04X:%04X\n",
					  piu->hdev->vendor, piu->hdev->product);
		return;
	}

	// Limit processing based on report size and defined max inputs
	inputs_to_process = min(max_inputs_in_report, PIUIO_NUM_INPUTS);
	num_longs = BITS_TO_LONGS(inputs_to_process);

	hid_dbg(piu->hdev, "Comparing %d input bits (%d longs)\n", inputs_to_process, num_longs);

	// Compare current state with previous state and report changes
	for (i = 0; i < num_longs; i++) {
		// Check if the current long index is valid within the received data size
		if ((i + 1) * sizeof(unsigned long) > size) {
			hid_warn(piu->hdev, "Bounds check failed reading input state (long %d, size %d)\n", i, size);
			break; // Stop processing longs
		}
		unsigned long current_bits = current_data_bits[i];
		unsigned long changed_bits = current_bits ^ piu->prev_inputs[i];

		if (changed_bits!= 0) {
			hid_dbg(piu->hdev, "Input change detected in long %d (changed=0x%lx)\n", i, changed_bits);
			state_changed = true;
			for_each_set_bit(bit_index, &changed_bits, BITS_PER_LONG) {
				int global_bit_index = i * BITS_PER_LONG + bit_index;
				if (global_bit_index >= inputs_to_process)
					break;

				// Assumes 0 = pressed / bit cleared
				bool pressed =!test_bit(global_bit_index, current_data_bits);
				hid_dbg(piu->hdev, "Input bit %d changed to %d\n", global_bit_index, pressed);

				// Report event to input subsystem
				input_event(piu->idev, EV_MSC, MSC_SCAN, global_bit_index + 1);
				input_report_key(piu->idev, piuio_keycode(global_bit_index), pressed);
			}
			piu->prev_inputs[i] = current_bits; // Update previous state
		}
	}

	// Sync input state if changes occurred
	if (state_changed) {
		hid_dbg(piu->hdev, "Sending input_sync\n");
		input_sync(piu->idev);
	}
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
	int size;

	// Check critical pointers
	if (!piu ||!piu->hdev ||!piu->idev ||!piu->udev) {
		pr_err("piuio: Invalid state in timer callback (piu=%p)\n", piu);
		return HRTIMER_NORESTART; // Stop timer
	}

	size = piuio_get_input_report(piu, piu->in_buf, PIUIO_INPUT_BUF_SIZE);

	if (size >= 0) {
		// Process if data was received successfully
		piuio_process_polled_input(piu, piu->in_buf, size);
	} else {
		// Error occurred during polling
		if (size == -ENODEV || size == -ESHUTDOWN) {
			hid_info(piu->hdev, "Device disconnected, stopping timer.\n");
			return HRTIMER_NORESTART; // Stop timer if device gone
		}
		// Continue polling despite other transient errors (e.g., -EPIPE, -ETIMEDOUT)
		hid_dbg(piu->hdev, "Polling failed with error %d, continuing timer.\n", size);
	}

	// Reschedule the timer for the next poll
	hrtimer_forward_now(timer, ms_to_ktime(poll_interval_ms));
	return HRTIMER_RESTART;
}


/**
 * piuio_set_report_legacy_ctrl_work - Work function for legacy output (runs in process context).
 * @work: Pointer to the work_struct.
 *
 * Sends a SET_REPORT control transfer for the legacy 0x1010 device.
 */
static void piuio_set_report_legacy_ctrl_work(struct work_struct *work)
{
	struct piuio_legacy_output_work *output_work =
		container_of(work, struct piuio_legacy_output_work, work);
	struct piuio *piu = output_work->piu;
	u8 rptid = output_work->report_id;
	int bank = rptid - PIUIO_RPT_OUT_BASE;
	u8 *buf;
	int i, ret;
	unsigned long flags;

	// Check piu and associated device pointers
	if (!piu ||!piu->hdev ||!piu->udev) {
		pr_err("piuio: Invalid state in legacy output work (piu=%p)\n", piu);
		kfree(output_work); // Free work struct if state invalid
		return;
	}

	hid_dbg(piu->hdev, "Sending SET_REPORT(ctrl) for ID 0x%02x (bank %d) via workqueue\n", rptid, bank);

	// Allocate buffer with GFP_KERNEL (safe to sleep here)
	buf = kmalloc(PIUIO_OUTPUT_CHUNK + 1, GFP_KERNEL);
	if (!buf) {
		hid_err(piu->hdev, "Failed to allocate buffer for SET_REPORT(ctrl)\n");
		kfree(output_work);
		return;
	}

	// Prepare report data from led_shadow
	buf[0] = rptid; // Report ID
	spin_lock_irqsave(&piu->led_lock, flags);
	for (i = 0; i < PIUIO_OUTPUT_CHUNK; i++) {
		int idx = bank * PIUIO_OUTPUT_CHUNK + i;
		// Bounds check against PIUIO_MAX_LEDS
		buf[i + 1] = (idx < PIUIO_MAX_LEDS && piu->led_shadow[idx])? 1 : 0;
	}
	spin_unlock_irqrestore(&piu->led_lock, flags);

	// Send control message (can sleep)
	ret = usb_control_msg(
		piu->udev,
		usb_sndctrlpipe(piu->udev, 0),
		HID_REQ_SET_REPORT,
		USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		(HID_OUTPUT_REPORT << 8) | rptid,
		piu->iface,
		buf, PIUIO_OUTPUT_CHUNK + 1,
		msecs_to_jiffies(1000)); // Allow blocking

	kfree(buf);
	kfree(output_work); // Free the work structure itself

	if (ret < 0) {
		hid_warn(piu->hdev, "SET_REPORT(ctrl) for ID 0x%02x failed: %d\n", rptid, ret);
	} else if (ret!= (PIUIO_OUTPUT_CHUNK + 1)) {
		hid_warn(piu->hdev, "SET_REPORT(ctrl) for ID 0x%02x transferred %d bytes (expected %d)\n",
				 rptid, ret, PIUIO_OUTPUT_CHUNK + 1);
	} else {
		hid_dbg(piu->hdev, "SET_REPORT(ctrl) for ID 0x%02x successful\n", rptid);
	}
}

/**
 * piuio_queue_legacy_output_work - Queue work to send legacy output report.
 * @piu: Device instance data.
 * @rptid: Report ID to send.
 *
 * Allocates and queues a work item to send a SET_REPORT control transfer.
 * Uses a mutex to prevent flooding the workqueue.
 *
 * Return: 0 on success or if busy, negative error code on failure.
 */
static int piuio_queue_legacy_output_work(struct piuio *piu, u8 rptid)
{
	struct piuio_legacy_output_work *output_work;

	// Check piu and required workqueue pointer
	if (!piu ||!piu->legacy_output_wq) {
		pr_err("piuio: Invalid state for queuing legacy work (piu=%p, wq=%p)\n",
			   piu, piu? piu->legacy_output_wq : NULL);
		return -ENODEV;
	}

	// Allocate work structure (use GFP_ATOMIC if called from atomic context)
	output_work = kmalloc(sizeof(*output_work), GFP_ATOMIC);
	if (!output_work) {
		hid_err(piu->hdev, "Failed to allocate legacy output work\n");
		return -ENOMEM;
	}

	INIT_WORK(&output_work->work, piuio_set_report_legacy_ctrl_work);
	output_work->piu = piu;
	output_work->report_id = rptid;

	// Use mutex to prevent flooding the workqueue if called rapidly
	if (mutex_trylock(&piu->legacy_output_mutex)) {
		if (!queue_work(piu->legacy_output_wq, &output_work->work)) {
			hid_err(piu->hdev, "Failed to queue legacy output work for ID 0x%02x\n", rptid);
			kfree(output_work); // Free if queue_work failed
			mutex_unlock(&piu->legacy_output_mutex);
			return -EBUSY;
		}
		// Success, unlock mutex
		mutex_unlock(&piu->legacy_output_mutex);
	} else {
		hid_dbg(piu->hdev, "Legacy output work already pending, skipping for ID 0x%02x\n", rptid);
		kfree(output_work); // Free if not queued
		return -EBUSY; // Indicate busy, but not necessarily an error for caller
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
	if (status && status!= -ENOENT && status!= -ECONNRESET && status!= -ESHUTDOWN) {
		hid_warn(piu->hdev, "Interrupt OUT URB unexpected status %d\n", status);
	}
}


/**
 * piuio_send_output_interrupt - Send LED state via Interrupt OUT endpoint (for 1020).
 * @piu: Device instance data.
 *
 * Prepares and submits a 16-byte report reflecting the current led_shadow state
 * to the Interrupt OUT endpoint (0x02). Uses locking to prevent concurrent submissions.
 *
 * Return: 0 on success or if busy, negative error code on failure.
 */
static int piuio_send_output_interrupt(struct piuio *piu)
{
	int i, ret = 0;
	unsigned long flags_led, flags_submit;

	// Basic validation
	if (!piu ||!piu->output_urb ||!piu->output_buf ||!piu->udev ||!piu->hdev) {
		pr_err("piuio: Invalid state in piuio_send_output_interrupt (piu=%p, urb=%p, buf=%p)\n",
			   piu, piu? piu->output_urb : NULL, piu? piu->output_buf : NULL);
		return -EFAULT;
	}

	hid_dbg(piu->hdev, "Attempting to send Interrupt OUT report\n");

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

	// Prepare the buffer using the led_shadow state
	memset(piu->output_buf, 0, PIUIO_OUTPUT_SIZE_NEW);
	spin_lock_irqsave(&piu->led_lock, flags_led);
	// Map first 48 LEDs into the 16 bytes (128 bits)
	for (i = 0; i < PIUIO_MAX_LEDS && i < (PIUIO_OUTPUT_SIZE_NEW * 8); i++) {
		if (piu->led_shadow[i]) {
			__set_bit(i, (unsigned long *)piu->output_buf);
		}
	}
	spin_unlock_irqrestore(&piu->led_lock, flags_led);

	// Fill and submit the URB
	usb_fill_int_urb(piu->output_urb, piu->udev, piu->output_pipe,
					 piu->output_buf, PIUIO_OUTPUT_SIZE_NEW,
					 piuio_output_report_interrupt_complete, piu,
					 1); // Interval ignored for OUT

	hid_dbg(piu->hdev, "Submitting Interrupt OUT URB (size %d)\n", PIUIO_OUTPUT_SIZE_NEW);
	ret = usb_submit_urb(piu->output_urb, GFP_ATOMIC); // Use GFP_ATOMIC as could be called from led_set
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
	unsigned long flags;

	// Check piu and associated hid device
	if (!piu ||!piu->hdev) {
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
		hid_dbg(piu->hdev, "Queueing legacy control transfer work for LED set\n");
		ret = piuio_queue_legacy_output_work(piu, PIUIO_RPT_OUT_BASE + (ld->idx / PIUIO_OUTPUT_CHUNK));
		// Return 0 even if busy/queued, as the request was accepted
		if (ret == -EBUSY)
			ret = 0;
	} else {
		hid_warn(piu->hdev, "LED set called for unknown product ID %04x\n", piu->hdev->product);
		ret = -ENODEV;
	}

	hid_dbg(piu->hdev, "LED set for index %d returned %d\n", ld->idx, ret);
	// Return 0 for success or if busy/queued, error otherwise
	return (ret < 0)? ret : 0;
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
	// Get misc device struct from file pointer
	struct miscdevice *misc = filp->private_data;
	// FIX: Use misc_get_drvdata to retrieve the associated piu struct
	struct piuio *piu = misc_get_drvdata(misc);
	const size_t legacy_bits = PIUIO_LEGACY_SIZE * 8;
	u8 *tmp = NULL;
	int pin, ret = 0;
	int first_error = 0;
	unsigned long flags;

	// Check piu and associated hid device
	if (!piu ||!piu->hdev) {
		pr_err("piuio: Invalid state in dev_write (piu=%p)\n", piu);
		// Check if misc is valid before trying to access name
		pr_err("piuio: Write attempt on potentially invalid device %s\n", misc? misc->name : "<unknown>");
		return -ENODEV;
	}

	hid_dbg(piu->hdev, "Write to legacy device /dev/%s, len=%zu\n", piu->misc.name, len);

	// Check for expected legacy write size
	if (len!= PIUIO_LEGACY_SIZE) {
		hid_err(piu->hdev, "Legacy write invalid length %zu (expected %d)\n", len, PIUIO_LEGACY_SIZE);
		return -EINVAL;
	}

	// Allocate temporary buffer (use GFP_KERNEL, we're in process context)
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
	// Update only the bits corresponding to the legacy interface size
	for (pin = 0; pin < legacy_bits && pin < PIUIO_MAX_LEDS; pin++) {
		piu->led_shadow[pin] =!!(tmp[pin / 8] & (1 << (pin % 8)));
	}
	// Leave LEDs beyond legacy_bits untouched by this write
	spin_unlock_irqrestore(&piu->led_lock, flags);

	kfree(tmp); // Free temporary buffer

	// Trigger appropriate output mechanism
	if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_NEW) { // 1020
		hid_dbg(piu->hdev, "Updating LEDs via Interrupt OUT due to legacy write\n");
		ret = piuio_send_output_interrupt(piu);
		if (ret < 0)
			first_error = ret;
	} else if (piu->hdev->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) { // 1010
		hid_dbg(piu->hdev, "Updating LEDs via legacy control transfer workqueue due to legacy write\n");
		// Queue work for each chunk affected by the legacy write
		for (pin = 0; pin < legacy_bits && pin < PIUIO_MAX_LEDS; pin += PIUIO_OUTPUT_CHUNK) {
			ret = piuio_queue_legacy_output_work(piu, PIUIO_RPT_OUT_BASE + (pin / PIUIO_OUTPUT_CHUNK));
			// Report the first error encountered (ignore -EBUSY as it means queued/skipped)
			if (ret < 0 && first_error == 0 && ret!= -EBUSY) {
				first_error = ret;
			}
		}
	} else {
		hid_err(piu->hdev, "Cannot update LEDs for unknown product ID %04x via legacy write\n", piu->hdev->product);
		first_error = -ENODEV;
	}

	hid_dbg(piu->hdev, "Legacy write processing finished, result=%d\n", first_error? first_error : (int)len);
	// Return length on success, or the first error encountered
	return first_error? first_error : len;
}

// File operations for the legacy /dev/piuioX device
static const struct file_operations piuio_fops = {
	.owner   = THIS_MODULE,
	.write   = piuio_dev_write,
	.open    = simple_open, // Sets filp->private_data = miscdev
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
		hid_err(hdev, "Cannot get USB interface\n");
		return -ENODEV;
	}

	// Allocate the main struct using device-managed memory
	piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
	if (!piu) {
		hid_err(hdev, "Failed to allocate piu struct\n");
		return -ENOMEM;
	}
	hid_set_drvdata(hdev, piu); // Associate piu struct with hid_device
	hid_info(hdev, "Allocated piu struct: %p (size %zu)", piu, sizeof(*piu));

	// Initialize basic fields
	piu->hdev = hdev;
	piu->udev = interface_to_usbdev(intf);
	if (!piu->udev) { // Check result of interface_to_usbdev
		hid_err(hdev, "Cannot get USB device\n");
		return -ENODEV; // devm_kzalloc handles piu cleanup
	}
	piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;
	spin_lock_init(&piu->led_lock);
	spin_lock_init(&piu->output_submit_lock);
	atomic_set(&piu->output_active, 0);
	mutex_init(&piu->legacy_output_mutex);

	// --- Find Interrupt OUT Endpoint (for 1020 output) ---
	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *ep = &intf->cur_altsetting->endpoint[i].desc;
		if (!ep_out && usb_endpoint_is_int_out(ep)) {
			ep_out = ep;
			hid_info(hdev, "Found Interrupt OUT endpoint: addr=0x%02x, size=%d\n",
					 ep->bEndpointAddress, usb_endpoint_maxp(ep));
			break; // Assume only one interrupt OUT endpoint
		}
	}

	// --- Input device setup ---
	snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys);
	idev = devm_input_allocate_device(&hdev->dev);
	if (!idev) {
		hid_err(hdev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_free_piu;
	}
	piu->idev = idev;
	hid_info(hdev, "Allocated input device: %p", idev);

	// Configure input device properties
	idev->name = hdev->name;
	idev->phys = piu->phys;
	idev->id.bustype = hdev->bus;
	idev->id.vendor  = hdev->vendor;
	idev->id.product = hdev->product;
	idev->id.version = hdev->version;
	idev->dev.parent = &hdev->dev;

	// Set capabilities (keys, buttons, scan codes)
	set_bit(EV_KEY, idev->evbit);
	for (i = 0; i < PIUIO_NUM_INPUTS; i++)
		set_bit(piuio_keycode(i), idev->keybit);
	set_bit(EV_MSC, idev->evbit);
	set_bit(MSC_SCAN, idev->mscbit);

	// Register the input device
	ret = input_register_device(idev);
	if (ret) {
		hid_err(hdev, "Failed to register input device: %d\n", ret);
		goto err_free_piu;
	}
	hid_info(hdev, "Registered input device %s", idev->name);

	// --- LED class device setup ---
	piu->led = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS, sizeof(*piu->led), GFP_KERNEL);
	if (!piu->led) {
		hid_err(hdev, "Failed to allocate LED structures\n");
		ret = -ENOMEM;
		goto err_unregister_input;
	}
	hid_info(hdev, "Allocated %d LED structures", PIUIO_MAX_LEDS);

	// Register each LED
	for (i = 0; i < PIUIO_MAX_LEDS; i++) {
		piu->led[i].piu = piu;
		piu->led[i].idx = i;
		piu->led[i].cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "%s::output%u", dev_name(&hdev->dev), i);
		if (!piu->led[i].cdev.name) {
			hid_err(hdev, "Failed to allocate name for LED %d\n", i);
			ret = -ENOMEM;
			goto err_unregister_input;
		}
		piu->led[i].cdev.brightness_set_blocking = piuio_led_set;
		piu->led[i].cdev.max_brightness = 1;
		ret = devm_led_classdev_register(&hdev->dev, &piu->led[i].cdev);
		if (ret) {
			hid_err(hdev, "Failed to register LED %d (%s): %d\n", i, piu->led[i].cdev.name, ret);
			goto err_unregister_input;
		}
	}
	hid_info(hdev, "Registered %d LED class devices", PIUIO_MAX_LEDS);

	// --- Setup for Interrupt OUT URB (Required for 1020 output) ---
	if (id->product == USB_PRODUCT_ID_BTNBOARD_NEW) {
		if (!ep_out || ep_out->bEndpointAddress!= PIUIO_INT_OUT_EP) { // Check correct EP addr
			hid_warn(hdev, "Interrupt OUT endpoint 0x%02x not found for 1020 device. Output may not work.\n", PIUIO_INT_OUT_EP);
			// Continue probe, but output might fail later
		} else {
			// Allocate buffer (devm) and URB (manual)
			piu->output_buf = devm_kzalloc(&hdev->dev, PIUIO_OUTPUT_SIZE_NEW, GFP_KERNEL);
			// Use usb_alloc_urb (manual free needed)
			piu->output_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!piu->output_buf ||!piu->output_urb) {
				hid_err(hdev, "Failed to allocate output URB/buffer\n");
				usb_free_urb(piu->output_urb); // Free urb if buf failed or urb failed
				ret = -ENOMEM;
				goto err_unregister_input;
			}
			piu->output_pipe = usb_sndintpipe(piu->udev, ep_out->bEndpointAddress);
			hid_info(hdev, "Allocated URB %p and buffer %p for Interrupt OUT EP 0x%02x\n",
					 piu->output_urb, piu->output_buf, ep_out->bEndpointAddress);
		}
	}

	// --- Setup Workqueue for Legacy Output (1010) ---
	if (id->product == USB_PRODUCT_ID_BTNBOARD_LEGACY) {
		// Allocate workqueue (needs manual destruction)
		piu->legacy_output_wq = alloc_ordered_workqueue("piuio_legacy_output", WQ_MEM_RECLAIM);
		if (!piu->legacy_output_wq) {
			hid_err(hdev, "Failed to allocate legacy output workqueue\n");
			ret = -ENOMEM;
			goto err_free_output_urb; // Free output URB if allocated
		}
		hid_info(hdev, "Allocated workqueue for legacy output\n");
	}

	// --- Start HID Hardware Layer (Disable generic HID input handling) ---
	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_HIDINPUT);
	if (ret) {
		hid_err(hdev, "hid_hw_start failed: %d\n", ret);
		goto err_destroy_workqueue;
	}
	hid_info(hdev, "hid_hw_start successful");

	// --- Parse HID Reports (Optional but good practice) ---
	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "hid_parse failed: %d\n", ret);
		goto err_stop_hid;
	}
	hid_info(hdev, "hid_parse successful");

	// --- Open HID Device Communication Channel ---
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hid_hw_open failed: %d\n", ret);
		goto err_stop_hid;
	}
	hid_info(hdev, "hid_hw_open successful");

	// --- Send SET_IDLE Request (Required by 1020) ---
	ret = usb_control_msg(piu->udev,
						  usb_sndctrlpipe(piu->udev, 0),
						  HID_REQ_SET_IDLE,
						  USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
						  0, // Duration = 0 (infinite)
						  piu->iface, // Interface
						  NULL, 0, // No data
						  msecs_to_jiffies(100));
	if (ret < 0 && ret!= -EPIPE) {
		// Log as warning, but continue probe as it might not be strictly required
		hid_warn(hdev, "SET_IDLE failed: %d (continuing)\n", ret);
	} else {
		hid_info(hdev, "SET_IDLE sent successfully (or stalled)\n");
	}

	// --- Initialize and Start Polling Timer ---
	hrtimer_init(&piu->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	piu->timer.function = piuio_timer_cb;
	hrtimer_start(&piu->timer, ms_to_ktime(poll_interval_ms), HRTIMER_MODE_REL);
	hid_info(hdev, "Started input polling timer (interval %d ms)\n", poll_interval_ms);


	// --- Misc device setup (/dev/piuioX) - REGISTER LAST ---
	piu->misc.minor = MISC_DYNAMIC_MINOR;
	piu->misc_name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "piuio-%s", dev_name(&hdev->dev));
	if (!piu->misc_name) {
		hid_err(hdev, "Failed to allocate misc device name string\n");
		ret = -ENOMEM;
		goto err_cancel_timer; // Cancel timer before cleanup
	}
	piu->misc.name = piu->misc_name;
	piu->misc.fops = &piuio_fops;
	piu->misc.parent = &hdev->dev;

 // Register misc device (needs manual deregistration on failure/remove)
 ret = misc_register(&piu->misc);
 if (ret) {
        hid_err(hdev, "Failed to register misc device '%s': %d\n", piu->misc.name, ret);
        goto err_cancel_timer; // devm_* handles misc_name
 }
 // Associate piu struct with the misc device using the underlying device data function
 dev_set_drvdata(&piu->misc.this_device, piu);
 hid_info(hdev, "Registered misc device /dev/%s\n", piu->misc.name);
	hid_info(hdev, "PIUIO HID Driver probe completed successfully for /dev/%s\n", piu->misc.name);
	return 0; // Success!

/* --- Error Handling Cleanup: Unwind in reverse order --- */
err_cancel_timer:
	hrtimer_cancel(&piu->timer);
err_stop_hid:
	hid_hw_close(hdev); // Close HID if open
	hid_hw_stop(hdev);
err_destroy_workqueue:
	if (piu->legacy_output_wq)
		destroy_workqueue(piu->legacy_output_wq); // Manual cleanup
err_free_output_urb:
	// Free output URB manually if it was allocated
	usb_free_urb(piu->output_urb);
	// devm_* handles output_buf
err_unregister_input:
	input_unregister_device(piu->idev); // Manual cleanup
	// devm_* handles LEDs, led names
err_free_piu:
	// devm_kzalloc handles piu struct freeing
	hid_info(hdev, "PIUIO HID Driver probe failed (%d)\n", ret);
	return ret;
}


/**
 * piuio_remove - Called when the HID device is removed or driver unloaded.
 * @hdev: Pointer to the HID device structure.
 */
static void piuio_remove(struct hid_device *hdev)
{
	struct piuio *piu = hid_get_drvdata(hdev);

	if (!piu) {
		hid_warn(hdev, "piuio_remove called with NULL drvdata\n");
		return;
	}

	hid_info(hdev, "Unregistering PIUIO HID Driver for %s\n", piu->misc.name? piu->misc.name : dev_name(&hdev->dev));

	// 1. Deregister misc device first
	hid_info(hdev, "Deregistering misc device /dev/%s...", piu->misc.name);
	misc_deregister(&piu->misc);
	hid_info(hdev, "Misc device deregistered.");

	// 2. Cancel timer
	hid_info(hdev, "Cancelling input timer...");
	hrtimer_cancel(&piu->timer);
	hid_info(hdev, "Input timer cancelled.");

	// 3. Kill pending output URB and free it
	if (piu->output_urb) {
		hid_info(hdev, "Killing and freeing output URB %p...", piu->output_urb);
		usb_kill_urb(piu->output_urb);
		// Free the manually allocated output URB
		usb_free_urb(piu->output_urb);
		hid_info(hdev, "Output URB killed and freed.");
	}

	// 4. Destroy workqueue (if created)
	if (piu->legacy_output_wq) {
		hid_info(hdev, "Destroying legacy output workqueue...");
		destroy_workqueue(piu->legacy_output_wq);
		hid_info(hdev, "Workqueue destroyed.");
	}

	// 5. Close and stop HID hardware layer
	hid_info(hdev, "Closing HID hardware...");
	hid_hw_close(hdev);
	hid_info(hdev, "HID hardware closed.");

	hid_info(hdev, "Stopping HID hardware...");
	hid_hw_stop(hdev);
	hid_info(hdev, "HID hardware stopped.");

	// 6. Remaining resources (input device, LEDs, output_buf, misc_name, piu struct)
	//    are cleaned up automatically by the devm_* framework.

	hid_info(hdev, "PIUIO HID Driver remove finished.\n");
}

// --- ID Table Includes Both Devices ---
static const struct hid_device_id piuio_ids[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_LEGACY) }, // 0x0d2f:0x1010
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_NEW) },    // 0x0d2f:0x1020
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
MODULE_AUTHOR("Diego Acevedo, based on work by Devin J. Pohly");
MODULE_DESCRIPTION("PIUIO HID interface driver (using polling input)");
MODULE_LICENSE("GPL v2");