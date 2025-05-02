// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad (16-byte report) + coin counter
 * 2025-04-xx  Diego Acevedo
 * · coin: watch only byte-15 of Feature-report 0x01
 * · any change => one KEY_COIN press/release pulse
 * · WAKE frame now sent through hid-core helper (reliable)
 *
 * --- Debugging Note ---
 * This version includes pr_info/pr_err messages to help trace the probe/remove
 * sequence and specifically the execution of the send_wake function.
 * Monitor kernel logs (dmesg -w or journalctl -f) when loading/unloading
 * the module and plugging/unplugging the device.
 * Look for error messages and the sequence of info messages to diagnose
 * why the wake command might not be having the expected effect (e.g., light feedback).
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <linux/kernel.h>           /* clamp_val() */
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ratelimit.h>
#include <linux/ktime.h>
#include <linux/string.h>
#include <linux/delay.h>            /* usleep_range() */

/* --- KEY_COIN fallback for old kernels --- */
#ifndef KEY_COIN
#define KEY_COIN  KEY_KPENTER
#endif

/* --- device IDs --- */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

/* --- input report (buttons) --- */
#define RPT_ID_BTN  0x30
#define RPT_LEN_BTN 16

/* --- coin feature report & wake frame --- */
#define COIN_RID        0x01
#define COIN_OFFSET_HI  15
#define COIN_BUF_LEN    258       /* full size packet – required */

#define WAKE_RID        0x81
#define WAKE_LEN        19

/* --- button map (active-low) --- */
struct mapent { u8 mask; u16 p1, p2; };
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH , BTN_TR      },
	{ 0x08, BTN_WEST  , BTN_SELECT  },
	{ 0x04, BTN_NORTH , BTN_START   },
	{ 0x02, BTN_EAST  , BTN_THUMBL  },
	{ 0x10, BTN_TL    , BTN_THUMBR  },
};
#define NBTN 10

/* --- context --- */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int               iface;
	bool              hid_started; // Flag to track if hid_hw_start succeeded

	u8   btn_buf[RPT_LEN_BTN];
	bool down[NBTN];

	u8   coin_last;
	u8   coin_buf[COIN_BUF_LEN];

	struct input_dev  *idev;

	struct work_struct poll_wq;
	struct hrtimer     timer;
	atomic_t           stop;
};
// Use a single global context pointer, assumes only one device instance
static struct piuio *ctx;

/* --- module param – button polling interval --- */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval for buttons in milliseconds (1-1000, default 4)");
static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* --- helpers --- */

/**
 * send_wake - Sends the 19-byte Output Report 0x81 required to enable coin counting.
 * @p: Device context.
 */
static void send_wake(struct piuio *p)
{
	// Allocate buffer for the wake report
	// Needs to be WAKE_LEN (19) bytes: Report ID (1) + Payload (18)
	u8 buf[WAKE_LEN];

	// Check if p and p->hdev are valid before proceeding
	if (!p || !p->hdev) {
		pr_err("send_wake: Invalid context or hdev\n");
		return;
	}

	pr_info("send_wake: Preparing buffer (RID 0x%02X, %d bytes total)...\n",
		WAKE_RID, WAKE_LEN);

	// Set the Report ID
	buf[0] = WAKE_RID;
	// Fill the rest of the buffer (18 bytes) with 0xFF
	memset(buf + 1, 0xFF, WAKE_LEN - 1);

	pr_info("send_wake: Calling hid_hw_output_report...\n");
	// Use the hid-core helper to send the Output report.
	// This function handles underlying USB details (control transfer) and power management.
	// Note: This function typically doesn't return an error code directly. Failures
	// might appear as lower-level USB errors in dmesg or prevent subsequent reads.
	hid_hw_output_report(p->hdev, buf, WAKE_LEN);

	pr_info("send_wake: hid_hw_output_report call finished. Applying delay...\n");

	// Small delay: The original code suggests this helps the board process the frame.
	// Python code doesn't have this, but kernel timing might differ.
	usleep_range(2000, 3000); // 2-3 milliseconds

	pr_info("send_wake: Delay finished.\n");
}

/**
 * get_matrix - Reads the button state using a GetReport request.
 * @p: Device context.
 * Returns: 0 on success, negative error code on failure, -EIO if wrong length.
 */
static int get_matrix(struct piuio *p)
{
	// Construct wValue for GetReport: Report Type (Input=1) << 8 | Report ID
	u16 w = (HID_INPUT_REPORT << 8) | RPT_ID_BTN;
	int r;

	// Use standard USB control message for GetReport (Input)
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0), // Pipe
				HID_REQ_GET_REPORT, // bRequest
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // bmRequestType
				w, p->iface, // wValue, wIndex
				p->btn_buf, RPT_LEN_BTN, // Buffer, Length
				30); // Timeout (ms)

	// Check the return value: should be exactly the report length
	if (r == RPT_LEN_BTN) {
		return 0; // Success
	} else {
		if (r >= 0) { // Received data, but wrong length
			pr_warn_ratelimited("get_matrix: Incorrect report length received (%d, expected %d)\n",
					  r, RPT_LEN_BTN);
			return -EIO;
		} else { // Negative error code from usb_control_msg
			pr_warn_ratelimited("get_matrix: usb_control_msg failed (%d)\n", r);
			return r;
		}
	}
}

/**
 * read_coin_hi - Reads the high byte of the coin counter via GetReport (Feature).
 * @p: Device context.
 * @hi: Pointer to store the coin counter high byte.
 * Returns: 0 on success, negative error code or -EIO on failure.
 */
static int read_coin_hi(struct piuio *p, u8 *hi)
{
	int r;
	// Construct wValue for GetReport: Report Type (Feature=3) << 8 | Report ID
	u16 wValue = (HID_FEATURE_REPORT << 8) | COIN_RID;

	// Use standard USB control message for GetReport (Feature)
	// Request the full buffer size, even though we only need byte 15.
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0), // Pipe
				HID_REQ_GET_REPORT, // bRequest
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // bmRequestType
				wValue, p->iface, // wValue, wIndex
				p->coin_buf, COIN_BUF_LEN, // Buffer, Length
				30); // Timeout (ms)

	// Check if the returned length is sufficient to read the target byte
	if (r < COIN_OFFSET_HI + 1) {
		if (r >= 0) {
			pr_warn_ratelimited("read_coin_hi: Report too short (%d bytes)\n", r);
			return -EIO; // Report received but too short
		} else {
			pr_warn_ratelimited("read_coin_hi: usb_control_msg failed (%d)\n", r);
			return r; // Negative error code
		}
	}

	// Extract the high byte of the coin counter (little-endian, byte 15)
	*hi = p->coin_buf[COIN_OFFSET_HI];
	return 0; // Success
}

/* --- matrix -> events --- */
/**
 * push_btn_events - Translates the button matrix buffer into input events.
 * @p: Device context.
 */
static void push_btn_events(struct piuio *p)
{
	bool now[NBTN] = { false }; // Track current button states
	int i, b;

	// Decode P1 buttons (bytes 0-3)
	for (i = 0; i < 4; i++)
		for (b = 0; b < 5; b++)
			if (!(p->btn_buf[i] & map[b].mask)) // Active low
				now[b] = true;

	// Decode P2 buttons (bytes 4-7)
	for (i = 4; i < 8; i++)
		for (b = 0; b < 5; b++)
			if (!(p->btn_buf[i] & map[b].mask)) // Active low
				now[5 + b] = true;

	// Compare current state ('now') with previous state ('p->down')
	// and report changes as key events.
	for (b = 0; b < NBTN; b++)
		if (now[b] != p->down[b]) {
			input_report_key(p->idev,
				(b < 5) ? map[b].p1 : map[b - 5].p2, // Get key code from map
				now[b]); // Report press (1) or release (0)
			p->down[b] = now[b]; // Update stored state
		}
}

/* --- polling work --- */
// Rate limit debug messages from polling work
static DEFINE_RATELIMIT_STATE(rl, HZ, 10); // Max 10 messages per second

/**
 * poll_work - Workqueue function to poll buttons and coin counter.
 * @w: Work struct.
 */
static void poll_work(struct work_struct *w)
{
	// Ensure context is valid and module isn't stopping
	if (!ctx || atomic_read(&ctx->stop))
		return;

	// Poll button matrix
	if (!get_matrix(ctx)) { // If get_matrix succeeded
		// Optional debug: print button buffer if rate limit allows
		if (__ratelimit(&rl))
			pr_debug("Button matrix: %*phN\n", RPT_LEN_BTN, ctx->btn_buf);
		// Process button buffer and send events
		push_btn_events(ctx);
	} else {
		// get_matrix failed, error already printed by get_matrix (ratelimited)
	}

	// Poll coin counter less frequently (e.g., every 100ms)
	static ktime_t last_coin_check;
	ktime_t now = ktime_get();

	if (ktime_ms_delta(now, last_coin_check) >= 100) {
		u8 current_coin_hi;
		// Read the current coin counter high byte
		if (!read_coin_hi(ctx, &current_coin_hi)) { // If read succeeded
			// Check if the coin byte has changed since last time
			if (current_coin_hi != ctx->coin_last) {
				pr_info("Coin event detected! Byte 15 changed from 0x%02X to 0x%02X\n",
					ctx->coin_last, current_coin_hi);
				// Send a KEY_COIN press/release pulse
				input_report_key(ctx->idev, KEY_COIN, 1); // Press
				input_sync(ctx->idev); // Sync immediately after press
				input_report_key(ctx->idev, KEY_COIN, 0); // Release
				ctx->coin_last = current_coin_hi; // Update last known value
			}
		} else {
			// read_coin_hi failed, error already printed (ratelimited)
		}
		last_coin_check = now; // Update time of last check
	}

	// Sync input events generated by button and coin checks
	input_sync(ctx->idev);
}

/**
 * timer_cb - HRTimer callback function to schedule the polling work.
 * @t: Timer struct.
 * Returns: HRTIMER_RESTART to reschedule, HRTIMER_NORESTART to stop.
 */
static enum hrtimer_restart timer_cb(struct hrtimer *t)
{
	// If context is valid and module is not stopping, queue the work
	if (ctx && !atomic_read(&ctx->stop))
		queue_work(system_unbound_wq, &ctx->poll_wq);

	// Reschedule the timer for the next interval if not stopping
	if (!atomic_read(&ctx->stop)) {
		hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
		return HRTIMER_RESTART;
	} else {
		return HRTIMER_NORESTART;
	}
}


/* --- probe / remove --- */

/**
 * probe - Called when a matching USB device is connected.
 * @hdev: HID device structure.
 * @id: Matching HID device ID entry.
 * Returns: 0 on success, negative error code on failure.
 */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r = 0; // Use one variable for return codes
	int b;     // Loop counter for buttons

	pr_info("Probe function entered for VID=0x%04X PID=0x%04X\n",
		id->vendor, id->product);

	// Prevent multiple instances if ctx is already assigned
	if (ctx) {
		pr_err("Device context already exists. Refusing to probe again.\n");
		return -EBUSY;
	}

	// Allocate device context structure using device-managed memory
	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) {
		pr_err("Failed to allocate device context memory.\n");
		return -ENOMEM;
	}
	pr_info("Device context allocated.\n");

	// Initialize context fields
	p->hdev = hdev;
	p->udev = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	p->hid_started = false; // Initialize flag
	atomic_set(&p->stop, 0);  // Initialize stop flag

	// Associate context with the hid_device
	hid_set_drvdata(hdev, p);
	pr_info("Context basic fields initialized and associated with hid_device.\n");

	// --- Essential HID Initialization ---
	pr_info("Parsing HID report descriptor...\n");
	r = hid_parse(hdev);
	if (r) {
		pr_err("hid_parse failed! Error %d\n", r);
		// No need to clean up 'p', devm handles it on return
		return r;
	}
	pr_info("hid_parse OK.\n");

	// Start HID hardware communication (required for I/O)
	pr_info("Starting HID hardware communication (hid_hw_start)...\n");
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (r) {
		pr_err("hid_hw_start failed! Error %d\n", r);
		// hid_parse succeeded, but start failed. No need to stop.
		return r;
	}
	p->hid_started = true; // Mark HID HW as started
	pr_info("hid_hw_start OK.\n");

	// --- Device-Specific Initialization Sequence ---
	// Mimic BIOS sequence: SET_IDLE followed by first WAKE attempt

	pr_info("Sending SET_IDLE request...\n");
	// Send SET_IDLE. Ignore potential STALL error as some firmware might
	// return it, but the command might still be processed. Use short timeout.
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0),
			HID_REQ_SET_IDLE, // Request: SET_IDLE
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, // Type
			0, p->iface, // Value (0), Index (Interface)
			NULL, 0, // Data (None), Length (0)
			10); // Timeout 10ms
	// We don't check the return value strictly here based on original logic.

	pr_info("Calling send_wake (1st attempt - may be ignored by some boards)...\n");
	send_wake(p); // First attempt to wake the device

	// --- Input Device Setup ---
	pr_info("Allocating input device...\n");
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) {
		pr_err("Failed to allocate input device memory.\n");
		r = -ENOMEM;
		goto err_hw; // Go to cleanup
	}
	pr_info("Input device allocated.\n");

	// Configure the input device properties
	p->idev->name = "PIUIO Gamepad + Coin";
	p->idev->phys = "piuio/input0"; // Physical path
	// *** FIX: Revert to original manual ID setting ***
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VID_PIUIO;
	p->idev->id.product = PID_PIUIO1020;
	// *** End of FIX ***
	p->idev->dev.parent = &hdev->dev; // Set parent device

	// Announce supported event types and keys/buttons
	set_bit(EV_KEY, p->idev->evbit); // Announce KEY events
	// Map buttons based on the 'map' structure
	for (b = 0; b < NBTN; b++) {
		set_bit((b < 5) ? map[b].p1 : map[b - 5].p2, // Get key code
			p->idev->keybit); // Mark key as supported
	}
	set_bit(KEY_COIN, p->idev->keybit); // Announce COIN key support
	pr_info("Input device properties and key bits configured.\n");

	// Register the input device with the input subsystem
	pr_info("Registering input device...\n");
	r = input_register_device(p->idev);
	if (r) {
		pr_err("Failed to register input device! Error %d\n", r);
		// devm handles idev freeing if allocated.
		goto err_hw; // Go to cleanup
	}
	pr_info("Input device registered successfully.\n");

	// --- Final Wake and Initial State Reading ---
	// Read initial coin state and send the 'guaranteed' wake command

	pr_info("Reading initial coin state (high byte)...\n");
	if (read_coin_hi(p, &p->coin_last)) {
		// If read fails, log warning and default the value
		pr_warn("Failed to read initial coin state, defaulting hi-byte to 0.\n");
		p->coin_last = 0;
	} else {
		// Log the successfully read initial value
		pr_info("Initial coin hi-byte: 0x%02X\n", p->coin_last);
	}

	pr_info("Calling send_wake (2nd attempt - should activate coin counting)...\n");
	send_wake(p); // This wake call is expected to be effective

	// --- Start Polling ---
	pr_info("Initializing and starting polling timer/workqueue...\n");
	// Initialize the work structure
	INIT_WORK(&p->poll_wq, poll_work);
	// Initialize the high-resolution timer
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = timer_cb; // Set timer callback function
	// Start the timer to fire after the first interval
	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	// Assign to global context pointer *only after* successful setup
	ctx = p;

	pr_info("Device attached successfully. Polling interval: %d ms. Feature report size expected: %u B.\n",
		clamp_poll(), COIN_BUF_LEN);
	return 0; // Success

err_hw:
	// Error handling path: only reached if hid_hw_start succeeded but later steps failed.
	pr_err("Probe failed during setup. Cleaning up HID hardware...\n");
	// Stop HID hardware communication if it was successfully started
	if (p->hid_started) {
		hid_hw_stop(hdev);
		p->hid_started = false; // Update flag
	}
	// devm handles freeing of 'p' and 'p->idev' if allocated.
	// Clear global context if it was somehow set before failure.
	if (ctx == p) {
		ctx = NULL;
	}
	return r; // Return the encountered error code
}

/**
 * remove - Called when the USB device is disconnected or module is unloaded.
 * @hdev: HID device structure.
 */
static void remove(struct hid_device *hdev)
{
	// Retrieve device context associated with the hid_device
	struct piuio *p = hid_get_drvdata(hdev);

	pr_info("Remove function entered.\n");

	// Check if context pointer is valid
	if (!p) {
		pr_warn("Remove called but no private data found for hdev.\n");
		return;
	}

	// Check if this instance is the one assigned to the global context 'ctx'
	// This prevents issues if probe failed before setting 'ctx' or if multiple
	// devices were somehow probed (though blocked by current logic).
	if (ctx == p) {
		pr_info("Stopping polling mechanisms...\n");
		// Signal polling loops and timer to stop
		atomic_set(&p->stop, 1);
		// Cancel the timer synchronously
		hrtimer_cancel(&p->timer);
		// Cancel any pending or running work item synchronously
		cancel_work_sync(&p->poll_wq);
		pr_info("Polling stopped.\n");
	} else {
		pr_warn("Remove called for an instance not matching global ctx. Polling mechanisms not stopped for this instance.\n");
	}

	// Unregister the input device if it was registered
	// Check if p->idev was successfully allocated/registered in probe
	if (p->idev) {
		pr_info("Unregistering input device...\n");
		input_unregister_device(p->idev);
		// Memory for p->idev is managed by devm, no need to free explicitly.
	} else {
		pr_info("No input device was registered for this instance.\n");
	}

	// Stop HID hardware communication if it was started
	// Check the flag set in probe after successful hid_hw_start
	if (p->hid_started) {
		pr_info("Stopping HID hardware communication (hid_hw_stop)...\n");
		hid_hw_stop(p->hdev);
		p->hid_started = false; // Update flag
	} else {
		pr_info("HID hardware was not started or already stopped.\n");
	}

	// Clear the global context pointer if it points to the removed device instance
	if (ctx == p) {
		pr_info("Clearing global device context pointer.\n");
		ctx = NULL;
	}

	pr_info("Remove function finished.\n");
	// Memory for 'p' context structure is managed by devm, no need to free.
}


/* --- boiler-plate --- */
// Define the USB device IDs this driver supports
static const struct hid_device_id ids[] = {
	{ HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) }, // Match PIUIO 1020
	{} // Terminating entry
};
// Export the device ID table to the module loading system
MODULE_DEVICE_TABLE(hid, ids);

// Define the HID driver structure
static struct hid_driver drv = {
	.name     = "piuio", // Driver name
	.id_table = ids,        // Supported device IDs
	.probe    = probe,      // Probe function
	.remove   = remove,     // Remove function
	// Add .raw_event handler here if processing raw HID reports directly
};
// Register the HID driver with the HID subsystem
module_hid_driver(drv);

// Module metadata
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020 - Gamepad + Coin (debug logging enabled, compile fix)");
