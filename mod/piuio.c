// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter driver
 * Based on work by Diego Acevedo.
 * Cleaned up version with 16-bit coin detection, mapped coin to button.
 */

#define pr_fmt(fmt) "piuio: " fmt // Matched driver name below

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h> // Includes BTN_ codes
#include <linux/kernel.h>           /* clamp_val(), le16_to_cpu, container_of */
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ratelimit.h>        // For pr_warn_ratelimited
#include <linux/ktime.h>
#include <linux/string.h>           // For memset
#include <linux/delay.h>            /* usleep_range() */
#include <linux/byteorder/generic.h> /* le16_to_cpu */
#include <linux/slab.h>             /* Needed if kmalloc were still used */
#include <linux/atomic.h>           /* For atomic_t */

/* Define which button code to use for coin input */
// Options: BTN_SELECT, BTN_START, BTN_MODE, BTN_A, BTN_B, BTN_C, etc.
// WARNING: Original map below uses BTN_SELECT for P2 West and BTN_START for P2 North.
// BTN_MODE is chosen as an example of an often unused button.
#define COIN_BTN_CODE BTN_MODE

/* Device IDs */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

/* Report IDs and constants */
#define RPT_ID_BTN      0x30
#define RPT_LEN_BTN     16
#define COIN_RID        0x01 // Feature Report ID for coin
#define COIN_OFFSET_LO  14   // Offset for Coin Counter Low Byte in Feature Report
#define COIN_OFFSET_HI  15   // Offset for Coin Counter High Byte in Feature Report
#define COIN_BUF_LEN    258  // Max buffer size for Feature Report read
#define WAKE_RID        0x81 // Output Report ID for wake/init
#define WAKE_LEN        19   // Length of wake/init report

/* Button bit masks and input event code mapping */
struct mapent { u8 mask; u16 p1, p2; };
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH , BTN_TR      }, // P1 DownLeft, P2 UpRight
	{ 0x08, BTN_WEST  , BTN_SELECT  }, // P1 UpLeft,   P2 Select
	{ 0x04, BTN_NORTH , BTN_START   }, // P1 Center,   P2 Start
	{ 0x02, BTN_EAST  , BTN_THUMBL  }, // P1 DownRight,P2 Thumb Left
	{ 0x10, BTN_TL    , BTN_THUMBR  }, // P1 UpRight,  P2 Thumb Right
};
#define NBTN 10 // Total number of physical buttons (5 per player)

/* Driver context structure */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int               iface;
	bool              hid_started;
	u8                btn_buf[RPT_LEN_BTN];     // Buffer for button input report
	bool              down[NBTN];               // Tracks current state of physical buttons
	u16               coin_last_16;             // Last read 16-bit coin counter value
	u8                coin_buf[COIN_BUF_LEN];   // Buffer for coin feature report
	struct input_dev  *idev;                    // Input device associated with this PIUIO
	struct work_struct poll_wq;                 // Workqueue item for polling
	struct hrtimer     timer;                   // High-resolution timer for polling schedule
	atomic_t           stop;                    // Flag to signal polling stop
};
static struct piuio *ctx; // Global pointer to the active device context (singleton)

/* Module parameter for polling interval */
static int poll_ms = 4; // Default polling interval in milliseconds
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval for buttons in milliseconds (1-1000, default 4)");
static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* --- Helper Functions --- */

/**
 * send_wake - Sends the Output Report 0x81 required for coin counting.
 */
static void send_wake(struct piuio *p)
{
	u8 buf[WAKE_LEN]; // Use stack allocation for small, fixed-size buffer

	if (!p || !p->hdev)
		return;

	buf[0] = WAKE_RID;
	memset(buf + 1, 0xFF, WAKE_LEN - 1);

	hid_hw_output_report(p->hdev, buf, WAKE_LEN);
	// Short delay recommended after sending wake/init sequence
	usleep_range(2000, 3000);
}

/**
 * get_matrix - Reads the button state Input Report 0x30 via USB control transfer.
 * Returns: 0 on success, negative error code or -EIO on failure.
 */
static int get_matrix(struct piuio *p)
{
	u16 wValue = (HID_INPUT_REPORT << 8) | RPT_ID_BTN;
	int r;

	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				wValue, p->iface, p->btn_buf, RPT_LEN_BTN,
				USB_CTRL_GET_TIMEOUT);

	if (r == RPT_LEN_BTN) {
		return 0; // Success
	}

	if (r >= 0) { // Wrong length received
		pr_warn_ratelimited("get_matrix: Incorrect length (%d != %d)\n", r, RPT_LEN_BTN);
		return -EIO;
	}
	// Negative error code from usb_control_msg
	pr_warn_ratelimited("get_matrix: usb_control_msg failed (%d)\n", r);
	return r;
}

/**
 * read_coin_feature_report - Reads the Feature Report 0x01 containing the coin counter.
 * Stores the full report in p->coin_buf.
 * Returns: 0 on success, negative error code or -EIO on failure.
 */
static int read_coin_feature_report(struct piuio *p)
{
	u16 wValue = (HID_FEATURE_REPORT << 8) | COIN_RID;
	int r;

	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				wValue, p->iface, p->coin_buf, COIN_BUF_LEN,
				USB_CTRL_GET_TIMEOUT);

	// Check if read succeeded and was long enough to contain the counter bytes
	if (r >= COIN_OFFSET_HI + 1) {
		return 0; // Success
	}

	if (r >= 0) { // Report received, but too short
		pr_warn_ratelimited("read_coin: Report too short (%d bytes, needed %d)\n", r, COIN_OFFSET_HI + 1);
		return -EIO;
	}
	// Negative error code from usb_control_msg
	pr_warn_ratelimited("read_coin: usb_control_msg failed (%d)\n", r);
	return r;
}


/* --- Input Event Processing --- */

/**
 * push_btn_events - Decodes button report buffer and sends input events.
 */
static void push_btn_events(struct piuio *p)
{
	bool now[NBTN] = { false }; // Track current state from this report read
	int i, b;

	// Decode button presses (active low) from the report buffer
	for (i = 0; i < 4; i++) // Bytes 0-3 for P1
		for (b = 0; b < 5; b++) // 5 buttons per player
			if (!(p->btn_buf[i] & map[b].mask))
				now[b] = true; // P1 button 'b' is pressed
	for (i = 4; i < 8; i++) // Bytes 4-7 for P2
		for (b = 0; b < 5; b++)
			if (!(p->btn_buf[i] & map[b].mask))
				now[5 + b] = true; // P2 button 'b' is pressed

	// Report changes in state compared to last known state (p->down)
	for (b = 0; b < NBTN; b++) {
		if (now[b] != p->down[b]) { // State changed for button 'b'
			// Map index 'b' back to the correct input event code
			u16 code = (b < 5) ? map[b].p1 : map[b - 5].p2;
			input_report_key(p->idev, code, now[b]);
			p->down[b] = now[b]; // Update last known state
		}
	}
}

/* --- Polling Workqueue and Timer --- */

/**
 * poll_work - Workqueue function to poll buttons and coin counter.
 */
static void poll_work(struct work_struct *w)
{
	// Get context 'p' from the global pointer 'ctx'
	// Assumption: 'ctx' is valid if work is running (checked by timer_cb and remove)
	struct piuio *p = ctx;
	u16 current_coin_16;
	static ktime_t last_coin_check = 0; // Time of the last coin check
	ktime_t now;

	if (!p || atomic_read(&p->stop)) // Double-check context and stop flag
		return;

	/* Poll Buttons */
	if (!get_matrix(p)) // Read button state into p->btn_buf
		push_btn_events(p); // Process buffer and send events if changed
	/* else: get_matrix failed, warning already printed by helper */


	/* Poll Coin Counter (less frequently) */
	now = ktime_get();
	// Check coin state roughly every 100ms
	if (last_coin_check == 0 || ktime_after(now, ktime_add_ms(last_coin_check, 100))) {
		if (!read_coin_feature_report(p)) { // Read coin state into p->coin_buf
			// Extract the current 16-bit coin value (Little Endian)
			current_coin_16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);

			// Check if the 16-bit value has changed since last read
			if (current_coin_16 != p->coin_last_16) {
				pr_info("Coin event: %u -> %u, sending %s press\n",
					p->coin_last_16, current_coin_16,
					input_event_get_name(EV_KEY, COIN_BTN_CODE));

				// Send configured coin button press/release pulse
				input_report_key(p->idev, COIN_BTN_CODE, 1); // Press
				input_sync(p->idev); // Ensure press is processed
				input_report_key(p->idev, COIN_BTN_CODE, 0); // Release

				// Update the last known coin value
				p->coin_last_16 = current_coin_16;
			}
		} /* else: read_coin_feature_report failed, warning printed by helper */
		last_coin_check = now; // Update time of last check
	}

	/* Sync all input events generated in this poll cycle */
	input_sync(p->idev);
}

/**
 * timer_cb - High-resolution timer callback to schedule polling work.
 */
static enum hrtimer_restart timer_cb(struct hrtimer *t)
{
	// Get context 'p' from the embedded timer struct
	struct piuio *p = container_of(t, struct piuio, timer);

	// Check stop flag before queueing work or restarting timer
	if (atomic_read(&p->stop))
		return HRTIMER_NORESTART;

	queue_work(system_unbound_wq, &p->poll_wq); // Queue the polling work

	// Reschedule the timer for the next poll interval
	hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
	return HRTIMER_RESTART;
}


/* --- HID Driver Probe and Remove --- */

static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r = 0;
	int b;

	pr_info("Probing PIUIO device VID=0x%04X PID=0x%04X...\n", id->vendor, id->product);

	if (ctx) { // Prevent multiple instances if already loaded
		pr_err("Device context already exists. Module already loaded?\n");
		return -EBUSY;
	}

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	/* Initialize context fields */
	p->hdev = hdev;
	p->udev = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	atomic_set(&p->stop, 0);
	hid_set_drvdata(hdev, p); // Associate context with hid_device

	/* Initialize HID communication */
	r = hid_parse(hdev); // Parse HID report descriptor
	if (r) { pr_err("hid_parse failed: %d\n", r); goto err_cleanup; }

	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW); // Start HID HW handling
	if (r) { pr_err("hid_hw_start failed: %d\n", r); goto err_cleanup; }
	p->hid_started = true;

	/* Send device initialization sequence: SET_IDLE and WAKE report */
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0), HID_REQ_SET_IDLE,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0, /* Idle Rate 0 = infinite */
			p->iface, NULL, 0, USB_CTRL_SET_TIMEOUT);
	/* Ignore return value of SET_IDLE for now */

	send_wake(p); // Send the 0x81 output report

	/* Setup Input Device */
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err_cleanup; }

	p->idev->name = "PIUIO Gamepad + Coin";
	p->idev->phys = hdev->phys; // Physical path from HID layer
	p->idev->uniq = hdev->uniq; // Unique ID from HID layer
	p->idev->id.bustype = hdev->bus;
	p->idev->id.vendor  = le16_to_cpu(hdev->vendor);
	p->idev->id.product = le16_to_cpu(hdev->product);
	p->idev->id.version = le16_to_cpu(hdev->version);
	p->idev->dev.parent = &hdev->dev; // Parent is the HID device

	set_bit(EV_KEY, p->idev->evbit); // This device reports button/key events

	/* Register capabilities for the 10 physical buttons */
	pr_info("Registering %d physical buttons...\n", NBTN);
	for (b = 0; b < NBTN; b++) {
		u16 code = (b < 5) ? map[b].p1 : map[b - 5].p2;
		set_bit(code, p->idev->keybit);
	}

	/* Register capability for the coin button */
	pr_info("Registering coin input as button: %s\n", input_event_get_name(EV_KEY, COIN_BTN_CODE));
	set_bit(COIN_BTN_CODE, p->idev->keybit);

	r = input_register_device(p->idev);
	if (r) { pr_err("Failed to register input device: %d\n", r); goto err_cleanup; }

	/* Read Initial Coin State */
	if (read_coin_feature_report(p)) {
		pr_warn("Failed initial coin read, defaulting count to 0.\n");
		p->coin_last_16 = 0; // Default on failure
	} else {
		p->coin_last_16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);
		pr_info("Initial coin count read: %u\n", p->coin_last_16);
	}

	/* Start Polling Timer */
	INIT_WORK(&p->poll_wq, poll_work);
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = timer_cb;

	// Assign global context *only* on full success, before starting timer
	ctx = p;

	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	pr_info("Device attached successfully. Polling every %d ms.\n", clamp_poll());
	return 0; // Success

err_cleanup: // Centralized cleanup for probe errors
	pr_err("Probe failed (%d). Cleaning up...\n", r);
	if (p && p->hid_started) // Only stop HID if it was successfully started
		hid_hw_stop(hdev);
	// devm_* allocations (p, p->idev) are freed automatically on error return.
	if (ctx == p) // Should not happen if probe fails, but clear defensively
		ctx = NULL;
	return r;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);

	pr_info("Removing PIUIO device...\n");
	if (!p) {
		pr_warn("No driver data found on remove.\n");
		return;
	}

	// Stop polling activities first
	// Check if this instance is the active one stored in 'ctx'
	if (ctx == p) {
		atomic_set(&p->stop, 1);      // Signal poll_work and timer_cb to stop
		hrtimer_cancel(&p->timer);    // Cancel pending timer callbacks
		cancel_work_sync(&p->poll_wq); // Wait for any running poll_work to finish
		ctx = NULL;                   // Clear the global context pointer
	} else {
		// This case might occur if probe failed after setting drvdata but before setting ctx,
		// or if multiple devices were somehow probed (though prevented by 'if (ctx)' check).
		pr_warn("Removing device that wasn't the active context.\n");
		// Still try to clean up this instance's resources
		atomic_set(&p->stop, 1);
		hrtimer_cancel(&p->timer);
		cancel_work_sync(&p->poll_wq);
	}

	// Stop HID hardware communication
	if (p->hid_started)
		hid_hw_stop(p->hdev);

	// input_unregister_device() and kfree(p) are handled by devm management
	// because we used devm_input_allocate_device and devm_kzalloc.

	pr_info("PIUIO device removed.\n");
}


/* --- HID Driver Boilerplate --- */
static const struct hid_device_id ids[] = {
	{ HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) },
	{ } /* Terminating entry */
};
MODULE_DEVICE_TABLE(hid, ids);

static struct hid_driver drv = {
	.name     = "piuio", // Matches pr_fmt prefix
	.id_table = ids,
	.probe    = probe,
	.remove   = remove,
};

module_hid_driver(drv); // Helper macro to register/unregister the HID driver

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo (Modified)");
MODULE_DESCRIPTION("PIUIO 0x1020 Gamepad + Coin Counter Driver (Coin as Button)");

