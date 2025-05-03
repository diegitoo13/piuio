// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter driver
 * Based on work by Diego Acevedo.
 * Cleaned up version with 16-bit coin detection.
 * Modified: Removed unused ratelimit state, mapped coin to BTN_SELECT.
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h> // Includes BTN_SELECT, BTN_MODE etc.
#include <linux/kernel.h>           /* clamp_val(), le16_to_cpu */
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ratelimit.h>        // Keep for pr_warn_ratelimited
#include <linux/ktime.h>
#include <linux/string.h>
#include <linux/delay.h>            /* usleep_range() */
#include <linux/byteorder/generic.h> /* le16_to_cpu */

/* --- REMOVED KEY_COIN fallback --- */
// #ifndef KEY_COIN
// #define KEY_COIN  KEY_KPENTER
// #endif

/* --- NEW: Define which button code to use for coin input --- */
// Options: BTN_SELECT, BTN_START, BTN_MODE, BTN_A, BTN_B, BTN_C, etc.
// WARNING: Original code uses BTN_SELECT for P2 West and BTN_START for P2 North.
// Choose one that doesn't conflict or adjust the map below if needed.
// Using BTN_MODE as an example of an often unused button.
#define COIN_BTN_CODE BTN_MODE

/* --- device IDs --- */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

/* --- reports --- */
#define RPT_ID_BTN      0x30
#define RPT_LEN_BTN     16
#define COIN_RID        0x01
#define COIN_OFFSET_LO  14 // Offset for Coin Counter Low Byte
#define COIN_OFFSET_HI  15 // Offset for Coin Counter High Byte
#define COIN_BUF_LEN    258
#define WAKE_RID        0x81
#define WAKE_LEN        19

/* --- button map --- */
struct mapent { u8 mask; u16 p1, p2; };
// NOTE: Original mapping uses BTN_SELECT and BTN_START.
// If you choose COIN_BTN_CODE to be one of these, you might get
// double inputs or need to change these mappings.
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH , BTN_TR      }, // P1 DownLeft, P2 UpRight
	{ 0x08, BTN_WEST  , BTN_SELECT  }, // P1 UpLeft,   P2 Select (Back/Coin on some pads)
	{ 0x04, BTN_NORTH , BTN_START   }, // P1 Center,   P2 Start
	{ 0x02, BTN_EAST  , BTN_THUMBL  }, // P1 DownRight,P2 Thumb Left (Unused on stock pads?)
	{ 0x10, BTN_TL    , BTN_THUMBR  }, // P1 UpRight,  P2 Thumb Right(Unused on stock pads?)
};
#define NBTN 10

/* --- context --- */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int               iface;
	bool              hid_started;
	u8   btn_buf[RPT_LEN_BTN];
	bool down[NBTN];
	u16  coin_last_16; // Store full 16-bit coin value
	u8   coin_buf[COIN_BUF_LEN]; // Buffer for the feature report
	struct input_dev  *idev;
	struct work_struct poll_wq;
	struct hrtimer     timer;
	atomic_t           stop;
};
static struct piuio *ctx;

/* --- module param --- */
static int poll_ms = 4; // Polling interval for buttons
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval for buttons in milliseconds (1-1000, default 4)");
static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* --- helpers --- */

/**
 * send_wake - Sends the Output Report 0x81 required for coin counting.
 */
static void send_wake(struct piuio *p)
{
	// Allocate buffer on stack instead of globally if it's only used here
	u8 *buf = kmalloc(WAKE_LEN, GFP_KERNEL);
	if (!buf) {
		pr_err("Failed to allocate memory for wake report\n");
		return;
	}
	if (!p || !p->hdev) {
		kfree(buf);
		return; // Basic check
	}
	buf[0] = WAKE_RID;
	memset(buf + 1, 0xFF, WAKE_LEN - 1);
	// Use hid_hw_raw_request for output reports if hid_hw_output_report gives issues
	// For now, keeping hid_hw_output_report as it was in the original code.
	hid_hw_output_report(p->hdev, buf, WAKE_LEN);
	kfree(buf);
	usleep_range(2000, 3000); // Short delay after wake
}


/**
 * get_matrix - Reads the button state Input Report 0x30.
 * Returns: 0 on success, negative error code or -EIO on failure.
 */
static int get_matrix(struct piuio *p)
{
	// HID_INPUT_REPORT is 1, HID_FEATURE_REPORT is 3
	// The request type for GET_REPORT uses ((report_type << 8) | report_id)
	u16 w = (HID_INPUT_REPORT << 8) | RPT_ID_BTN;
	int r;

	// Ensure buffer is allocated correctly. Use devm_kzalloc in probe if possible.
	// Assuming p->btn_buf is correctly allocated (it is in the struct).

	// Use hid_hw_raw_request for more direct control if usb_control_msg fails
	// int hid_hw_raw_request(struct hid_device *hdev, unsigned char reportnum, __u8 *buf, size_t len, unsigned char rtype, int reqtype);
	// r = hid_hw_raw_request(p->hdev, RPT_ID_BTN, p->btn_buf, RPT_LEN_BTN, HID_INPUT_REPORT, HID_REQ_GET_REPORT);
	// Check hid_hw_raw_request return value (usually returns bytes transferred or negative error)

	// Stick with usb_control_msg for now as per original code structure
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				w, p->iface, p->btn_buf, RPT_LEN_BTN, USB_CTRL_GET_TIMEOUT); // Use defined timeout

	if (r == RPT_LEN_BTN) {
		return 0; // Success
	} else if (r >= 0) {
		pr_warn_ratelimited("get_matrix: Incorrect length (%d != %d)\n", r, RPT_LEN_BTN);
		return -EIO;
	} else {
		pr_warn_ratelimited("get_matrix: usb_control_msg failed (%d)\n", r);
		// Consider stopping polling on repeated errors like -EPIPE?
		return r;
	}
}


/**
 * read_coin_feature_report - Reads the Feature Report 0x01 containing the coin counter.
 * Stores the full report in p->coin_buf.
 * Returns: 0 on success, negative error code or -EIO on failure.
 */
static int read_coin_feature_report(struct piuio *p)
{
	int r;
	u16 wValue = (HID_FEATURE_REPORT << 8) | COIN_RID; // report type 3, id 1

	// Use hid_hw_raw_request for FEATURE reports too
	// r = hid_hw_raw_request(p->hdev, COIN_RID, p->coin_buf, COIN_BUF_LEN, HID_FEATURE_REPORT, HID_REQ_GET_REPORT);

	// Stick with usb_control_msg for now
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				wValue, p->iface, p->coin_buf, COIN_BUF_LEN, USB_CTRL_GET_TIMEOUT); // Use defined timeout


	// Check if read succeeded and was long enough to contain the counter bytes
	if (r >= COIN_OFFSET_HI + 1) { // Need at least up to the high byte offset + 1 byte
		return 0; // Success
	} else { // Handle insufficient length OR negative error code
		if (r >= 0) {
			pr_warn_ratelimited("read_coin_feature_report: Report too short (%d bytes, needed %d)\n", r, COIN_OFFSET_HI + 1);
			return -EIO;
		} else {
			pr_warn_ratelimited("read_coin_feature_report: usb_control_msg failed (%d)\n", r);
			return r;
		}
	}
}


/* --- matrix -> events --- */
static void push_btn_events(struct piuio *p)
{
	bool now[NBTN] = { false }; int i, b;
	// Decode button presses from the report buffer (active low)
	for (i = 0; i < 4; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[b] = true; // Player 1 buttons
	for (i = 4; i < 8; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[5 + b] = true; // Player 2 buttons

	// Report changes in state
	for (b = 0; b < NBTN; b++) if (now[b] != p->down[b]) {
		input_report_key(p->idev, (b < 5) ? map[b].p1 : map[b - 5].p2, now[b]);
		p->down[b] = now[b];
	}
}

/* --- polling work / timer callback --- */
// static DEFINE_RATELIMIT_STATE(rl, HZ, 10); // REMOVED - Unused variable causing warning
static void poll_work(struct work_struct *w)
{
	u16 current_coin_16;

	// Ensure context is valid and module isn't stopping
	// Need proper locking if context could be removed while work is queued/running
	// but the current stop mechanism with atomic_t and cancel_work_sync is generally okay.
	if (!ctx || atomic_read(&ctx->stop)) return;

	// --- Poll Buttons ---
	if (!get_matrix(ctx)) {
		// Optional debug: Uncomment if needed, but now without rate limiting
		// pr_info("Button matrix: %*phN\n", RPT_LEN_BTN, ctx->btn_buf);
		push_btn_events(ctx); // Process button presses/releases
	} // else: get_matrix failed, warning already printed by helper

	// --- Poll Coin Counter (less frequently) ---
	// Use a static variable within the function for the last check time
	static ktime_t last_coin_check = 0; // Initialize to 0
	ktime_t now = ktime_get();

	// Check coin ~ every 100ms. Use ktime_after for cleaner comparison.
	if (last_coin_check == 0 || ktime_after(now, ktime_add_ms(last_coin_check, 100))) {
		// Read the feature report containing the coin counter
		if (!read_coin_feature_report(ctx)) { // If read succeeded

			// Extract the current 16-bit coin value (Little Endian)
			current_coin_16 = le16_to_cpu(*(__le16 *)&ctx->coin_buf[COIN_OFFSET_LO]);

			// Check if the 16-bit value has changed
			if (current_coin_16 != ctx->coin_last_16) {
				pr_info("Coin event detected! Count changed (%u -> %u), sending %s press\n",
					ctx->coin_last_16, current_coin_16, input_event_get_name(EV_KEY, COIN_BTN_CODE));

				// --- MODIFIED: Send chosen button code press/release pulse ---
				input_report_key(ctx->idev, COIN_BTN_CODE, 1); // Press
				input_sync(ctx->idev); // Ensure press event is processed immediately
				input_report_key(ctx->idev, COIN_BTN_CODE, 0); // Release

				// Update the last known value
				ctx->coin_last_16 = current_coin_16;
			}
		} // else: read_coin_feature_report failed, warning already printed by helper
		last_coin_check = now; // Update time of last check regardless of success
	}

	// Sync any input events (buttons or coin) generated in this poll cycle
	input_sync(ctx->idev);
}

static enum hrtimer_restart timer_cb(struct hrtimer *t)
{
	struct piuio *p = container_of(t, struct piuio, timer); // Get context from timer

	// Check stop condition before queueing work
	if (atomic_read(&p->stop))
		return HRTIMER_NORESTART;

	queue_work(system_unbound_wq, &p->poll_wq); // Queue the work

	// Check stop condition again before rescheduling timer
	if (atomic_read(&p->stop))
		return HRTIMER_NORESTART;

	hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
	return HRTIMER_RESTART; // Reschedule timer
}


/* --- probe / remove --- */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r = 0; int b;

	pr_info("Probing for PIUIO device VID=0x%04X PID=0x%04X...\n", id->vendor, id->product);
	// Use a local context pointer 'p' until fully successful, then assign to global 'ctx'.
	// Avoid using global 'ctx' directly during probing.

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) { return -ENOMEM; } // devm_kzalloc logs failure

	p->hdev = hdev;
	p->udev = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	p->hid_started = false;
	atomic_set(&p->stop, 0);
	// Associate 'p' with the hid_device early on
	hid_set_drvdata(hdev, p);

	// --- Essential HID Initialization ---
	r = hid_parse(hdev);
	if (r) { pr_err("hid_parse failed: %d\n", r); goto err_cleanup; } // Use goto for cleanup

	// Try opening the device. hid_hw_start can be problematic. Let's use hid_hw_open.
	// r = hid_hw_open(hdev);
	// if (r) { pr_err("hid_hw_open failed: %d\n", r); goto err_cleanup; }
	// For polling drivers like this, hid_hw_start might be okay. Keep original.
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (r) { pr_err("hid_hw_start failed: %d\n", r); goto err_cleanup; }
	p->hid_started = true; // Mark HID as started *after* success

	// --- Initialization: SET_IDLE then ONE WAKE call ---
	// Send SET_IDLE(0) to device - infinite idle rate (reports only on change, though we poll)
	// Some devices need this. Using usb_control_msg directly is okay here.
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0), HID_REQ_SET_IDLE,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0, /* Idle Rate 0 = infinite */
			p->iface, NULL, 0, USB_CTRL_SET_TIMEOUT); // Use defined timeout
	// Consider checking the return value of SET_IDLE, though it's often non-critical

	send_wake(p); // Send the wake/init sequence

	// --- Input Device Setup ---
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err_cleanup; } // devm_ functions handle cleanup on failure

	p->idev->name = "PIUIO Gamepad + Coin";
	p->idev->phys = hdev->phys; // Use HID physical path
	p->idev->uniq = hdev->uniq; // Use HID unique ID
	p->idev->id.bustype = hdev->bus; // Use HID bus type (BUS_USB)
	p->idev->id.vendor  = le16_to_cpu(hdev->vendor); // Use actual vendor/product
	p->idev->id.product = le16_to_cpu(hdev->product);
	p->idev->id.version = le16_to_cpu(hdev->version);
	p->idev->dev.parent = &hdev->dev; // Set parent device

	set_bit(EV_KEY, p->idev->evbit); // We will send button events

	// Register the 10 physical buttons
	pr_info("Registering %d physical buttons...\n", NBTN);
	for (b = 0; b < NBTN; b++) {
		u16 code = (b < 5) ? map[b].p1 : map[b - 5].p2;
		pr_debug("Registering button %d: %s\n", b, input_event_get_name(EV_KEY, code));
		set_bit(code, p->idev->keybit);
	}

	// --- MODIFIED: Register the chosen coin button code ---
	pr_info("Registering coin input as button: %s\n", input_event_get_name(EV_KEY, COIN_BTN_CODE));
	set_bit(COIN_BTN_CODE, p->idev->keybit);
	// --- REMOVED: set_bit(KEY_COIN, p->idev->keybit); ---

	r = input_register_device(p->idev);
	if (r) { pr_err("Failed to register input device: %d\n", r); goto err_cleanup; }

	// --- Read Initial Coin State (AFTER wake call and input device setup) ---
	if (read_coin_feature_report(p)) { // Read into p->coin_buf
		pr_warn("Failed initial coin read, defaulting count to 0.\n");
		p->coin_last_16 = 0; // Default on failure
	} else {
		p->coin_last_16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);
		pr_info("Initial coin count read: %u\n", p->coin_last_16);
	}

	// --- Start Polling ---
	INIT_WORK(&p->poll_wq, poll_work);
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = timer_cb;

	// Assign global context *before* starting the timer that uses it indirectly
	ctx = p;

	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	pr_info("Device attached successfully. Polling buttons every %d ms.\n", clamp_poll());
	return 0; // Success!

err_cleanup: // Label for centralized cleanup on error
	pr_err("Probe failed (%d). Cleaning up...\n", r);
	// Resources allocated with devm_ are automatically cleaned up by the kernel
	// when probe returns an error.
	// We only need to explicitly clean up things not managed by devm,
	// or things created *before* the error occurred that devm wouldn't know about.
	if (p && p->hid_started) {
		hid_hw_stop(hdev); // Stop HID communication if it was started
	}
	// If we assigned ctx, clear it on failure.
	if (ctx == p) {
		ctx = NULL;
	}
	// hid_set_drvdata(hdev, NULL); // devm handles the driver data association
	return r; // Return the error code
}


static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);
	pr_info("Removing PIUIO device...\n");
	if (!p) {
		pr_warn("No driver data found on remove.\n");
		return;
	}

	// Check if this is the active context before manipulating global state
	if (ctx == p) {
		// Stop polling activities gracefully
		atomic_set(&p->stop, 1);      // Signal work and timer to stop
		hrtimer_cancel(&p->timer);    // Cancel the timer
		cancel_work_sync(&p->poll_wq); // Wait for any queued work to finish
		ctx = NULL;                   // Clear the global context pointer
	} else {
		pr_warn("Removing device that wasn't the active context?\n");
		// Still attempt cleanup, but be cautious
		atomic_set(&p->stop, 1); // Set stop flag anyway
		hrtimer_cancel(&p->timer);
		cancel_work_sync(&p->poll_wq);
	}

	// Stop HID hardware communication if it was started
	// This should happen *after* stopping polling that might use HID functions
	if (p->hid_started) {
		hid_hw_stop(p->hdev);
		p->hid_started = false; // Mark as stopped
	}

	// Input device unregistration and freeing of 'p' and 'p->idev'
	// are handled automatically by devm management when the device is removed,
	// because they were allocated using devm_kzalloc and devm_input_allocate_device
	// and input_register_device succeeded.

	pr_info("PIUIO device removed.\n");
	// hid_set_drvdata(hdev, NULL); // Kernel handles this on device removal
}


/* --- boiler-plate --- */
static const struct hid_device_id ids[] = { { HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) }, {} };
MODULE_DEVICE_TABLE(hid, ids);

static struct hid_driver drv = {
	.name="piuio",      // Driver name
	.id_table=ids,       // Matching device IDs
	.probe=probe,        // Probe function
	.remove=remove,      // Remove function
	// Add suspend/resume handlers if power management is needed
	// .suspend = piuio_suspend,
	// .resume = piuio_resume,
};

// Replace module_hid_driver(drv) with module_init/module_exit if more control needed
// but module_hid_driver is standard for HID drivers.
// static int __init piuio_init(void) { return hid_register_driver(&drv); }
// static void __exit piuio_exit(void) { hid_unregister_driver(&drv); }
// module_init(piuio_init);
// module_exit(piuio_exit);

module_hid_driver(drv); // Use helper macro for registration

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo (Modified)");
MODULE_DESCRIPTION("PIUIO 0x1020 - Gamepad + Coin (Mapped to Button, Cleaned)");
