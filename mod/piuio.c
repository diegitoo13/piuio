// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter driver
 * Based on work by Diego Acevedo.
 * Cleaned up version with 16-bit coin detection.
 * Modified: Coin mapped to BTN_MODE, unused 'rl' removed.
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h> // Needed for BTN_MODE etc.
#include <linux/kernel.h>           /* clamp_val(), le16_to_cpu, container_of */
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ratelimit.h>        // Needed for pr_warn_ratelimited
#include <linux/ktime.h>
#include <linux/string.h>           // Needed for memset
#include <linux/delay.h>            /* usleep_range() */
#include <linux/byteorder/generic.h> /* le16_to_cpu */
#include <linux/atomic.h>           /* Needed for atomic_t */

/* --- Define which button code to use for coin input --- */
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
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH , BTN_TR      }, { 0x08, BTN_WEST  , BTN_SELECT  },
	{ 0x04, BTN_NORTH , BTN_START   }, { 0x02, BTN_EAST  , BTN_THUMBL  },
	{ 0x10, BTN_TL    , BTN_THUMBR  },
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
	// Use stack allocation as buffer is small and fixed size
	u8 buf[WAKE_LEN];
	if (!p || !p->hdev) return;
	buf[0] = WAKE_RID;
	memset(buf + 1, 0xFF, WAKE_LEN - 1);
	hid_hw_output_report(p->hdev, buf, WAKE_LEN);
	usleep_range(2000, 3000); // Short delay after wake
}

/**
 * get_matrix - Reads the button state Input Report 0x30.
 * Returns: 0 on success, negative error code or -EIO on failure.
 */
static int get_matrix(struct piuio *p)
{
	u16 w = (HID_INPUT_REPORT << 8) | RPT_ID_BTN;
	int r;
	// Use standard timeout value
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				w, p->iface, p->btn_buf, RPT_LEN_BTN, USB_CTRL_GET_TIMEOUT);
	if (r == RPT_LEN_BTN) {
		return 0;
	} else if (r >= 0) {
		pr_warn_ratelimited("get_matrix: Incorrect length (%d != %d)\n", r, RPT_LEN_BTN);
		return -EIO;
	} else {
		pr_warn_ratelimited("get_matrix: usb_control_msg failed (%d)\n", r);
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
	u16 wValue = (HID_FEATURE_REPORT << 8) | COIN_RID;

	// Read the full feature report into the context buffer
	// Use standard timeout value
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				wValue, p->iface, p->coin_buf, COIN_BUF_LEN, USB_CTRL_GET_TIMEOUT);

	// Check if read succeeded and was long enough to contain the counter bytes
	if (r >= COIN_OFFSET_HI + 1) { // Need at least up to the high byte offset + 1 byte
		return 0; // Success
	} else { // Handle insufficient length OR negative error code
		if (r >= 0) {
			pr_warn_ratelimited("read_coin: Report too short (%d bytes, needed %d)\n", r, COIN_OFFSET_HI + 1);
			return -EIO;
		} else {
			pr_warn_ratelimited("read_coin: usb_control_msg failed (%d)\n", r);
			return r;
		}
	}
}

/* --- matrix -> events --- */
static void push_btn_events(struct piuio *p)
{
	bool now[NBTN] = { false }; int i, b;
	for (i = 0; i < 4; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[b] = true;
	for (i = 4; i < 8; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[5 + b] = true;
	for (b = 0; b < NBTN; b++) if (now[b] != p->down[b]) {
		input_report_key(p->idev, (b < 5) ? map[b].p1 : map[b - 5].p2, now[b]);
		p->down[b] = now[b];
	}
}

/* --- polling work / timer callback --- */
// static DEFINE_RATELIMIT_STATE(rl, HZ, 10); // REMOVED: Unused variable
static void poll_work(struct work_struct *w)
{
	// Get context 'p' from the global pointer 'ctx'
	struct piuio *p = ctx;
	u16 current_coin_16;
	static ktime_t last_coin_check = 0;
	ktime_t now;

	// Ensure context is valid and module isn't stopping
	if (!p || atomic_read(&p->stop)) return;

	// --- Poll Buttons ---
	if (!get_matrix(p)) {
		// Optional debug for button state - removed unused __ratelimit call
		// pr_debug("Button matrix: %*phN\n", RPT_LEN_BTN, p->btn_buf);
		push_btn_events(p); // Process button presses/releases
	} // else: get_matrix failed, warning already printed by helper

	// --- Poll Coin Counter (less frequently) ---
	now = ktime_get();

	// Check coin ~ every 100ms. Use ktime_after for robustness.
	if (last_coin_check == 0 || ktime_after(now, ktime_add_ms(last_coin_check, 100))) {
		// Read the feature report containing the coin counter
		if (!read_coin_feature_report(p)) { // If read succeeded

			// Extract the current 16-bit coin value (Little Endian)
			current_coin_16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);

			// Check if the 16-bit value has changed
			if (current_coin_16 != p->coin_last_16) {
				pr_info("Coin event detected! Count changed (%u -> %u), sending %s press\n",
					p->coin_last_16, current_coin_16,
					input_event_get_name(EV_KEY, COIN_BTN_CODE)); // Log the button name

				// --- MODIFIED: Send chosen button code press/release pulse ---
				input_report_key(p->idev, COIN_BTN_CODE, 1); // Press
				input_sync(p->idev); // Ensure press event is processed
				input_report_key(p->idev, COIN_BTN_CODE, 0); // Release

				// Update the last known value
				p->coin_last_16 = current_coin_16;
			}
		} // else: read_coin_feature_report failed, warning already printed by helper
		last_coin_check = now; // Update time of last check
	}

	// Sync any input events (buttons or coin) generated in this poll cycle
	input_sync(p->idev);
}

// Modified timer_cb to match cleaned up version's logic
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


/* --- probe / remove --- */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r = 0; int b;

	pr_info("Probing for PIUIO device VID=0x%04X PID=0x%04X...\n", id->vendor, id->product);
	if (ctx) { pr_err("Device context already exists.\n"); return -EBUSY; }

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) { return -ENOMEM; }

	p->hdev = hdev; p->udev = interface_to_usbdev(itf); p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	p->hid_started = false; atomic_set(&p->stop, 0);
	hid_set_drvdata(hdev, p);

	// --- Essential HID Initialization ---
	r = hid_parse(hdev);
	if (r) { pr_err("hid_parse failed: %d\n", r); goto err_cleanup; } // Use goto for cleanup
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (r) { pr_err("hid_hw_start failed: %d\n", r); goto err_cleanup; }
	p->hid_started = true;

	// --- Initialization: SET_IDLE then ONE WAKE call ---
	// Use standard timeout value
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0), HID_REQ_SET_IDLE,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0, p->iface, NULL, 0, USB_CTRL_SET_TIMEOUT);
	send_wake(p); // Single wake call

	// --- Input Device Setup ---
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err_cleanup; }
	p->idev->name = "PIUIO Gamepad + Coin";
	// Use HID device properties for better integration
	p->idev->phys = hdev->phys;
	p->idev->uniq = hdev->uniq;
	p->idev->id.bustype = hdev->bus;
	p->idev->id.vendor  = le16_to_cpu(hdev->vendor);
	p->idev->id.product = le16_to_cpu(hdev->product);
	p->idev->id.version = le16_to_cpu(hdev->version);
	p->idev->dev.parent = &hdev->dev;

	set_bit(EV_KEY, p->idev->evbit);
	// Register physical buttons
	for (b = 0; b < NBTN; b++) { set_bit((b < 5) ? map[b].p1 : map[b - 5].p2, p->idev->keybit); }
	// --- MODIFIED: Register the chosen coin button code ---
	set_bit(COIN_BTN_CODE, p->idev->keybit);
	// --- REMOVED: set_bit(KEY_COIN, p->idev->keybit); ---

	r = input_register_device(p->idev);
	if (r) { pr_err("Failed to register input device: %d\n", r); goto err_cleanup; }

	// --- Read Initial Coin State (AFTER wake call) ---
	if (read_coin_feature_report(p)) { // Read into p->coin_buf
		pr_warn("Failed initial coin read, defaulting count to 0.\n");
		p->coin_last_16 = 0; // Default on failure
	} else {
		// Extract initial 16-bit value if read succeeded
		p->coin_last_16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);
		pr_info("Initial coin count read: %u\n", p->coin_last_16);
	}

	// --- Start Polling ---
	INIT_WORK(&p->poll_wq, poll_work);
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = timer_cb;

	// Assign global context *before* starting the timer
	ctx = p;

	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	pr_info("Device attached successfully. Polling buttons every %d ms.\n", clamp_poll());
	return 0; // Success

err_cleanup: // Use goto label for centralized error cleanup
	pr_err("Probe failed (%d). Cleaning up...\n", r);
	if (p && p->hid_started) // Only stop if started
		hid_hw_stop(hdev);
	// idev and p are devm managed, automatically freed on error return
	if (ctx == p) // Clear global ctx if it was assigned before failure
		ctx = NULL;
	return r;
}

// Use remove function logic from cleaned up version for robustness
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
	if (p->hid_started) {
		hid_hw_stop(p->hdev);
		p->hid_started = false; // Mark as stopped
	}

	// devm handles input device unregister/free and context 'p' free
	pr_info("PIUIO device removed.\n");
}

/* --- boiler-plate --- */
static const struct hid_device_id ids[] = { { HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) }, {} };
MODULE_DEVICE_TABLE(hid, ids);
static struct hid_driver drv = { .name="piuio_gp", .id_table=ids, .probe=probe, .remove=remove, };
module_hid_driver(drv); // Use helper macro for registration
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo (Modified)"); // Update author
MODULE_DESCRIPTION("PIUIO 0x1020 - Gamepad + Coin (Mapped to Button)"); // Update description

