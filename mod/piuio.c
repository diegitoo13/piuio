// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter driver
 * Based on work by Diego Acevedo.
 * Cleaned up version with 16-bit coin detection.
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <linux/kernel.h>           /* clamp_val(), le16_to_cpu */
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ratelimit.h>
#include <linux/ktime.h>
#include <linux/string.h>
#include <linux/delay.h>            /* usleep_range() */
#include <linux/byteorder/generic.h> /* le16_to_cpu */


/* --- KEY_COIN fallback --- */
#ifndef KEY_COIN
#define KEY_COIN  KEY_KPENTER
#endif

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
	u8 buf[WAKE_LEN];
	if (!p || !p->hdev) return; // Basic check
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
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				w, p->iface, p->btn_buf, RPT_LEN_BTN, 30);
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
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				wValue, p->iface, p->coin_buf, COIN_BUF_LEN, 30);

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
	for (i = 0; i < 4; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[b] = true;
	for (i = 4; i < 8; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[5 + b] = true;
	for (b = 0; b < NBTN; b++) if (now[b] != p->down[b]) {
		input_report_key(p->idev, (b < 5) ? map[b].p1 : map[b - 5].p2, now[b]);
		p->down[b] = now[b];
	}
}

/* --- polling work / timer callback --- */
static DEFINE_RATELIMIT_STATE(rl, HZ, 10); // For warnings/debug
static void poll_work(struct work_struct *w)
{
	u16 current_coin_16;

	// Ensure context is valid and module isn't stopping
	if (!ctx || atomic_read(&ctx->stop)) return;

	// --- Poll Buttons ---
	// Note: If EPIPE errors returned previously, this is the place to disable first.
	if (!get_matrix(ctx)) {
		// Optional debug for button state - leave commented unless needed
		// if (__ratelimit(&rl)) pr_debug("Button matrix: %*phN\n", RPT_LEN_BTN, ctx->btn_buf);
		push_btn_events(ctx); // Process button presses/releases
	} // else: get_matrix failed, warning already printed by helper

	// --- Poll Coin Counter (less frequently) ---
	static ktime_t last_coin_check;
	ktime_t now = ktime_get();

	// Check coin ~ every 100ms
	if (ktime_ms_delta(now, last_coin_check) >= 100) {
		// Read the feature report containing the coin counter
		if (!read_coin_feature_report(ctx)) { // If read succeeded

			// Extract the current 16-bit coin value (Little Endian)
			// Use kernel helper for safety and correct endian conversion
			current_coin_16 = le16_to_cpu(*(__le16 *)&ctx->coin_buf[COIN_OFFSET_LO]);
			// Alternative manual way:
			// current_coin_16 = ctx->coin_buf[COIN_OFFSET_LO] | (ctx->coin_buf[COIN_OFFSET_HI] << 8);

			// Check if the 16-bit value has changed
			if (current_coin_16 != ctx->coin_last_16) {
				pr_info("Coin event detected! Count changed (%u -> %u)\n",
					ctx->coin_last_16, current_coin_16);

				// Send KEY_COIN press/release pulse
				input_report_key(ctx->idev, KEY_COIN, 1); // Press
				input_sync(ctx->idev); // Ensure press event is processed
				input_report_key(ctx->idev, KEY_COIN, 0); // Release

				// Update the last known value
				ctx->coin_last_16 = current_coin_16;
			}
		} // else: read_coin_feature_report failed, warning already printed by helper
		last_coin_check = now; // Update time of last check
	}

	// Sync any input events (buttons or coin) generated in this poll cycle
	input_sync(ctx->idev);
}

static enum hrtimer_restart timer_cb(struct hrtimer *t)
{
	if (ctx && !atomic_read(&ctx->stop)) queue_work(system_unbound_wq, &ctx->poll_wq);
	if (!atomic_read(&ctx->stop)) {
		hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
		return HRTIMER_RESTART;
	}
	return HRTIMER_NORESTART;
}


/* --- probe / remove --- */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r = 0; int b;

	// Minimal entry logging
	pr_info("Probing for PIUIO device VID=0x%04X PID=0x%04X...\n", id->vendor, id->product);
	if (ctx) { pr_err("Device context already exists.\n"); return -EBUSY; }

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) { return -ENOMEM; } // Error logged by kernel allocation failure

	p->hdev = hdev; p->udev = interface_to_usbdev(itf); p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	p->hid_started = false; atomic_set(&p->stop, 0);
	hid_set_drvdata(hdev, p);

	// --- Essential HID Initialization ---
	r = hid_parse(hdev);
	if (r) { pr_err("hid_parse failed: %d\n", r); return r; }
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (r) { pr_err("hid_hw_start failed: %d\n", r); return r; }
	p->hid_started = true;

	// --- Initialization: SET_IDLE then ONE WAKE call ---
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0), HID_REQ_SET_IDLE,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0, p->iface, NULL, 0, 10);
	send_wake(p); // Single wake call

	// --- Input Device Setup ---
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err_hw; }
	p->idev->name = "PIUIO Gamepad + Coin"; p->idev->phys = "piuio/input0";
	p->idev->id.bustype = BUS_USB; p->idev->id.vendor  = VID_PIUIO; p->idev->id.product = PID_PIUIO1020;
	p->idev->dev.parent = &hdev->dev;
	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NBTN; b++) { set_bit((b < 5) ? map[b].p1 : map[b - 5].p2, p->idev->keybit); }
	set_bit(KEY_COIN, p->idev->keybit);
	r = input_register_device(p->idev);
	if (r) { pr_err("Failed to register input device: %d\n", r); goto err_hw; }

	// --- Read Initial Coin State (AFTER single wake call) ---
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
	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	ctx = p; // Assign global context only on full success
	pr_info("Device attached successfully. Polling buttons every %d ms.\n", clamp_poll());
	return 0; // Success

err_hw:
	pr_err("Probe failed during setup. Cleaning up...\n");
	// Cleanup resources allocated before failure occurred
	if (p->hid_started) hid_hw_stop(hdev);
	// idev and p are devm managed, automatically freed on error return
	if (ctx == p) ctx = NULL; // Should not happen if probe fails, but be safe
	return r;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);
	// Minimal logging for remove
	pr_info("Removing PIUIO device...\n");
	if (!p) return;
	if (ctx == p) { // Only stop polling if this is the active context
		atomic_set(&p->stop, 1);
		hrtimer_cancel(&p->timer);
		cancel_work_sync(&p->poll_wq);
	}
	// input_unregister_device is handled implicitly by devm on device removal
	// if registered via devm_input_allocate_device and input_register_device succeeds.
	// If probe failed before register, devm handles idev free.
	// Explicit unregister might be needed if NOT using devm for idev, but we are.
	// However, let's keep it for clarity if registration succeeded.
	// We need to track registration success if we want perfect cleanup.
	// For now, assume devm + kernel handles input device unregister/free.

	// Explicitly stop HID HW communication if started
	if (p->hid_started) hid_hw_stop(p->hdev);
	if (ctx == p) ctx = NULL; // Clear global context
	pr_info("PIUIO device removed.\n");
}

/* --- boiler-plate --- */
static const struct hid_device_id ids[] = { { HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) }, {} };
MODULE_DEVICE_TABLE(hid, ids);
static struct hid_driver drv = { .name="piuio_gp", .id_table=ids, .probe=probe, .remove=remove, };
module_hid_driver(drv); // Use helper macro for registration
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020 - Gamepad + Coin (Cleaned, 16-bit coin)");
