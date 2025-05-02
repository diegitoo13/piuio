// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter
 * ... (rest of header comments) ...
 * --- Debugging Note ---
 * This version attempts initialization:
 * - Wake command sent only ONCE after SET_IDLE.
 * - Initial coin state IS read during probe (after wake) to prevent false events.
 * Includes pr_info/pr_err messages. Monitor kernel logs.
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

/* --- KEY_COIN fallback --- */
#ifndef KEY_COIN
#define KEY_COIN  KEY_KPENTER
#endif

/* --- device IDs --- */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

/* --- reports --- */
#define RPT_ID_BTN  0x30
#define RPT_LEN_BTN 16
#define COIN_RID        0x01
#define COIN_OFFSET_HI  15
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
	u8   coin_last; // Will be read from device in probe
	u8   coin_buf[COIN_BUF_LEN];
	struct input_dev  *idev;
	struct work_struct poll_wq;
	struct hrtimer     timer;
	atomic_t           stop;
};
static struct piuio *ctx;

/* --- module param --- */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval in milliseconds (1-1000, default 4)");
static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* --- helpers (send_wake, get_matrix, read_coin_hi) --- */
// No changes to the helper function implementations
static void send_wake(struct piuio *p) { /* ... same as before ... */
	u8 buf[WAKE_LEN]; if (!p || !p->hdev) { pr_err("send_wake: Invalid context\n"); return; }
	pr_info("send_wake: Preparing buffer...\n"); buf[0] = WAKE_RID; memset(buf + 1, 0xFF, WAKE_LEN - 1);
	pr_info("send_wake: Calling hid_hw_output_report...\n"); hid_hw_output_report(p->hdev, buf, WAKE_LEN);
	pr_info("send_wake: Call finished. Applying delay...\n"); usleep_range(2000, 3000); pr_info("send_wake: Delay finished.\n");
}
static int get_matrix(struct piuio *p) { /* ... same as before ... */
	u16 w = (HID_INPUT_REPORT << 8) | RPT_ID_BTN; int r;
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0), HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, w, p->iface, p->btn_buf, RPT_LEN_BTN, 30);
	if (r == RPT_LEN_BTN) return 0; if (r >= 0) { pr_warn_ratelimited("get_matrix: Incorrect length (%d != %d)\n", r, RPT_LEN_BTN); return -EIO; }
	pr_warn_ratelimited("get_matrix: usb_control_msg failed (%d)\n", r); return r;
}
static int read_coin_hi(struct piuio *p, u8 *hi) { /* ... same as before ... */
	int r; u16 wValue = (HID_FEATURE_REPORT << 8) | COIN_RID;
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0), HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, wValue, p->iface, p->coin_buf, COIN_BUF_LEN, 30);
	if (r < COIN_OFFSET_HI + 1) { if (r >= 0) { pr_warn_ratelimited("read_coin_hi: Report too short (%d bytes)\n", r); return -EIO; } pr_warn_ratelimited("read_coin_hi: usb_control_msg failed (%d)\n", r); return r; }
	*hi = p->coin_buf[COIN_OFFSET_HI]; return 0;
}


/* --- matrix -> events (push_btn_events) --- */
// No changes needed here
static void push_btn_events(struct piuio *p) { /* ... same as before ... */
	bool now[NBTN] = { false }; int i, b;
	for (i = 0; i < 4; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[b] = true;
	for (i = 4; i < 8; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[5 + b] = true;
	for (b = 0; b < NBTN; b++) if (now[b] != p->down[b]) { input_report_key(p->idev, (b < 5) ? map[b].p1 : map[b - 5].p2, now[b]); p->down[b] = now[b]; }
}


/* --- polling work / timer callback --- */
// No changes needed to the polling logic itself yet.
// Still recommend disabling get_matrix call inside poll_work if EPIPE persists.
static DEFINE_RATELIMIT_STATE(rl, HZ, 10);
static void poll_work(struct work_struct *w) { /* ... same as before ... */
	if (!ctx || atomic_read(&ctx->stop)) return;
	/* *** Recommendation: Still consider commenting this block out if EPIPE errors continue *** */
	if (!get_matrix(ctx)) { if (__ratelimit(&rl)) pr_debug("Button matrix: %*phN\n", RPT_LEN_BTN, ctx->btn_buf); push_btn_events(ctx); }
	static ktime_t last_coin_check; ktime_t now = ktime_get();
	if (ktime_ms_delta(now, last_coin_check) >= 100) {
		u8 current_coin_hi;
		if (!read_coin_hi(ctx, &current_coin_hi)) { if (current_coin_hi != ctx->coin_last) { pr_info("Coin event detected! (0x%02X -> 0x%02X)\n", ctx->coin_last, current_coin_hi); input_report_key(ctx->idev, KEY_COIN, 1); input_sync(ctx->idev); input_report_key(ctx->idev, KEY_COIN, 0); ctx->coin_last = current_coin_hi; } }
		last_coin_check = now;
	} input_sync(ctx->idev);
}
static enum hrtimer_restart timer_cb(struct hrtimer *t) { /* ... same as before ... */
	if (ctx && !atomic_read(&ctx->stop)) queue_work(system_unbound_wq, &ctx->poll_wq);
	if (!atomic_read(&ctx->stop)) { hrtimer_forward_now(t, ms_to_ktime(clamp_poll())); return HRTIMER_RESTART; } return HRTIMER_NORESTART;
}


/* --- probe / remove --- */

static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r = 0; int b;

	pr_info("Probe function entered for VID=0x%04X PID=0x%04X\n", id->vendor, id->product);
	if (ctx) { pr_err("Device context already exists.\n"); return -EBUSY; }

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) { pr_err("Failed context allocation.\n"); return -ENOMEM; }
	pr_info("Device context allocated.\n");

	p->hdev = hdev; p->udev = interface_to_usbdev(itf); p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	p->hid_started = false; atomic_set(&p->stop, 0);
	hid_set_drvdata(hdev, p);
	pr_info("Context basic fields initialized.\n");

	// --- Essential HID Initialization ---
	pr_info("Parsing HID report descriptor...\n");
	r = hid_parse(hdev);
	if (r) { pr_err("hid_parse failed! Error %d\n", r); return r; }
	pr_info("hid_parse OK.\n");

	pr_info("Starting HID hardware communication (hid_hw_start)...\n");
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (r) { pr_err("hid_hw_start failed! Error %d\n", r); return r; }
	p->hid_started = true;
	pr_info("hid_hw_start OK.\n");

	// --- Initialization: SET_IDLE then ONE WAKE call ---
	pr_info("Sending SET_IDLE request...\n");
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0),
			HID_REQ_SET_IDLE, USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0, p->iface, NULL, 0, 10);

	pr_info("Calling send_wake (Single attempt, mirroring Python)...\n");
	send_wake(p); // Single wake call

	// --- Input Device Setup ---
	pr_info("Allocating input device...\n");
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { pr_err("Failed input alloc.\n"); r = -ENOMEM; goto err_hw; }
	pr_info("Input device allocated.\n");

	p->idev->name = "PIUIO Gamepad + Coin"; p->idev->phys = "piuio/input0";
	p->idev->id.bustype = BUS_USB; p->idev->id.vendor  = VID_PIUIO; p->idev->id.product = PID_PIUIO1020;
	p->idev->dev.parent = &hdev->dev;
	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NBTN; b++) { set_bit((b < 5) ? map[b].p1 : map[b - 5].p2, p->idev->keybit); }
	set_bit(KEY_COIN, p->idev->keybit);
	pr_info("Input device properties and key bits configured.\n");

	pr_info("Registering input device...\n");
	r = input_register_device(p->idev);
	if (r) { pr_err("Failed input register! Error %d\n", r); goto err_hw; }
	pr_info("Input device registered successfully.\n");

	// --- Read Initial Coin State (AFTER single wake call) ---
	// *** MODIFICATION: Read initial coin state here ***
	pr_info("Reading initial coin state (high byte) post-wake...\n");
	if (read_coin_hi(p, &p->coin_last)) {
		// If read fails, log warning and default the value
		pr_warn("Failed to read initial coin state during probe, defaulting hi-byte to 0.\n");
		p->coin_last = 0;
	} else {
		// Log the successfully read initial value
		pr_info("Initial coin hi-byte read successfully: 0x%02X\n", p->coin_last);
	}
	// *** End of MODIFICATION ***

	// --- Start Polling ---
	pr_info("Initializing and starting polling timer/workqueue...\n");
	INIT_WORK(&p->poll_wq, poll_work);
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = timer_cb;
	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	ctx = p;
	pr_info("Device attached successfully. Polling interval: %d ms.\n", clamp_poll());
	return 0; // Success

err_hw:
	pr_err("Probe failed during setup. Cleaning up...\n");
	if (p->hid_started) hid_hw_stop(hdev);
	if (ctx == p) ctx = NULL;
	return r;
}

static void remove(struct hid_device *hdev) { /* ... same as before ... */
	struct piuio *p = hid_get_drvdata(hdev); pr_info("Remove function entered.\n");
	if (!p) { pr_warn("Remove: No private data.\n"); return; }
	if (ctx == p) { pr_info("Stopping polling...\n"); atomic_set(&p->stop, 1); hrtimer_cancel(&p->timer); cancel_work_sync(&p->poll_wq); pr_info("Polling stopped.\n"); }
	else { pr_warn("Remove: Instance mismatch.\n"); }
	if (p->idev) { pr_info("Unregistering input device...\n"); input_unregister_device(p->idev); } else { pr_info("No input device registered.\n"); }
	if (p->hid_started) { pr_info("Stopping HID hardware...\n"); hid_hw_stop(p->hdev); } else { pr_info("HID hardware not started.\n"); }
	if (ctx == p) { pr_info("Clearing global context.\n"); ctx = NULL; } pr_info("Remove function finished.\n");
}

/* --- boiler-plate --- */
static const struct hid_device_id ids[] = { { HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) }, {} };
MODULE_DEVICE_TABLE(hid, ids);
static struct hid_driver drv = { .name="piuio_gp", .id_table=ids, .probe=probe, .remove=remove, };
module_hid_driver(drv);
MODULE_LICENSE("GPL v2"); MODULE_AUTHOR("Diego Acevedo"); MODULE_DESCRIPTION("PIUIO 0x1020 - Gamepad+Coin (Debug, Init fix attempt 2)");
