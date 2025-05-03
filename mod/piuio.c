// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter driver
 * Based on work by Diego Acevedo.
 * Cleaned up version with 16-bit coin detection.
 * Coin counter mapped to BTN_MODE.
 * Re-adding minimal logs for wake debug and reverted SET_IDLE timeout.
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/ratelimit.h>
#include <linux/ktime.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#include <linux/atomic.h> // Include for atomic_t

/* --- coin button mapping --- */
#define COIN_BTN_CODE BTN_MODE

/* --- USB device IDs --- */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

/* --- report IDs & sizes --- */
#define RPT_ID_BTN      0x30
#define RPT_LEN_BTN     16
#define COIN_RID        0x01
#define COIN_OFFSET_LO  14
#define COIN_OFFSET_HI  15
#define COIN_BUF_LEN    258
#define WAKE_RID        0x81
#define WAKE_LEN        19

/* --- button matrix mapping --- */
struct mapent { u8 mask; u16 p1, p2; };
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH, BTN_TR    },
	{ 0x08, BTN_WEST,  BTN_SELECT},
	{ 0x04, BTN_NORTH, BTN_START },
	{ 0x02, BTN_EAST,  BTN_THUMBL},
	{ 0x10, BTN_TL,    BTN_THUMBR},
};
#define NBTN 10

/* --- driver context --- */
struct piuio {
	struct hid_device  *hdev;
	struct usb_device  *udev;
	int                 iface;
	bool                hid_started;
	u8                  btn_buf[RPT_LEN_BTN];
	bool                down[NBTN];
	u16                 coin_last_16;
	u8                  coin_buf[COIN_BUF_LEN];
	struct input_dev   *idev;
	struct work_struct  poll_wq;
	struct hrtimer      timer;
	atomic_t            stop; // Use atomic_t type
};
static struct piuio *ctx; // Global context pointer

/* --- module parameter: polling interval --- */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Button-poll interval (1–1000 ms, default 4)");
static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* --- send the “wake” output report (0x81) --- */
static void send_wake(struct piuio *p)
{
	u8 buf[WAKE_LEN];

	// Basic check
	if (!p || !p->hdev) {
		pr_err("send_wake: Invalid context passed!\n");
		return;
	}

	// Minimal log added back
	pr_info("send_wake: Sending wake report...\n");

	buf[0] = WAKE_RID;
	memset(buf + 1, 0xFF, WAKE_LEN - 1);

	// The core HID command to send the wake report
	hid_hw_output_report(p->hdev, buf, WAKE_LEN);

	// Small delay, potentially needed by device firmware
	usleep_range(2000, 3000);

	// Minimal log added back
	pr_info("send_wake: Wake report potentially sent.\n");
}

/* --- read button matrix (Input report 0x30) --- */
static int get_matrix(struct piuio *p)
{
	u16 w = (HID_INPUT_REPORT << 8) | RPT_ID_BTN;
	int r;

	// Check for valid context pointers needed for usb_control_msg
	if (!p || !p->udev) return -ENODEV;

	r = usb_control_msg(p->udev,
	                       usb_rcvctrlpipe(p->udev, 0),
	                       HID_REQ_GET_REPORT,
	                       USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	                       w, p->iface,
	                       p->btn_buf, RPT_LEN_BTN,
	                       USB_CTRL_GET_TIMEOUT); // Keep original timeout from user code

	// Check result with clearer structure
	if (r == RPT_LEN_BTN) {
		return 0; // Success
	} else if (r >= 0) { // Wrong length
		pr_warn_ratelimited("get_matrix: bad length %d (expected %d)\n",
		                    r, RPT_LEN_BTN);
		return -EIO;
	} else { // Negative error code
		pr_warn_ratelimited("get_matrix: usb error %d\n", r);
		return r;
	}
}

/* --- read full coin feature report (0x01) --- */
static int read_coin_feature_report(struct piuio *p)
{
	u16 w = (HID_FEATURE_REPORT << 8) | COIN_RID;
	int r;

	// Check for valid context pointers
	if (!p || !p->udev) return -ENODEV;

	r = usb_control_msg(p->udev,
	                       usb_rcvctrlpipe(p->udev, 0),
	                       HID_REQ_GET_REPORT,
	                       USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	                       w, p->iface,
	                       p->coin_buf, COIN_BUF_LEN,
	                       USB_CTRL_GET_TIMEOUT); // Keep original timeout from user code

	// Check result with clearer structure
	if (r >= COIN_OFFSET_HI + 1) {
		return 0; // Success, buffer should contain enough data
	} else if (r >= 0) { // Report too short
		pr_warn_ratelimited("read_coin: too short %d (need %d)\n",
		                    r, COIN_OFFSET_HI + 1);
		return -EIO;
	} else { // Negative error code
		pr_warn_ratelimited("read_coin: usb error %d\n", r);
		return r;
	}
}

/* --- translate matrix -> input events --- */
static void push_btn_events(struct piuio *p)
{
	bool now[NBTN] = { false };
	int i, b;

	// Check for valid context pointers
	if (!p || !p->idev) return;

	/* first 4 bytes map to buttons 0–4 */
	for (i = 0; i < 4; i++)
		for (b = 0; b < 5; b++)
			if (!(p->btn_buf[i] & map[b].mask))
				now[b] = true;

	/* next 4 bytes map to buttons 5–9 */
	for (i = 4; i < 8; i++)
		for (b = 0; b < 5; b++)
			if (!(p->btn_buf[i] & map[b].mask))
				now[5 + b] = true;

	/* report any changes */
	for (b = 0; b < NBTN; b++) {
		if (now[b] != p->down[b]) {
			input_report_key(p->idev,
			                 (b < 5) ? map[b].p1 : map[b - 5].p2,
			                 now[b]);
			p->down[b] = now[b];
		}
	}
}

/* --- workqueue handler: poll buttons + coin counter --- */
static DEFINE_RATELIMIT_STATE(rl_get_matrix_err, HZ, 5); // Separate rate limit states
static DEFINE_RATELIMIT_STATE(rl_read_coin_err, HZ, 5); // for different warnings if desired

static void poll_work(struct work_struct *w)
{
	// Note: container_of is used in timer_cb, so 'p' should be derived there.
	// Here we rely on the global 'ctx' as before, ensure probe sets it correctly.
	struct piuio *p = ctx;
	u16           coin16;
	static ktime_t last_check; // Coin check timer
	ktime_t        now = ktime_get();

	if (!p || atomic_read(&p->stop)) // Check global ctx and stop flag
		return;

	/* buttons every cycle */
	if (get_matrix(p) == 0) { // If get_matrix succeeded
		push_btn_events(p);
	} // else: Error already printed by get_matrix (ratelimited)


	/* coin every ~100ms */
	// Initialize last_check on first run
	if (unlikely(last_check == 0)) last_check = now;

	if (ktime_after(now, ktime_add_ms(last_check, 100))) {

		if (read_coin_feature_report(p) == 0) { // If read succeeded
			coin16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);
			if (coin16 != p->coin_last_16) {
				// Use standard pr_info for coin events
				pr_info("Coin count changed %u -> %u, sending %s\n",
				        p->coin_last_16, coin16, "BTN_MODE");
				input_report_key(p->idev, COIN_BTN_CODE, 1);
				input_sync(p->idev); // Sync after press
				input_report_key(p->idev, COIN_BTN_CODE, 0);
				// input_sync(p->idev); // Sync after release (optional, one sync at end is ok)
				p->coin_last_16 = coin16;
			}
		} // else: Error already printed by read_coin_feature_report (ratelimited)
		last_check = now; // Update time of last check regardless of success
	}

	input_sync(p->idev); // Sync all events at the end of the poll cycle
}


/* --- hrtimer callback to re-queue our work --- */
static enum hrtimer_restart timer_cb(struct hrtimer *t)
{
	// Get context pointer from timer struct
	struct piuio *p = container_of(t, struct piuio, timer);

	// Check stop flag using the derived pointer 'p'
	if (!p || atomic_read(&p->stop))
		return HRTIMER_NORESTART;

	// Queue work using the unbound workqueue
	queue_work(system_unbound_wq, &p->poll_wq);

	// Reschedule timer
	hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
	return HRTIMER_RESTART;
}


/* --- HID probe --- */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int          r, b;

	pr_info("Probing PIUIO %04x:%04x\n", id->vendor, id->product);
	if (ctx) { // Check if already loaded (simple check)
		pr_err("Device/driver already loaded?\n");
		return -EBUSY;
	}

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) { return -ENOMEM; }

	// Initialize context
	p->hdev  = hdev;
	p->udev  = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	atomic_set(&p->stop, 0); // Initialize stop flag
	p->hid_started = false;  // Initialize HID status flag

	hid_set_drvdata(hdev, p); // Associate context with HID device

	// --- HID Core Initialization ---
	pr_info("probe: Parsing HID descriptor...\n");
	r = hid_parse(hdev);
	if (r) { pr_err("hid_parse failed: %d\n", r); return r; }

	pr_info("probe: Starting HID hardware...\n");
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (r) { pr_err("hid_hw_start failed: %d\n", r); return r; }
	p->hid_started = true; // Mark HID HW as successfully started
	pr_info("probe: hid_hw_start successful.\n");

	/* wake up coin counter sequence */
	pr_info("probe: Sending SET_IDLE...\n");
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev,0),
	                HID_REQ_SET_IDLE,
	                USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
	                0, p->iface, NULL, 0,
					// *** Use 100ms timeout, like Python script ***
	                100); // Timeout in ms
	// Note: Ignoring return value of SET_IDLE as before

	pr_info("probe: Calling send_wake...\n");
	send_wake(p); // Single call to send the wake report

	/* set up input device */
	pr_info("probe: Setting up input device...\n");
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err; }

	// Populate input device info from hid_device (as per user's code)
	p->idev->name       = "PIUIO Gamepad + Coin";
	p->idev->phys       = hdev->phys;
	p->idev->uniq       = hdev->uniq;
	p->idev->id.bustype = hdev->bus;
	p->idev->id.vendor  = le16_to_cpu(hdev->vendor);
	p->idev->id.product = le16_to_cpu(hdev->product);
	p->idev->id.version = le16_to_cpu(hdev->version);
	p->idev->dev.parent = &hdev->dev; // Link to parent device

	// Declare supported event types and codes
	set_bit(EV_KEY, p->idev->evbit); // We will send button events
	// Declare the 10 standard buttons
	for (b = 0; b < NBTN; b++)
		set_bit((b < 5) ? map[b].p1 : map[b - 5].p2, p->idev->keybit);
	// Declare the coin button (mapped to BTN_MODE)
	set_bit(COIN_BTN_CODE, p->idev->keybit);

	// Register the input device with the kernel
	r = input_register_device(p->idev);
	if (r) { pr_err("input_register_device failed: %d\n", r); goto err; }

	/* initial coin count */
	pr_info("probe: Reading initial coin count...\n");
	if (read_coin_feature_report(p)) { // Read report into p->coin_buf
		p->coin_last_16 = 0; // Default to 0 on read failure
		pr_warn("Failed initial coin read, defaulting count to 0.\n");
	} else {
		p->coin_last_16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);
		// Keep minimal log for initial count
		pr_info("Initial coin count: %u\n", p->coin_last_16);
	}

	/* start polling */
	pr_info("probe: Starting polling...\n");
	INIT_WORK(&p->poll_wq, poll_work); // Initialize work struct
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL); // Initialize timer
	p->timer.function = timer_cb; // Assign timer callback

	// Assign global context *before* starting timer
	// (Timer callback uses container_of, but poll_work currently uses global ctx)
	ctx = p;

	// Start the timer
	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	pr_info("Attached, polling every %d ms\n", clamp_poll());
	return 0; // Success

err:
	pr_err("probe: Error during setup (err = %d), cleaning up...\n", r);
	// Cleanup only happens if probe fails mid-way
	if (p->hid_started)
		hid_hw_stop(hdev); // Stop HID communication if it was started
	// devm_* functions handle freeing p and p->idev if allocated
	ctx = NULL; // Ensure global ctx is NULL if probe fails
	return r; // Return the error code
}

/* --- HID remove --- */
static void remove(struct hid_device *hdev)
{
	// Retrieve context - essential first step
	struct piuio *p = hid_get_drvdata(hdev);

	pr_info("Removing PIUIO device...\n");

	// Check if context is valid (it should be if probe succeeded)
	if (!p) {
		pr_warn("remove: No context found!\n");
		return;
	}

	// Stop polling mechanisms *only* if this is the active context 'ctx'
	// Also check against 'p' derived from hdev for safety
	if (ctx == p) {
		atomic_set(&p->stop, 1); // Signal work and timer to stop
		hrtimer_cancel(&p->timer); // Cancel timer synchronously
		cancel_work_sync(&p->poll_wq); // Cancel work synchronously
		ctx = NULL; // Clear global context
		pr_info("remove: Polling stopped and context cleared.\n");
	} else {
        // This might happen if remove is called after probe failed but before ctx was set,
        // or if somehow ctx points elsewhere (shouldn't happen with -EBUSY check).
        pr_warn("remove: Context mismatch (ctx=%p, p=%p), polling not stopped.\n", ctx, p);
    }


	// Stop HID hardware communication if it was started
	if (p->hid_started) {
		hid_hw_stop(p->hdev);
		p->hid_started = false; // Update flag (optional here, as 'p' is going away)
		pr_info("remove: HID hardware stopped.\n");
	}

	// Input device unregistration/freeing is handled by devm

	pr_info("PIUIO device removed.\n");
}

/* --- Module boilerplate --- */
static const struct hid_device_id ids[] = {
	{ HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) },
	{ } /* terminating entry */
};
MODULE_DEVICE_TABLE(hid, ids);

static struct hid_driver drv = {
	.name     = "piuio_gp", // Use different name than original hid-generic if needed
	.id_table = ids,
	.probe    = probe,
	.remove   = remove,
	// Add other callbacks like .raw_event here if needed later
};
// Use module_hid_driver helper macro for registration/unregistration
module_hid_driver(drv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo (Modified)");
MODULE_DESCRIPTION("PIUIO 0x1020 - Gamepad + Coin (Mapped to Button, Wake Debug)");
