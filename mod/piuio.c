// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter driver
 * Based on work by Diego Acevedo.
 * Cleaned up version with 16-bit coin detection.
 * Coin counter mapped to BTN_MODE.
 * Attempting wake via direct usb_control_msg.
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
#include <linux/atomic.h>
#include <linux/slab.h> // Needed for kmalloc/kfree

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
	{ 0x01, BTN_SOUTH, BTN_TR    }, { 0x08, BTN_WEST,  BTN_SELECT},
	{ 0x04, BTN_NORTH, BTN_START }, { 0x02, BTN_EAST,  BTN_THUMBL},
	{ 0x10, BTN_TL,    BTN_THUMBR},
};
#define NBTN 10

/* --- driver context --- */
struct piuio { /* ... same context struct ... */
	struct hid_device  *hdev; struct usb_device  *udev; int iface; bool hid_started;
	u8 btn_buf[RPT_LEN_BTN]; bool down[NBTN]; u16 coin_last_16; u8 coin_buf[COIN_BUF_LEN];
	struct input_dev   *idev; struct work_struct  poll_wq; struct hrtimer timer; atomic_t stop;
};
static struct piuio *ctx;

/* --- module parameter: polling interval --- */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Button-poll interval (1–1000 ms, default 4)");
static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* --- send the “wake” output report (0x81) using direct usb_control_msg --- */
static void send_wake(struct piuio *p)
{
	u8 *buf = NULL; // Use kmalloc for buffer
	int ret;

	// Basic check
	if (!p || !p->udev) {
		pr_err("send_wake: Invalid context/udev pointer!\n");
		return;
	}

	// Allocate buffer dynamically
	buf = kmalloc(WAKE_LEN, GFP_KERNEL);
	if (!buf) {
		pr_err("send_wake: Failed to allocate buffer\n");
		return;
	}

	// Prepare the payload (Report ID 0x81 + 18 x 0xFF)
	buf[0] = WAKE_RID; // Report ID = 0x81
	memset(buf + 1, 0xFF, WAKE_LEN - 1); // Payload = 0xFF * 18

	pr_info("send_wake: Sending wake report via usb_control_msg...\n");

	// Mimic Python's ctrl_transfer(0x21, 0x09, 0x0281, IFACE, data, TO)
	ret = usb_control_msg(
		p->udev,                          // device
		usb_sndctrlpipe(p->udev, 0),      // pipe (Control OUT endpoint 0)
		0x09,                             // bRequest (HID SET_REPORT)
		0x21,                             // bmRequestType (Host-to-Device, Class, Interface)
		0x0281,                           // wValue (Report Type Output=2 << 8 | Report ID 0x81)
		p->iface,                         // wIndex (Interface number)
		buf,                              // data buffer
		WAKE_LEN,                         // wLength (size of data)
		100                               // timeout (ms, matching Python's TO)
	);

	// Check the return value of usb_control_msg
	if (ret < 0) {
		pr_err("send_wake: usb_control_msg failed! Error: %d\n", ret);
	} else {
		pr_info("send_wake: usb_control_msg potentially sent %d bytes.\n", ret);
	}

	// Free the allocated buffer
	kfree(buf);

	// Delay after sending removed for now to match Python closer
	// usleep_range(2000, 3000);
}

/* --- read button matrix (Input report 0x30) --- */
static int get_matrix(struct piuio *p) { /* ... same as before ... */
	u16 w = (HID_INPUT_REPORT << 8) | RPT_ID_BTN; int r; if (!p || !p->udev) return -ENODEV;
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0), HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, w, p->iface, p->btn_buf, RPT_LEN_BTN, USB_CTRL_GET_TIMEOUT);
	if (r == RPT_LEN_BTN) { return 0; } else if (r >= 0) { pr_warn_ratelimited("get_matrix: bad length %d (exp %d)\n", r, RPT_LEN_BTN); return -EIO; } else { pr_warn_ratelimited("get_matrix: usb error %d\n", r); return r; }
}

/* --- read full coin feature report (0x01) --- */
static int read_coin_feature_report(struct piuio *p) { /* ... same as before ... */
	u16 w = (HID_FEATURE_REPORT << 8) | COIN_RID; int r; if (!p || !p->udev) return -ENODEV;
	r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0), HID_REQ_GET_REPORT, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE, w, p->iface, p->coin_buf, COIN_BUF_LEN, USB_CTRL_GET_TIMEOUT);
	if (r >= COIN_OFFSET_HI + 1) { return 0; } else if (r >= 0) { pr_warn_ratelimited("read_coin: too short %d (need %d)\n", r, COIN_OFFSET_HI + 1); return -EIO; } else { pr_warn_ratelimited("read_coin: usb error %d\n", r); return r; }
}

/* --- translate matrix -> input events --- */
static void push_btn_events(struct piuio *p) { /* ... same as before ... */
	bool now[NBTN] = { false }; int i, b; if (!p || !p->idev) return;
	for (i = 0; i < 4; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[b] = true;
	for (i = 4; i < 8; i++) for (b = 0; b < 5; b++) if (!(p->btn_buf[i] & map[b].mask)) now[5 + b] = true;
	for (b = 0; b < NBTN; b++) if (now[b] != p->down[b]) { input_report_key(p->idev, (b < 5) ? map[b].p1 : map[b - 5].p2, now[b]); p->down[b] = now[b]; }
}

/* --- workqueue handler: poll buttons + coin counter --- */
static DEFINE_RATELIMIT_STATE(rl_get_matrix_err, HZ, 5);
static DEFINE_RATELIMIT_STATE(rl_read_coin_err, HZ, 5);
static void poll_work(struct work_struct *w) { /* ... same as before ... */
	struct piuio *p = ctx; u16 coin16; static ktime_t last_check; ktime_t now = ktime_get();
	if (!p || atomic_read(&p->stop)) return;
	if (get_matrix(p) == 0) { push_btn_events(p); }
	if (unlikely(last_check == 0)) last_check = now;
	if (ktime_after(now, ktime_add_ms(last_check, 100))) {
		if (read_coin_feature_report(p) == 0) { coin16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]);
			if (coin16 != p->coin_last_16) { pr_info("Coin count changed %u -> %u, sending %s\n", p->coin_last_16, coin16, "BTN_MODE");
				input_report_key(p->idev, COIN_BTN_CODE, 1); input_sync(p->idev); input_report_key(p->idev, COIN_BTN_CODE, 0); p->coin_last_16 = coin16; } }
		last_check = now; }
	input_sync(p->idev);
}

/* --- hrtimer callback to re-queue our work --- */
static enum hrtimer_restart timer_cb(struct hrtimer *t) { /* ... same as before ... */
	struct piuio *p = container_of(t, struct piuio, timer); if (!p || atomic_read(&p->stop)) return HRTIMER_NORESTART;
	queue_work(system_unbound_wq, &p->poll_wq); hrtimer_forward_now(t, ms_to_ktime(clamp_poll())); return HRTIMER_RESTART;
}


/* --- HID probe --- */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int          r, b;

	pr_info("Probing PIUIO %04x:%04x\n", id->vendor, id->product);
	if (ctx) { pr_err("Device/driver already loaded?\n"); return -EBUSY; }
	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL); if (!p) return -ENOMEM;
	p->hdev = hdev; p->udev = interface_to_usbdev(itf); p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	atomic_set(&p->stop, 0); p->hid_started = false;
	hid_set_drvdata(hdev, p);

	pr_info("probe: Parsing HID descriptor...\n");
	r = hid_parse(hdev); if (r) { pr_err("hid_parse failed: %d\n", r); return r; }

	pr_info("probe: Starting HID hardware...\n");
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW); if (r) { pr_err("hid_hw_start failed: %d\n", r); return r; } p->hid_started = true;
	pr_info("probe: hid_hw_start successful.\n");

	/* wake up coin counter sequence */
	pr_info("probe: Sending SET_IDLE...\n");
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev,0), HID_REQ_SET_IDLE, USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE, 0, p->iface, NULL, 0, 100);

	// Note: send_wake now uses usb_control_msg internally
	send_wake(p);

	/* set up input device */
	pr_info("probe: Setting up input device...\n");
	p->idev = devm_input_allocate_device(&hdev->dev); if (!p->idev) { r = -ENOMEM; goto err; }
	p->idev->name = "PIUIO Gamepad + Coin"; p->idev->phys = hdev->phys; p->idev->uniq = hdev->uniq;
	p->idev->id.bustype = hdev->bus; p->idev->id.vendor = le16_to_cpu(hdev->vendor); p->idev->id.product = le16_to_cpu(hdev->product); p->idev->id.version = le16_to_cpu(hdev->version);
	p->idev->dev.parent = &hdev->dev;
	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NBTN; b++) set_bit((b < 5) ? map[b].p1 : map[b - 5].p2, p->idev->keybit);
	set_bit(COIN_BTN_CODE, p->idev->keybit);
	r = input_register_device(p->idev); if (r) { pr_err("input_register_device failed: %d\n", r); goto err; }

	/* initial coin count */
	pr_info("probe: Reading initial coin count...\n");
	if (read_coin_feature_report(p)) { p->coin_last_16 = 0; pr_warn("Failed initial coin read, default 0.\n"); }
	else { p->coin_last_16 = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_OFFSET_LO]); pr_info("Initial coin count: %u\n", p->coin_last_16); }

	/* start polling */
	pr_info("probe: Starting polling...\n");
	INIT_WORK(&p->poll_wq, poll_work); hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL); p->timer.function = timer_cb;
	ctx = p; // Assign global context
	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	pr_info("Attached, polling every %d ms\n", clamp_poll());
	return 0; // Success

err:
	pr_err("probe: Error during setup (err = %d), cleaning up...\n", r);
	if (p->hid_started) hid_hw_stop(hdev);
	ctx = NULL; return r;
}

/* --- HID remove --- */
static void remove(struct hid_device *hdev) { /* ... same as before ... */
	struct piuio *p = hid_get_drvdata(hdev); pr_info("Removing PIUIO device...\n"); if (!p) return;
	if (ctx == p) { atomic_set(&p->stop, 1); hrtimer_cancel(&p->timer); cancel_work_sync(&p->poll_wq); ctx = NULL; pr_info("remove: Polling stopped.\n"); }
	else { pr_warn("remove: Context mismatch.\n"); }
	if (p->hid_started) { hid_hw_stop(p->hdev); pr_info("remove: HID hardware stopped.\n"); }
	pr_info("PIUIO device removed.\n");
}


/* --- Module boilerplate --- */
static const struct hid_device_id ids[] = { { HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) }, {} };
MODULE_DEVICE_TABLE(hid, ids);
static struct hid_driver drv = { .name="piuio_gp", .id_table=ids, .probe=probe, .remove=remove, };
module_hid_driver(drv);
MODULE_LICENSE("GPL v2"); MODULE_AUTHOR("Diego Acevedo (Modified)"); MODULE_DESCRIPTION("PIUIO 0x1020 - Gamepad+Coin (BTN_MODE, Direct Wake Msg)");
