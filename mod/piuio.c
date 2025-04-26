// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 polled HID  ► 10-button game-pad
 * • Five byte-patterns per player
 * • Button = held if *any* of the player's 4 bytes equals that pattern
 * 2025  Diego Acevedo
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/ratelimit.h>

/* ───────── constants / IDs ─────────────────────────────────────── */
#define VID_PIUIO      0x0d2f
#define PID_PIUIO1020  0x1020

#define REP_ID         0x30
#define REP_LEN        9            /* 1-byte ID + 8 sensor bytes        */

/* ───────── mapping table (edit freely) ─────────────────────────── */
struct mapent { u8 value; u16 btn_p1; u16 btn_p2; };

static const struct mapent mapping[5] = {
	/*  val    P1-button             P2-button            */
	{ 0xFE, BTN_SOUTH,  BTN_TR      },
	{ 0xF7, BTN_WEST,   BTN_SELECT  },
	{ 0xFB, BTN_NORTH,  BTN_START   },
	{ 0xFD, BTN_EAST,   BTN_THUMBL  },
	{ 0xEF, BTN_TL,     BTN_THUMBR  },
};
#define NUM_BTN 10  /* 5 per player */

/* ───────── private data ────────────────────────────────────────── */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int                iface;

	struct input_dev  *idev;
	u8                 buf[REP_LEN];
	bool               down[NUM_BTN];

	struct hrtimer     tim;
	struct work_struct work;
	atomic_t           shutting;
};
static struct piuio *ctx;

/* ───────── param: poll interval (ms) ───────────────────────────── */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval in ms (1-1000)");

static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* ───────── USB helpers ─────────────────────────────────────────── */
static int get_report(struct piuio *p)
{
	u16 wVal = (HID_INPUT_REPORT << 8) | REP_ID;
	int ret  = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev,0),
				   HID_REQ_GET_REPORT,
				   USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
				   wVal, p->iface,
				   p->buf, REP_LEN, 30);
	return (ret == REP_LEN) ? 0 : (ret < 0 ? ret : -EIO);
}
static void set_idle(struct piuio *p)
{
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev,0),
			HID_REQ_SET_IDLE,
			USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
			0, p->iface, NULL, 0, 10);
}

/* ───────── translate sensor bytes → button events ──────────────── */
static void push_events(struct piuio *p)
{
	struct input_dev *id = p->idev;
	bool now[NUM_BTN] = { false };
	int i, b;

	/* player-1 bytes 0-3 ---------------------------------------- */
	for (i = 0; i < 4; ++i) {
		u8 v = p->buf[1 + i];
		if (v == 0xFF) continue;
		for (b = 0; b < 5; ++b)
			if (v == mapping[b].value)
				now[b] = true;
	}
	/* player-2 bytes 4-7 ---------------------------------------- */
	for (i = 4; i < 8; ++i) {
		u8 v = p->buf[1 + i];
		if (v == 0xFF) continue;
		for (b = 0; b < 5; ++b)
			if (v == mapping[b].value)
				now[5 + b] = true;
	}

	/* emit diffs -------------------------------------------------- */
	for (b = 0; b < NUM_BTN; ++b)
		if (now[b] != p->down[b]) {
			input_report_key(id,
			   (b < 5) ? mapping[b].btn_p1 : mapping[b-5].btn_p2,
			   now[b]);
			p->down[b] = now[b];
		}
	input_sync(id);
}

/* ───────── polling machinery ───────────────────────────────────── */
static DEFINE_RATELIMIT_STATE(rl, HZ, 10);

static void poll_work(struct work_struct *w)
{
	if (!ctx || atomic_read(&ctx->shutting)) return;

	if (!get_report(ctx)) {
		if (__ratelimit(&rl))
			pr_debug("%*phN\n", 8, &ctx->buf[1]);
		push_events(ctx);
	}
}
static enum hrtimer_restart t_cb(struct hrtimer *t)
{
	if (ctx && !atomic_read(&ctx->shutting))
		queue_work(system_unbound_wq, &ctx->work);

	hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
	return atomic_read(&ctx->shutting) ? HRTIMER_NORESTART
					   : HRTIMER_RESTART;
}

/* ───────── probe / remove ──────────────────────────────────────── */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int ret, b;

	if (ctx) return -EBUSY;

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) return -ENOMEM;

	p->hdev  = hdev;
	p->udev  = interface_to_usbdev(intf);
	p->iface = intf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev, p);

	if ((ret = hid_parse(hdev)) ||
	    (ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW)))
		return ret;
	set_idle(p);

	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { ret = -ENOMEM; goto err_hw; }

	p->idev->name       = "PIUIO 10-Btn Gamepad";
	p->idev->phys       = "piuio/js0";
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VID_PIUIO;
	p->idev->id.product = PID_PIUIO1020;

	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NUM_BTN; ++b)
		set_bit((b < 5) ? mapping[b].btn_p1 : mapping[b-5].btn_p2,
			p->idev->keybit);
	if ((ret = input_register_device(p->idev)))
		goto err_hw;

	ctx = p;
	INIT_WORK(&p->work, poll_work);
	hrtimer_init(&p->tim, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->tim.function = t_cb;
	hrtimer_start(&p->tim, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);
	pr_info("game-pad ready – poll %d ms\n", clamp_poll());
	return 0;

err_hw:
	hid_hw_stop(hdev);
	return ret;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);
	if (!p) return;

	atomic_set(&p->shutting, 1);
	hrtimer_cancel(&p->tim);
	cancel_work_sync(&p->work);
	ctx = NULL;

	input_unregister_device(p->idev);
	hid_hw_stop(hdev);
}

/* ───────── boiler-plate ───────────────────────────────────────── */
static const struct hid_device_id ids[] = {
	{ HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) }, { }
};
MODULE_DEVICE_TABLE(hid, ids);

static struct hid_driver drv = {
	.name  = "piuio",
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};
module_hid_driver(drv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020 10-button game-pad driver");