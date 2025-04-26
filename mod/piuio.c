// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 polled HID  →  10-button game-pad (bit-level)
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

/* ───────── IDs & report layout ────────────────────────────────── */
#define VID_PIUIO       0x0d2f
#define PID_PIUIO1020   0x1020
#define REP_ID          0x30
#define REP_LEN         17          /* 1-byte ID + 16 sensor bytes */

/* ───────── mapping table (edit freely) ─────────────────────────── */
struct mapent { u8 mask; u16 btn_p1; u16 btn_p2; };

static const struct mapent map[5] = {
	/* bit ↓     P1 code         P2 code          */
	{ 0x01, BTN_SOUTH,  BTN_TR      },  /* DL */
	{ 0x08, BTN_WEST,   BTN_SELECT  },  /* DR */
	{ 0x04, BTN_NORTH,  BTN_START   },  /* UL */
	{ 0x02, BTN_EAST,   BTN_THUMBL  },  /* UR */
	{ 0x10, BTN_TL,     BTN_THUMBR  },  /* Center */
};
#define NBTN 10     /* 5 per player */

/* ───────── private state ───────────────────────────────────────── */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int                iface;

	struct input_dev  *idev;
	u8                 buf[REP_LEN];
	bool               down[NBTN];

	struct hrtimer     timer;
	struct work_struct work;
	atomic_t           stop;
};
static struct piuio *ctx;

/* ───────── module param – poll rate (ms) ───────────────────────── */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval in ms (1-1000)");
static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* ───────── USB helpers ─────────────────────────────────────────── */
static int get_report(struct piuio *p)
{
	u16 w = (HID_INPUT_REPORT << 8) | REP_ID;
	int r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev,0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
				w, p->iface, p->buf, REP_LEN, 30);
	return (r == REP_LEN) ? 0 : (r < 0 ? r : -EIO);
}
static void set_idle(struct piuio *p)
{
	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev,0),
			HID_REQ_SET_IDLE,
			USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
			0, p->iface, NULL, 0, 10);
}

/* ───────── translate 16 bytes → 10 button events ──────────────── */
static void push_events(struct piuio *p)
{
	struct input_dev *id = p->idev;
	bool now[NBTN] = { false };
	int i, b;

	/* Player-1  bytes 0-3 */
	for (i = 0; i < 4; ++i) {
		u8 v = p->buf[1 + i];
		for (b = 0; b < 5; ++b)
			if (!(v & map[b].mask))
				now[b] = true;
	}
	/* Player-2  bytes 4-7 */
	for (i = 4; i < 8; ++i) {
		u8 v = p->buf[1 + i];
		for (b = 0; b < 5; ++b)
			if (!(v & map[b].mask))
				now[5 + b] = true;
	}

	/* Emit only changes */
	for (b = 0; b < NBTN; ++b) {
		u16 code = (b < 5) ? map[b].btn_p1 : map[b-5].btn_p2;
		if (now[b] != p->down[b]) {
			input_report_key(id, code, now[b]);
			p->down[b] = now[b];
		}
	}
	input_sync(id);
}

/* ───────── polling machinery ───────────────────────────────────── */
static DEFINE_RATELIMIT_STATE(rl, HZ, 10);

static void poll_work(struct work_struct *w)
{
	if (!ctx || atomic_read(&ctx->stop)) return;

	if (!get_report(ctx)) {
		if (__ratelimit(&rl))
			pr_debug("%*phN\n", 16, &ctx->buf[1]);
		push_events(ctx);
	}
}
static enum hrtimer_restart timer_cb(struct hrtimer *t)
{
	if (ctx && !atomic_read(&ctx->stop))
		queue_work(system_unbound_wq, &ctx->work);
	hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
	return atomic_read(&ctx->stop) ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

/* ───────── probe / remove ─────────────────────────────────────── */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r, b;

	if (ctx) return -EBUSY;

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) return -ENOMEM;
	p->hdev  = hdev;
	p->udev  = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev, p);

	if ((r = hid_parse(hdev)) || (r = hid_hw_start(hdev, HID_CONNECT_HIDRAW)))
		return r;
	set_idle(p);

	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err_hw; }

	p->idev->name       = "PIUIO 10-Btn Gamepad";
	p->idev->phys       = "piuio/js0";
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VID_PIUIO;
	p->idev->id.product = PID_PIUIO1020;

	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NBTN; ++b)
		set_bit((b < 5) ? map[b].btn_p1 : map[b-5].btn_p2,
			p->idev->keybit);

	if ((r = input_register_device(p->idev)))
		goto err_hw;

	ctx = p;
	INIT_WORK(&p->work, poll_work);
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = timer_cb;
	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	pr_info("game-pad ready – poll %d ms\n", clamp_poll());
	return 0;

err_hw:
	hid_hw_stop(hdev);
	return r;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);
	if (!p) return;

	atomic_set(&p->stop, 1);
	hrtimer_cancel(&p->timer);
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
	.name = "piuio",
	.id_table = ids,
	.probe  = probe,
	.remove = remove,
};
module_hid_driver(drv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020 – 10-button bit-level game-pad");