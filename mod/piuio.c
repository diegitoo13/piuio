// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad (16-byte interrupt report) + coin counter
 * Interrupt-driven version:
 *   · buttons handled in hid-core raw_event()  (max USB rate ≃1 kHz)
 *   · byte-15 coin counter polled every 100 ms with a delayed-work task
 *
 * 2025-04-xx  Diego Acevedo
 */

#define pr_fmt(fmt) "piuio_irq: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>          /* key-codes */
#include <linux/workqueue.h>
#include <linux/ratelimit.h>
#include <linux/string.h>

/* ───── KEY_COIN fallback ──────────────────────────────────────── */
#ifndef KEY_COIN               /* removed in older headers                   */
/* choose one unused KEY_ code that already exists on the current kernel:    */
/* KEY_KPENTER is rock-solid (0x160) – StepMania/ITGMania can remap easily.  */
#define KEY_COIN KEY_KPENTER
#endif

/* ───── device IDs ─────────────────────────────────────────────── */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

/* ───── report IDs / lengths ───────────────────────────────────── */
#define RPT_BTN_ID   0x30        /* interrupt-IN report, 16 B */
#define RPT_BTN_LEN  16

#define COIN_RID     0x01        /* feature report            */
#define COIN_OFST_HI 15          /* use byte 15 only          */
#define COIN_BUF_LEN 32

#define WAKE_RID     0x81        /* output report             */
#define WAKE_LEN     19

/* ───── button map (active-low) ────────────────────────────────── */
struct mapent { u8 mask; u16 p1; u16 p2; };
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH , BTN_TR      },   /* DL  */
	{ 0x08, BTN_WEST  , BTN_SELECT  },   /* DR  */
	{ 0x04, BTN_NORTH , BTN_START   },   /* UL  */
	{ 0x02, BTN_EAST  , BTN_THUMBL  },   /* UR  */
	{ 0x10, BTN_TL    , BTN_THUMBR  },   /* CTR */
};
#define NBTN 10

/* ───── private context ────────────────────────────────────────── */
struct piuio {
	struct hid_device  *hdev;
	struct usb_device  *udev;
	int                 iface;

	bool                down[NBTN];  /* current button state        */
	u8                  coin_last;   /* last byte-15 value           */

	struct input_dev   *idev;
	struct delayed_work coin_wq;     /* 100 ms coin poll             */
};
static struct piuio *ctx;            /* single board supported      */

/* ───── wake helper (one-shot) ─────────────────────────────────── */
static void wake_board(struct piuio *p)
{
	u8 buf[WAKE_LEN];

	buf[0] = WAKE_RID;
	memset(buf + 1, 0xFF, WAKE_LEN - 1);

	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0),
			HID_REQ_SET_REPORT,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			(HID_OUTPUT_REPORT << 8) | WAKE_RID,
			p->iface, buf, WAKE_LEN, 30);
}

/* ───── read only byte-15 of feature report 0x01 ───────────────── */
static int read_coin_hi(struct piuio *p, u8 *hi)
{
	u8 buf[COIN_BUF_LEN];
	int r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				(HID_FEATURE_REPORT << 8) | COIN_RID,
				p->iface, buf, COIN_OFST_HI + 1, 30);
	if (r < COIN_OFST_HI + 1)
		return -EIO;
	*hi = buf[COIN_OFST_HI];
	return 0;
}

/* ───── delayed work – poll coin every 100 ms ──────────────────── */
static void coin_work(struct work_struct *w)
{
	struct piuio *p = container_of(to_delayed_work(w),
	                               struct piuio, coin_wq);
	u8 hi;
	if (!read_coin_hi(p, &hi) && hi != p->coin_last) {
		input_report_key(p->idev, KEY_COIN, 1);
		input_report_key(p->idev, KEY_COIN, 0);
		p->coin_last = hi;
		input_sync(p->idev);
	}
	queue_delayed_work(system_unbound_wq, &p->coin_wq,
	                   msecs_to_jiffies(100));
}

/* ───── raw_event – buttons parsed at interrupt time ───────────── */
static int raw_event(struct hid_device *hdev, struct hid_report *rep,
		     u8 *data, int size)
{
	if (!ctx || rep->id != RPT_BTN_ID || size < RPT_BTN_LEN)
		return 0;                       /* let HID core continue */

	bool now[NBTN] = { false };
	int i, b;

	for (i = 0; i < 4; ++i)
		for (b = 0; b < 5; ++b)
			if (!(data[i] & map[b].mask))
				now[b] = true;

	for (i = 4; i < 8; ++i)
		for (b = 0; b < 5; ++b)
			if (!(data[i] & map[b].mask))
				now[5 + b] = true;

	for (b = 0; b < NBTN; ++b)
		if (now[b] != ctx->down[b]) {
			input_report_key(ctx->idev,
				(b < 5) ? map[b].p1 : map[b - 5].p2,
				now[b]);
			ctx->down[b] = now[b];
		}
	input_sync(ctx->idev);
	return 1;                               /* stop generic mapping */
}

/* ───── probe / remove ─────────────────────────────────────────── */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r, b;

	if (ctx)
		return -EBUSY;                  /* only one board */

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->hdev  = hdev;
	p->udev  = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev, p);

	if ((r = hid_parse(hdev)))
		return r;

	/* we need raw_event, not the generic hid-input parsing */
	if ((r = hid_hw_start(hdev, HID_CONNECT_HIDRAW)))
		return r;

	wake_board(p);
	read_coin_hi(p, &p->coin_last);   /* ignore error, just init */

	/* input device */
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err_stop; }

	p->idev->name       = "PIUIO Gamepad + Coin (IRQ)";
	p->idev->phys       = "piuio/irq0";
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VID_PIUIO;
	p->idev->id.product = PID_PIUIO1020;

	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NBTN; ++b)
		set_bit((b < 5) ? map[b].p1 : map[b - 5].p2,
		        p->idev->keybit);
	set_bit(KEY_COIN, p->idev->keybit);

	if ((r = input_register_device(p->idev)))
		goto err_stop;

	/* kick off coin polling */
	INIT_DELAYED_WORK(&p->coin_wq, coin_work);
	queue_delayed_work(system_unbound_wq, &p->coin_wq,
	                   msecs_to_jiffies(100));

	ctx = p;
	pr_info("attached (interrupt driven)\n");
	return 0;

err_stop:
	hid_hw_stop(hdev);
	return r;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);
	if (!p)
		return;
	cancel_delayed_work_sync(&p->coin_wq);
	input_unregister_device(p->idev);
	hid_hw_stop(hdev);
	ctx = NULL;
}

/* ───── boiler-plate ───────────────────────────────────────────── */
static const struct hid_device_id ids[] = {
	{ HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) },
	{}
};
MODULE_DEVICE_TABLE(hid, ids);

static struct hid_driver drv = {
	.name       = "piuio_irq",
	.id_table   = ids,
	.probe      = probe,
	.remove     = remove,
	.raw_event  = raw_event,
};
module_hid_driver(drv);

/* ───── meta ───────────────────────────────────────────────────── */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020 interrupt-driven game-pad + coin");