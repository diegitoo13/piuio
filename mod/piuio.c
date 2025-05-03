// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 – 10-button game-pad + coin counter driver
 * Buttons via HID raw_event(), coin via delayed poll of byte 15
 * 2025-04-xx  Diego Acevedo
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/input-event-codes.h>
#include <linux/kernel.h>       /* clamp_val() */
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/string.h>

/* fallback if KEY_COIN isn’t defined */
#ifndef KEY_COIN
# define KEY_COIN KEY_KPENTER
#endif

/* USB IDs */
#define VID_PIUIO       0x0d2f
#define PID_PIUIO1020   0x1020

/* interrupt-in report for buttons */
#define RPT_ID_BTN      0x30
#define RPT_LEN_BTN     16

/* wake (output-report 0x81) */
#define WAKE_RID        0x81
#define WAKE_LEN        19

/* feature-report 0x01, we only care about byte 15 */
#define COIN_RID        0x01
#define COIN_HI         15
#define COIN_BUF_LEN    32

/* button bitmask → input code map */
struct mapent { u8 mask; u16 p1, p2; };
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH, BTN_TR     },
	{ 0x08, BTN_WEST,  BTN_SELECT },
	{ 0x04, BTN_NORTH, BTN_START  },
	{ 0x02, BTN_EAST,  BTN_THUMBL },
	{ 0x10, BTN_TL,    BTN_THUMBR },
};
#define NBTN 10

/* per-device data */
struct piuio {
	struct hid_device   *hdev;
	struct usb_device   *udev;
	int                  iface;

	bool                 down[NBTN];      /* current button state */

	u8                   coin_last;       /* last byte-15 seen */
	u8                   coin_buf[COIN_BUF_LEN];

	struct input_dev    *idev;
	struct delayed_work  coin_wq;
};
static struct piuio *ctx;

/* send the “wake” report (0x81 filled with 0xFF) */
static void wake_board(struct piuio *p)
{
	u8 buf[WAKE_LEN] = { [0] = WAKE_RID };
	memset(buf+1, 0xFF, WAKE_LEN-1);

	usb_control_msg(p->udev,
	                usb_sndctrlpipe(p->udev, 0),
	                HID_REQ_SET_REPORT,
	                USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	                (HID_OUTPUT_REPORT << 8) | WAKE_RID,
	                p->iface, buf, WAKE_LEN, 30);
}

/* read ONLY byte 15 of the feature report */
static int read_coin_hi(struct piuio *p, u8 *hi)
{
	int r = usb_control_msg(p->udev,
	                        usb_rcvctrlpipe(p->udev, 0),
	                        HID_REQ_GET_REPORT,
	                        USB_DIR_IN  | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
	                        (HID_FEATURE_REPORT << 8) | COIN_RID,
	                        p->iface, p->coin_buf, COIN_HI+1, 30);
	if (r < COIN_HI+1)
		return -EIO;
	*hi = p->coin_buf[COIN_HI];
	return 0;
}

/* every ~100 ms, poll coin counter byte 15 */
static void coin_work(struct work_struct *work)
{
	struct piuio *p = container_of(to_delayed_work(work),
	                               struct piuio, coin_wq);
	u8 hi;

	if (!read_coin_hi(p, &hi)) {
		if (hi != p->coin_last) {
			input_report_key(p->idev, KEY_COIN, 1);
			input_report_key(p->idev, KEY_COIN, 0);
			input_sync(p->idev);
			p->coin_last = hi;
		}
	}
	/* re-arm after 100 ms */
	queue_delayed_work(system_unbound_wq,
	                   &p->coin_wq,
	                   msecs_to_jiffies(100));
}

/* raw_event: called in interrupt context for report 0x30 */
static int raw_event(struct hid_device *hdev,
                     struct hid_report *report,
                     u8 *data, int size)
{
	struct piuio *p = hid_get_drvdata(hdev);
	bool now[NBTN] = { false };
	int i, b;

	if (!p || report->id != RPT_ID_BTN || size < RPT_LEN_BTN)
		return 0;  /* fall through to generic handling */

	/* P1 bytes 0–3 → now[0..4] */
	for (i = 0; i < 4; i++)
		for (b = 0; b < 5; b++)
			if (!(data[i] & map[b].mask))
				now[b] = true;

	/* P2 bytes 4–7 → now[5..9] */
	for (i = 4; i < 8; i++)
		for (b = 0; b < 5; b++)
			if (!(data[i] & map[b].mask))
				now[5 + b] = true;

	/* report diffs */
	for (b = 0; b < NBTN; b++) {
		if (now[b] != p->down[b]) {
			input_report_key(p->idev,
			                 (b < 5) ? map[b].p1 : map[b-5].p2,
			                 now[b]);
			p->down[b] = now[b];
		}
	}
	input_sync(p->idev);

	return 1;  /* consumed */
}

/* probe / initialization */
static int probe(struct hid_device *hdev,
                 const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r, b;

	if (ctx)
		return -EBUSY;  /* only one device at a time */

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->hdev  = hdev;
	p->udev  = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev, p);

	/* parse & start raw HID (no generic hid-input) */
	r = hid_parse(hdev);
	if (r)
		return r;
	r = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (r)
		return r;

	/* wake once for coin logic */
	wake_board(p);

	/* prime coin_last */
	if (read_coin_hi(p, &p->coin_last))
		p->coin_last = 0;

	/* allocate/register input device */
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) {
		r = -ENOMEM;
		goto err_stop;
	}
	p->idev->name       = "PIUIO Gamepad + Coin";
	p->idev->phys       = hdev->phys;
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VID_PIUIO;
	p->idev->id.product = PID_PIUIO1020;

	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NBTN; b++)
		set_bit((b < 5) ? map[b].p1 : map[b-5].p2,
		        p->idev->keybit);
	set_bit(KEY_COIN, p->idev->keybit);

	r = input_register_device(p->idev);
	if (r)
		goto err_stop;

	/* schedule first coin check after ~100 ms */
	INIT_DELAYED_WORK(&p->coin_wq, coin_work);
	queue_delayed_work(system_unbound_wq,
	                   &p->coin_wq,
	                   msecs_to_jiffies(100));

	ctx = p;
	pr_info("PIUIO attached (interrupt-driven)\n");
	return 0;

err_stop:
	hid_hw_stop(hdev);
	return r;
}

/* remove / cleanup */
static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);
	if (!p || p != ctx)
		return;

	cancel_delayed_work_sync(&p->coin_wq);
	input_unregister_device(p->idev);
	hid_hw_stop(p->hdev);
	ctx = NULL;
}

static const struct hid_device_id ids[] = {
	{ HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) },
	{ }
};
MODULE_DEVICE_TABLE(hid, ids);

static struct hid_driver drv = {
	.name       = "piuio",
	.id_table   = ids,
	.probe      = probe,
	.remove     = remove,
	.raw_event  = raw_event,
};
module_hid_driver(drv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020 – Interrupt-driven game-pad + coin counter");