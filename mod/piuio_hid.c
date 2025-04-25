// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO 0x1020 button panel — 10-button **game-pad** driver
 * 2025  Diego Acevedo <diego.acevedo.fernando@gmail.com>
 */

#define pr_fmt(fmt) "piuio_gp: " fmt

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/ratelimit.h>

/* ───────── device IDs & report layout ─────────────────────────── */
#define VID_PIUIO      0x0d2f
#define PID_PIUIO1020  0x1020
#define REP_ID         0x30
#define REP_LEN        9          /* 1-byte ID + 8 data                 */

/* ───────── byte-value → game-pad button map (10 entries) ───────── */
struct mapent { u8 val; u8 byte; u16 btn; };

/* Player 1 : bytes 0-3 (index 0-3) */
static const struct mapent map[10] = {
	/* value, byte-idx, evdev code                  */
	{ 0xFE, 0, BTN_SOUTH  },   /* Down-Left  */
	{ 0xFB, 1, BTN_WEST   },   /* Up-Left    */
	{ 0xEF, 2, BTN_NORTH  },   /* Centre     */
	{ 0xFD, 3, BTN_EAST   },   /* Up-Right   */
	{ 0xF7, 0, BTN_TL     },   /* Down-Right */

	/* Player 2 : bytes 4-7 (index 4-7) */
	{ 0xFE, 4, BTN_TR        },
	{ 0xFB, 5, BTN_SELECT    },
	{ 0xEF, 6, BTN_START     },
	{ 0xFD, 7, BTN_THUMBL    },
	{ 0xF7, 4, BTN_THUMBR    },
};

#define NUM_BTNS 10

/* ───────── driver private ─────────────────────────────────────── */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int                iface;

	struct input_dev  *idev;
	u8                 buf[REP_LEN];
	bool               down[NUM_BTNS];

	struct hrtimer     tim;
	struct work_struct poll_wq;
	atomic_t           stop;
};
static struct piuio *ctx;

/* ───────── module param – poll rate (ms) ───────────────────────── */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval in ms (1-1000)");

static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* ───────── USB helpers ─────────────────────────────────────────── */
static int get_matrix(struct piuio *p)
{
	u16 wValue = (HID_INPUT_REPORT << 8) | REP_ID;
	int ret = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				  HID_REQ_GET_REPORT,
				  USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
				  wValue, p->iface,
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

/* ───────── push game-pad events ────────────────────────────────── */
static void report_buttons(struct piuio *p)
{
	struct input_dev *id = p->idev;
	int i;
	for (i = 0; i < NUM_BTNS; ++i) {
		const struct mapent *m = &map[i];
		bool pressed = (p->buf[1 + m->byte] == m->val);
		if (pressed != p->down[i]) {
			input_report_key(id, m->btn, pressed);
			p->down[i] = pressed;
		}
	}
	input_sync(id);
}

/* ───────── poll machinery ─────────────────────────────────────── */
static DEFINE_RATELIMIT_STATE(rl, HZ, 10);

static void poll_work(struct work_struct *w)
{
	if (!ctx || atomic_read(&ctx->stop))
		return;
	if (!get_matrix(ctx)) {
		if (__ratelimit(&rl))
			pr_debug("%*phN\n", 8, &ctx->buf[1]);
		report_buttons(ctx);
	}
}
static enum hrtimer_restart t_cb(struct hrtimer *t)
{
	if (ctx && !atomic_read(&ctx->stop))
		queue_work(system_unbound_wq, &ctx->poll_wq);
	hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
	return atomic_read(&ctx->stop) ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

/* ───────── probe / remove ─────────────────────────────────────── */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int ret, i;

	if (ctx) return -EBUSY;       /* one board max */

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p) return -ENOMEM;
	p->hdev = hdev;
	p->udev = interface_to_usbdev(intf);
	p->iface = intf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev, p);

	if ((ret = hid_parse(hdev)) ||
	    (ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW)))
		return ret;
	set_idle(p);

	/* input dev --------------------------------------------------- */
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { ret = -ENOMEM; goto err_hw; }

	p->idev->name       = "PIUIO 10-Button Pad";
	p->idev->phys       = "piuio/js0";
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VID_PIUIO;
	p->idev->id.product = PID_PIUIO1020;

	set_bit(EV_KEY, p->idev->evbit);
	for (i = 0; i < NUM_BTNS; ++i)
		set_bit(map[i].btn, p->idev->keybit);
	if ((ret = input_register_device(p->idev)))
		goto err_hw;

	/* poll machinery --------------------------------------------- */
	ctx = p;
	INIT_WORK(&p->poll_wq, poll_work);
	hrtimer_init(&p->tim, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->tim.function = t_cb;
	hrtimer_start(&p->tim, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);
	pr_info("game-pad mode active (poll %d ms)\n", clamp_poll());
	return 0;

err_hw:
	hid_hw_stop(hdev);
	return ret;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);
	if (!p) return;

	atomic_set(&p->stop, 1);
	hrtimer_cancel(&p->tim);
	cancel_work_sync(&p->poll_wq);
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
	.name     = "piuio_hid",
	.id_table = ids,
	.probe    = probe,
	.remove   = remove,
};
module_hid_driver(drv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO 0x1020 – 10-button game-pad driver");