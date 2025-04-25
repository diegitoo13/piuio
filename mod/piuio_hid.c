// SPDX-License-Identifier: GPL-2.0
/*
 *  PIUIO 0x1020 button panel (polled HID) – **minimal** Linux driver
 *
 *  Copyright (C) 2025  Diego Acevedo <diego.acevedo.fernando@gmail.com>
 */

#define pr_fmt(fmt) "piuio_hid: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/byteorder/generic.h>
#include <linux/ratelimit.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

/* ------------------------------------------------------------------ */
/*                      device-specific constants                     */
/* ------------------------------------------------------------------ */
#define VENDOR_ID_PIUIO        0x0d2f
#define PROD_ID_PIUIO_1020     0x1020

#define REPORT_ID_INPUT        0x30
#define REPORT_LEN_INPUT       9       /* 1-byte ID + 8-byte matrix */

#define NUM_KEYS               48

/* simple key map – adjust to taste ----------------------------------------- */
static const unsigned short piu_keymap[NUM_KEYS] = {
	/* row0 */ KEY_Q, KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,
	/* row1 */ KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K,
	/* row2 */ KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA,
	/* row3 */ KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_ESC,
	/* row4 */ KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8,
	/* row5 */ KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8,
};
static inline unsigned short kc(int idx) { return piu_keymap[idx]; }

/* ------------------------------------------------------------------ */
struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	int                iface;

	struct input_dev  *idev;
	u8                 buf[REPORT_LEN_INPUT] ____cacheline_aligned;

	struct hrtimer     timer;
	struct work_struct poll_work;
	atomic_t           shutting_down;
};

/* ------------------------------------------------------------------ */
static DEFINE_RATELIMIT_STATE(piu_rl, HZ, 10);
#define DBG_RL(fmt, ...)   do { if (__ratelimit(&piu_rl)) \
				pr_debug(fmt, ##__VA_ARGS__); } while (0)

/* ------------------------------------------------------------------ */
static int poll_interval_ms = 4;     /* 250 Hz default */
module_param(poll_interval_ms, int, 0444);
MODULE_PARM_DESC(poll_interval_ms, "Polling interval (ms, 1–1000)");

static inline int poll_ms(void)
{
	return clamp_val(poll_interval_ms, 1, 1000);
}

/* ------------------------------------------------------------------ *\
|*                          USB helpers                              *|
\* ------------------------------------------------------------------ */
static int piu_get_matrix(struct piuio *p)
{
	u16 wValue = (HID_INPUT_REPORT << 8) | REPORT_ID_INPUT;
	int ret = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				  HID_REQ_GET_REPORT,
				  USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				  wValue, p->iface,
				  p->buf, REPORT_LEN_INPUT,
				  30);                       /* < 1 ms @ FS */
	if (ret < 0)
		return ret;
	return (ret == REPORT_LEN_INPUT) ? 0 : -EIO;
}

static int piu_set_idle(struct piuio *p)
{
	return usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0),
			       HID_REQ_SET_IDLE,
			       USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			       0, p->iface, NULL, 0, 10);
}

/* ------------------------------------------------------------------ *\
|*                     key-matrix → input layer                      *|
\* ------------------------------------------------------------------ */
static void piu_report_keys(struct piuio *p)
{
	struct input_dev *id = p->idev;
	int i;
	for (i = 0; i < NUM_KEYS; ++i) {
		bool pressed = p->buf[1 + (i >> 3)] & BIT(i & 7);
		input_report_key(id, kc(i), pressed);
	}
	input_sync(id);
}

/* ------------------------------------------------------------------ *\
|*                         poll machinery                            *|
\* ------------------------------------------------------------------ */
static struct piuio *poll_ctx;

static void piu_poll_work(struct work_struct *w)
{
	struct piuio *p = poll_ctx;

	if (!p || atomic_read(&p->shutting_down))
		return;

	if (!piu_get_matrix(p)) {
		DBG_RL("matrix %*phN\n", 8, &p->buf[1]);
		piu_report_keys(p);
	}
}

static enum hrtimer_restart piu_timer_cb(struct hrtimer *t)
{
	if (poll_ctx && !atomic_read(&poll_ctx->shutting_down))
		queue_work(system_unbound_wq, &poll_ctx->poll_work);

	hrtimer_forward_now(t, ms_to_ktime(poll_ms()));
	return atomic_read(&poll_ctx->shutting_down)
	       ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

/* ------------------------------------------------------------------ *\
|*                              probe                                *|
\* ------------------------------------------------------------------ */
static int piu_probe(struct hid_device *hdev,
		     const struct hid_device_id *id)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int i, ret;

	if (poll_ctx)        /* only one board supported */
		return -EBUSY;

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->hdev  = hdev;
	p->udev  = interface_to_usbdev(intf);
	p->iface = intf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev, p);

	/* HID core ---------------------------------------------------- */
	ret = hid_parse(hdev);
	if (ret)
		return ret;
	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		return ret;

	/* one-shot SET_IDLE                                             */
	piu_set_idle(p);

	/* input-dev ---------------------------------------------------- */
	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) {
		ret = -ENOMEM;
		goto err_hw;
	}
	p->idev->name       = "PIUIO 1020 Panel";
	p->idev->phys       = "piuio/input0";
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VENDOR_ID_PIUIO;
	p->idev->id.product = PROD_ID_PIUIO_1020;

	set_bit(EV_KEY, p->idev->evbit);
	for (i = 0; i < NUM_KEYS; ++i)
		set_bit(kc(i), p->idev->keybit);

	ret = input_register_device(p->idev);
	if (ret)
		goto err_hw;

	/* poll timer + work ------------------------------------------ */
	poll_ctx = p;
	INIT_WORK(&p->poll_work, piu_poll_work);
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = piu_timer_cb;
	hrtimer_start(&p->timer, ms_to_ktime(poll_ms()), HRTIMER_MODE_REL);

	pr_info("attached 0x1020 – polling %d ms\n", poll_ms());
	return 0;

err_hw:
	hid_hw_stop(hdev);
	return ret;
}

/* ------------------------------------------------------------------ */
static void piu_remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);

	if (!p)
		return;

	atomic_set(&p->shutting_down, 1);
	hrtimer_cancel(&p->timer);
	cancel_work_sync(&p->poll_work);
	poll_ctx = NULL;

	input_unregister_device(p->idev);
	hid_hw_stop(hdev);
}

/* ------------------------------------------------------------------ */
static const struct hid_device_id piu_table[] = {
	{ HID_USB_DEVICE(VENDOR_ID_PIUIO, PROD_ID_PIUIO_1020) },
	{ }
};
MODULE_DEVICE_TABLE(hid, piu_table);

static struct hid_driver piu_driver = {
	.name     = "piuio_hid",
	.id_table = piu_table,
	.probe    = piu_probe,
	.remove   = piu_remove,
};
module_hid_driver(piu_driver);

MODULE_AUTHOR("Diego Acevedo");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Minimal PIUIO 0x1020 polled HID driver (no LEDs)");
