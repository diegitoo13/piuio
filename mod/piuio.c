// SPDX-License-Identifier: GPL-2.0
/*
 * PIUIO-1020 – 10-button game-pad + coin counter
 *
 * Generates Linux-input key events for both pads and pulses BTN_MODE
 * whenever the 16-bit coin counter changes.  The board requires one
 * HID SET_IDLE and a single 19-byte Output-report (ID 0x81 filled with
 * 0xFF) to enable the counter; the driver sends those at probe.
 *
 * I/O is performed entirely with control-pipe transfers (GET_REPORT)
 * on a timer-driven work-queue, matching the vendor BIOS.
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
#include <linux/byteorder/generic.h>
#include <linux/atomic.h>
#include <linux/slab.h>

/* ── constants ──────────────────────────────────────────────── */
#define VID_PIUIO     0x0d2f
#define PID_PIUIO1020 0x1020

#define RPT_ID_BTN    0x30
#define RPT_LEN_BTN   16

#define COIN_RID      0x01
#define COIN_LO       14
#define COIN_HI       15
#define COIN_BUF_LEN  258

#define WAKE_RID      0x81
#define WAKE_LEN      19

#define COIN_BTN_CODE BTN_MODE

/* ── button matrix map (active-low) ─────────────────────────── */
struct mapent { u8 mask; u16 p1, p2; };
static const struct mapent map[5] = {
	{ 0x01, BTN_SOUTH, BTN_TR     },
	{ 0x08, BTN_WEST , BTN_SELECT },
	{ 0x04, BTN_NORTH, BTN_START  },
	{ 0x02, BTN_EAST , BTN_THUMBL },
	{ 0x10, BTN_TL   , BTN_THUMBR },
};
#define NBTN 10

/* ── driver context ─────────────────────────────────────────── */
struct piuio {
	struct hid_device  *hdev;
	struct usb_device  *udev;
	int                 iface;

	u8                  btn_buf[RPT_LEN_BTN];
	bool                down[NBTN];

	u16                 coin_last;
	u8                  coin_buf[COIN_BUF_LEN];

	struct input_dev   *idev;
	struct work_struct  poll_wq;
	struct hrtimer      timer;
	atomic_t            stop;
	bool                hid_started;
};

static struct piuio *ctx;

/* ── module parameter ───────────────────────────────────────── */
static int poll_ms = 4;
module_param(poll_ms, int, 0444);
MODULE_PARM_DESC(poll_ms, "Polling interval in ms (1–1000, default 4)");

static inline int clamp_poll(void) { return clamp_val(poll_ms, 1, 1000); }

/* ── helpers ────────────────────────────────────────────────── */
static void send_wake(struct piuio *p)
{
	u8 *buf;
	int ret;

	if (!p || !p->udev)
		return;

	buf = kmalloc(WAKE_LEN, GFP_KERNEL);
	if (!buf)
		return;

	buf[0] = WAKE_RID;
	memset(buf + 1, 0xFF, WAKE_LEN - 1);

	ret = usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0),
			      HID_REQ_SET_REPORT,
			      USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			      (HID_OUTPUT_REPORT << 8) | WAKE_RID,
			      p->iface, buf, WAKE_LEN, 100);
	if (ret != WAKE_LEN)
		pr_warn("wake report send failed (%d)\n", ret);

	kfree(buf);
}

static int get_matrix(struct piuio *p)
{
	u16 w = (HID_INPUT_REPORT << 8) | RPT_ID_BTN;
	int r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				w, p->iface, p->btn_buf, RPT_LEN_BTN,
				USB_CTRL_GET_TIMEOUT);
	return (r == RPT_LEN_BTN) ? 0 : -EIO;
}

static int read_coin(struct piuio *p)
{
	u16 w = (HID_FEATURE_REPORT << 8) | COIN_RID;
	int r = usb_control_msg(p->udev, usb_rcvctrlpipe(p->udev, 0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				w, p->iface, p->coin_buf, COIN_BUF_LEN,
				USB_CTRL_GET_TIMEOUT);
	return (r >= COIN_HI + 1) ? 0 : -EIO;
}

/* ── input translation ─────────────────────────────────────── */
static void push_buttons(struct piuio *p)
{
	bool now[NBTN] = { false };
	int i, b;

	for (i = 0; i < 4; i++)
		for (b = 0; b < 5; b++)
			if (!(p->btn_buf[i] & map[b].mask))
				now[b] = true;

	for (i = 4; i < 8; i++)
		for (b = 0; b < 5; b++)
			if (!(p->btn_buf[i] & map[b].mask))
				now[5 + b] = true;

	for (b = 0; b < NBTN; b++)
		if (now[b] != p->down[b]) {
			input_report_key(p->idev,
				(b < 5) ? map[b].p1 : map[b - 5].p2, now[b]);
			p->down[b] = now[b];
		}
}

/* ── poll work ─────────────────────────────────────────────── */
static void poll_work(struct work_struct *w)
{
	static ktime_t last_coin;
	ktime_t now = ktime_get();
	u16 cnt;

	if (!ctx || atomic_read(&ctx->stop))
		return;

	if (!get_matrix(ctx))
		push_buttons(ctx);

	if (ktime_ms_delta(now, last_coin) >= 100 &&
	    !read_coin(ctx)) {
		cnt = le16_to_cpu(*(__le16 *)&ctx->coin_buf[COIN_LO]);
		if (cnt != ctx->coin_last) {
			input_report_key(ctx->idev, COIN_BTN_CODE, 1);
			input_report_key(ctx->idev, COIN_BTN_CODE, 0);
			ctx->coin_last = cnt;
		}
		last_coin = now;
	}

	input_sync(ctx->idev);
}

static enum hrtimer_restart timer_cb(struct hrtimer *t)
{
	if (ctx && !atomic_read(&ctx->stop))
		queue_work(system_unbound_wq, &ctx->poll_wq);

	hrtimer_forward_now(t, ms_to_ktime(clamp_poll()));
	return atomic_read(&ctx->stop) ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

/* ── probe / remove ─────────────────────────────────────────── */
static int probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *itf = to_usb_interface(hdev->dev.parent);
	struct piuio *p;
	int r, b;

	if (ctx)
		return -EBUSY;

	p = devm_kzalloc(&hdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->hdev  = hdev;
	p->udev  = interface_to_usbdev(itf);
	p->iface = itf->cur_altsetting->desc.bInterfaceNumber;
	hid_set_drvdata(hdev, p);

	if ((r = hid_parse(hdev)) ||
	    (r = hid_hw_start(hdev, HID_CONNECT_HIDRAW)))
		return r;
	p->hid_started = true;

	usb_control_msg(p->udev, usb_sndctrlpipe(p->udev, 0),
			HID_REQ_SET_IDLE,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0, p->iface, NULL, 0, 100);
	send_wake(p);

	p->idev = devm_input_allocate_device(&hdev->dev);
	if (!p->idev) { r = -ENOMEM; goto err_hw; }

	p->idev->name       = "PIUIO-1020 Gamepad";
	p->idev->phys       = hdev->phys;
	p->idev->id.bustype = BUS_USB;
	p->idev->id.vendor  = VID_PIUIO;
	p->idev->id.product = PID_PIUIO1020;

	set_bit(EV_KEY, p->idev->evbit);
	for (b = 0; b < NBTN; b++)
		set_bit((b < 5) ? map[b].p1 : map[b - 5].p2,
		        p->idev->keybit);
	set_bit(COIN_BTN_CODE, p->idev->keybit);

	if (input_register_device(p->idev)) { r = -EIO; goto err_hw; }

	if (read_coin(p))
		p->coin_last = 0;
	else
		p->coin_last = le16_to_cpu(*(__le16 *)&p->coin_buf[COIN_LO]);

	ctx = p;

	INIT_WORK(&p->poll_wq, poll_work);
	hrtimer_init(&p->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->timer.function = timer_cb;
	hrtimer_start(&p->timer, ms_to_ktime(clamp_poll()), HRTIMER_MODE_REL);

	pr_info("PIUIO-1020 attached (poll %d ms)\n", clamp_poll());
	return 0;

err_hw:
	hid_hw_stop(hdev);
	return r;
}

static void remove(struct hid_device *hdev)
{
	struct piuio *p = hid_get_drvdata(hdev);

	if (!p)
		return;

	atomic_set(&p->stop, 1);
	hrtimer_cancel(&p->timer);
	cancel_work_sync(&p->poll_wq);
	ctx = NULL;

	if (p->hid_started)
		hid_hw_stop(hdev);
}

/* ── module boiler-plate ─────────────────────────────────────── */
static const struct hid_device_id ids[] = {
	{ HID_USB_DEVICE(VID_PIUIO, PID_PIUIO1020) },
	{ }
};
MODULE_DEVICE_TABLE(hid, ids);

static struct hid_driver drv = {
	.name   = "piuio_gp",
	.id_table = ids,
	.probe  = probe,
	.remove = remove,
};
module_hid_driver(drv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO-1020 HID driver (buttons + coin counter)");