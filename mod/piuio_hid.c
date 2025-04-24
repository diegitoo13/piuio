// SPDX-License-Identifier: GPL-2.0
/*
 *  PIUIO HID interface driver – unified support for
 *  0x1010 (legacy – interrupt-IN) and 0x1020 (polled) Pump-It-Up button
 *  boards.
 *
 *  Dynamic-debug:
 *      # echo "module piuio_hid +p" > /sys/kernel/debug/dynamic_debug/control
 *
 *  Copyright (C) 2012-2014  Devin J. Pohly <djpohly+linux@gmail.com>
 *  Copyright (C) 2025       Diego Acevedo <diego.acevedo.fernando@gmail.com>
 */

#define pr_fmt(fmt)  "piuio_hid: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/byteorder/generic.h>
#include <linux/ratelimit.h>

/* ------------------------------------------------------------------ */
/*                        board-specific constants                    */
/* ------------------------------------------------------------------ */
#define USB_VENDOR_ID_BTNBOARD          0x0d2f
#define USB_PRODUCT_ID_BTNBOARD_LEGACY  0x1010
#define USB_PRODUCT_ID_BTNBOARD_NEW     0x1020

/* 48 physical inputs laid out as a 6-byte bit-matrix */
#define PIUIO_NUM_INPUTS        48
#define PIUIO_INPUT_BUF_SIZE    32    /* whole INPUT report (0x30) */

/* LED / output report (0x81) – 48 bits */
#define PIUIO_OUTPUT_REPORT_ID  0x81
#define PIUIO_OUTPUT_SIZE_NEW   7     /* id + 6 bytes */

/* legacy /dev/piuioX interface uses the same 6-byte bitmap */
#define PIUIO_LEGACY_SIZE       6
#define PIUIO_MAX_LEDS          48

/* ------------------------------------------------------------------ */
/*                       simple key-mapping                           */
/* ------------------------------------------------------------------ */
static const unsigned short piuio_keymap[PIUIO_NUM_INPUTS] = {
	/* row 0 */ KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K,
	/* row 1 */ KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA,
	/* row 2 */ KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_ESC,
	/* row 3 */ KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8,
	/* row 4 */ KEY_Q, KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I,
	/* row 5 */ KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8,
};
static inline int piuio_keycode(int idx) { return piuio_keymap[idx]; }

/* ------------------------------------------------------------------ */
/*                               types                                */
/* ------------------------------------------------------------------ */
struct piuio;
struct piuio_led {
	struct piuio          *parent;
	int                    idx;
	struct led_classdev    cdev;
};

struct piuio {
	/* core devices -------------------------------------------------- */
	struct hid_device     *hdev;
	struct usb_device     *udev;
	int                    iface;

	/* input --------------------------------------------------------- */
	struct input_dev      *idev;
	char                   phys[64];
	u8                     in_buf[PIUIO_INPUT_BUF_SIZE] ____cacheline_aligned;

	/* misc (/dev/piuioX) ------------------------------------------- */
	struct miscdevice      misc;
	char                  *misc_name;

	/* polling ------------------------------------------------------- */
	struct hrtimer         poll_timer;
	atomic_t               shutting_down;

	/* output / LEDs (0x1020 only) ---------------------------------- */
	spinlock_t             out_lock;
	atomic_t               out_active;
	u8                    *out_buf;
	struct urb            *out_urb;
	unsigned int           out_pipe;

#ifdef CONFIG_LEDS_CLASS
	spinlock_t             led_lock;
	bool                   led_shadow[PIUIO_MAX_LEDS];
	struct piuio_led      *leds;
#endif
};

/* ------------------------------------------------------------------ */
/*                     build-time / dynamic debug                     */
/* ------------------------------------------------------------------ */
static DEFINE_RATELIMIT_STATE(piu_rl, HZ, 10); /* 10 msgs / s */
#define DBG(fmt, ...)     pr_debug(fmt, ##__VA_ARGS__)
#define DBG_RL(fmt, ...)  do { if (__ratelimit(&piu_rl)) pr_debug(fmt, ##__VA_ARGS__); } while (0)

/* ------------------------------------------------------------------ */
/*                   module parameters / helpers                      */
/* ------------------------------------------------------------------ */
static int poll_interval_ms = 4; /* 250 Hz default */
module_param(poll_interval_ms, int, 0444);
MODULE_PARM_DESC(poll_interval_ms, "Polling interval in milliseconds (1–1000)");
static inline int clamped_poll_interval(void)
{
	return clamp_val(poll_interval_ms, 1, 1000);
}

static atomic_t piuio_device_count = ATOMIC_INIT(0);

/* ------------------------------------------------------------------ */
/*              global poll work (single active board)                */
/* ------------------------------------------------------------------ */
static struct work_struct piuio_poll_work;
static struct piuio      *piuio_poll_ctx; /* current board */

static void              piuio_do_poll(struct work_struct *w);
static enum hrtimer_restart piuio_timer_cb(struct hrtimer *);

#ifdef CONFIG_LEDS_CLASS
static int  piuio_led_set(struct led_classdev *, enum led_brightness);
static int  piuio_queue_output(struct piuio *);
#endif

/* ------------------------------------------------------------------ */
/*                       USB helpers (0x1020)                         */
/* ------------------------------------------------------------------ */
#define PIUIO_POLL_REPORT_ID   0x30   /* INPUT */
#define PIUIO_POLL_REPORT_SIZE 32

static int piuio_get_input_report(struct piuio *piu)
{
	u16 wValue = (HID_INPUT_REPORT << 8) | PIUIO_POLL_REPORT_ID;
	int ret = usb_control_msg(piu->udev, usb_rcvctrlpipe(piu->udev, 0),
				  HID_REQ_GET_REPORT,
				  USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				  wValue, piu->iface,
				  piu->in_buf, PIUIO_POLL_REPORT_SIZE,
				  msecs_to_jiffies(50));
	DBG_RL("GET_REPORT → %d\n", ret);
	if (ret < 0)
		return ret;
	return (ret == PIUIO_POLL_REPORT_SIZE) ? 0 : -EIO;
}

/* SET_IDLE(duration=0, id=0) -------------------------------------- */
static int piuio_set_idle(struct piuio *piu)
{
	int ret = usb_control_msg(piu->udev, usb_sndctrlpipe(piu->udev, 0),
				  HID_REQ_SET_IDLE,
				  USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				  0, piu->iface, NULL, 0,
				  msecs_to_jiffies(1)); /* 1 ms */
	udelay(500); /* extra 0.5 ms ≃ python helper */
	DBG("SET_IDLE → %d\n", ret);
	return ret;
}

/* ------------------------------------------------------------------ */
/*                     key / event handling                           */
/* ------------------------------------------------------------------ */
static void piuio_send_keys(struct piuio *piu)
{
	struct input_dev *idev = piu->idev;
	int i;
	for (i = 0; i < PIUIO_NUM_INPUTS; ++i) {
		int  code = piuio_keycode(i);
		bool down = piu->in_buf[(i >> 3) + 1] & BIT(i & 7);
		if (code > 0)
			input_report_key(idev, code, down);
	}
	input_sync(idev);
}

/* ------------------------------------------------------------------ */
/*                     poll timer & worker                            */
/* ------------------------------------------------------------------ */
static enum hrtimer_restart piuio_timer_cb(struct hrtimer *t)
{
	struct piuio *piu = container_of(t, struct piuio, poll_timer);
	if (!atomic_read(&piu->shutting_down) && piuio_poll_ctx == piu)
		queue_work(system_unbound_wq, &piuio_poll_work);
	hrtimer_forward_now(t, ms_to_ktime(clamped_poll_interval()));
	return atomic_read(&piu->shutting_down) ? HRTIMER_NORESTART : HRTIMER_RESTART;
}

static void piuio_do_poll(struct work_struct *w)
{
	struct piuio *piu = piuio_poll_ctx;
	if (!piu || atomic_read(&piu->shutting_down))
		return;
	if (!piuio_get_input_report(piu)) {
		DBG_RL("matrix %*phN\n", 6, &piu->in_buf[1]);
		piuio_send_keys(piu);
	}
}

/* ------------------------------------------------------------------ */
/*                  LED / output (0x1020 only)                        */
/* ------------------------------------------------------------------ */
#ifdef CONFIG_LEDS_CLASS
static void piuio_out_complete(struct urb *urb)
{
	struct piuio *piu = urb->context;
	atomic_set(&piu->out_active, 0);
	if (urb->status && urb->status != -ENOENT && urb->status != -ESHUTDOWN &&
	    urb->status != -ECONNRESET)
		hid_warn(piu->hdev, "output URB error %d\n", urb->status);
}

static int piuio_queue_output(struct piuio *piu)
{
	unsigned long flags;
	int i, ret;

	if (!piu->out_urb)
		return 0; /* legacy board */

	spin_lock_irqsave(&piu->out_lock, flags);
	if (atomic_read(&piu->out_active)) {
		spin_unlock_irqrestore(&piu->out_lock, flags);
		return 0;
	}
	atomic_set(&piu->out_active, 1);
	spin_unlock_irqrestore(&piu->out_lock, flags);

	memset(piu->out_buf, 0, PIUIO_OUTPUT_SIZE_NEW);
	piu->out_buf[0] = PIUIO_OUTPUT_REPORT_ID;

	spin_lock_irqsave(&piu->led_lock, flags);
	for (i = 0; i < PIUIO_MAX_LEDS; ++i)
		if (piu->led_shadow[i])
			piu->out_buf[1 + (i >> 3)] |= BIT(i & 7);
	spin_unlock_irqrestore(&piu->led_lock, flags);

	usb_fill_int_urb(piu->out_urb, piu->udev, piu->out_pipe,
			 piu->out_buf, PIUIO_OUTPUT_SIZE_NEW,
			 piuio_out_complete, piu, 1);
	ret = usb_submit_urb(piu->out_urb, GFP_ATOMIC);
	if (ret) {
		atomic_set(&piu->out_active, 0);
		hid_err(piu->hdev, "usb_submit_urb failed %d\n", ret);
	}
	return ret;
}

static int piuio_led_set(struct led_classdev *cdev, enum led_brightness br)
{
	struct piuio_led *ld  = container_of(cdev, struct piuio_led, cdev);
	struct piuio     *piu = ld->parent;
	unsigned long flags;

	spin_lock_irqsave(&piu->led_lock, flags);
	piu->led_shadow[ld->idx] = (br != 0);
	spin_unlock_irqrestore(&piu->led_lock, flags);
	return piuio_queue_output(piu);
}
#endif /* CONFIG_LEDS_CLASS */

/* ------------------------------------------------------------------ */
/*          misc (/dev/piuioX) – legacy LED userspace API             */
/* ------------------------------------------------------------------ */
static ssize_t piuio_misc_write(struct file *f, const char __user *ubuf,
				size_t len, loff_t *ppos)
{
	struct miscdevice *m  = f->private_data;
	struct piuio      *piu = dev_get_drvdata(m->this_device);
#ifdef CONFIG_LEDS_CLASS
	u8 tmp[PIUIO_LEGACY_SIZE];
	int i;
#endif

	if (!piu)
		return -ENODEV;
	if (len != PIUIO_LEGACY_SIZE)
		return -EINVAL;

#ifndef CONFIG_LEDS_CLASS
	return len;		/* LED support compiled out */
#else
	if (copy_from_user(tmp, ubuf, len))
		return -EFAULT;

	spin_lock(&piu->led_lock);
	for (i = 0; i < PIUIO_MAX_LEDS && i < PIUIO_LEGACY_SIZE * 8; ++i)
		piu->led_shadow[i] = !!(tmp[i >> 3] & BIT(i & 7));
	spin_unlock(&piu->led_lock);

	piuio_queue_output(piu);
	return len;
#endif
}

static const struct file_operations piuio_misc_fops = {
	.owner  = THIS_MODULE,
	.write  = piuio_misc_write,
	.open   = simple_open,
	.llseek = no_llseek,
};

/* ------------------------------------------------------------------ */
/*                            probe()                                 */
/* ------------------------------------------------------------------ */
static int piuio_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct piuio *piu;
	struct usb_endpoint_descriptor *ep_out = NULL;
	int i, ret;
#ifdef CONFIG_LEDS_CLASS
	int leds_ok = 0;
#endif

	DBG("probe for %04x:%04x\n", hdev->vendor, hdev->product);

	if (atomic_inc_return(&piuio_device_count) > 1) {
		atomic_dec(&piuio_device_count);
		return -EBUSY;
	}
	if (!intf) {
		ret = -ENODEV;
		goto err_dec;
	}

	/* -------------------------------------------------------------- */
	piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
	if (!piu) {
		ret = -ENOMEM;
		goto err_dec;
	}

	hid_set_drvdata(hdev, piu);
	piu->hdev  = hdev;
	piu->udev  = interface_to_usbdev(intf);
	piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;

	/* scan for interrupt-OUT (new board only) ---------------------- */
	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; ++i) {
		struct usb_endpoint_descriptor *ep =
			&intf->cur_altsetting->endpoint[i].desc;
		if (usb_endpoint_is_int_out(ep)) {
			ep_out = ep;
			break;
		}
	}

	/* start HID core ---------------------------------------------- */
	ret = hid_parse(hdev);
	if (ret)
		goto err_dec;
	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		goto err_dec;

	piuio_set_idle(piu);		/* best-effort */

	/* input-dev ----------------------------------------------------- */
	piu->idev = devm_input_allocate_device(&hdev->dev);
	if (!piu->idev) {
		ret = -ENOMEM;
		goto err_hw;
	}
	snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys ?: "piuio");
	piu->idev->name       = "PIUIO Panel";
	piu->idev->phys       = piu->phys;
	piu->idev->dev.parent = &hdev->dev;
	piu->idev->id.bustype = hdev->bus;
	piu->idev->id.vendor  = le16_to_cpu(hdev->vendor);
	piu->idev->id.product = le16_to_cpu(hdev->product);
	piu->idev->id.version = le16_to_cpu(hdev->version);

	set_bit(EV_KEY, piu->idev->evbit);
	for (i = 0; i < PIUIO_NUM_INPUTS; ++i) {
		int kc = piuio_keycode(i);
		if (kc > 0)
			set_bit(kc, piu->idev->keybit);
	}
	ret = input_register_device(piu->idev);
	if (ret)
		goto err_hw;

	/* -------------------------------------------------------------- */
	spin_lock_init(&piu->out_lock);
	atomic_set(&piu->out_active, 0);
	atomic_set(&piu->shutting_down, 0);

#ifdef CONFIG_LEDS_CLASS
	/* LED class-devs ---------------------------------------------- */
	spin_lock_init(&piu->led_lock);

	piu->leds = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS,
				 sizeof(*piu->leds), GFP_KERNEL);
	if (!piu->leds) {
		ret = -ENOMEM;
		goto err_leds;
	}
	for (i = 0; i < PIUIO_MAX_LEDS; ++i) {
		struct piuio_led *ld = &piu->leds[i];

		ld->parent    = piu;
		ld->idx       = i;
		ld->cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL,
					       "piuio:%s:led%02d",
					       dev_name(&hdev->dev), i);
		ld->cdev.max_brightness          = 1;
		ld->cdev.brightness_set_blocking = piuio_led_set;
		ret = led_classdev_register(&hdev->dev, &ld->cdev);
		if (ret)
			goto err_leds;
	}
	leds_ok = 1;

	/* allocate OUT resources if this is a 0x1020 ------------------ */
	if (id->product == USB_PRODUCT_ID_BTNBOARD_NEW && ep_out) {
		piu->out_buf = devm_kzalloc(&hdev->dev,
					    PIUIO_OUTPUT_SIZE_NEW, GFP_KERNEL);
		piu->out_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!piu->out_buf || !piu->out_urb) {
			ret = -ENOMEM;
			goto err_leds;
		}
		piu->out_pipe =
			usb_sndintpipe(piu->udev, ep_out->bEndpointAddress);
	}
#endif /* CONFIG_LEDS_CLASS */

	/* misc device -------------------------------------------------- */
	piu->misc_name = devm_kasprintf(&hdev->dev, GFP_KERNEL, "piuio%d",
				       atomic_read(&piuio_device_count) - 1);
	piu->misc.minor  = MISC_DYNAMIC_MINOR;
	piu->misc.name   = piu->misc_name;
	piu->misc.fops   = &piuio_misc_fops;
	piu->misc.parent = &hdev->dev;

	ret = misc_register(&piu->misc);
	if (ret)
		goto err_leds;
	dev_set_drvdata(piu->misc.this_device, piu);

	/* poll machinery ---------------------------------------------- */
	piuio_poll_ctx = piu;
	INIT_WORK(&piuio_poll_work, piuio_do_poll);

	hrtimer_init(&piu->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	piu->poll_timer.function = piuio_timer_cb;
	hrtimer_start(&piu->poll_timer,
		      ms_to_ktime(clamped_poll_interval()),
		      HRTIMER_MODE_REL);

	pr_info("attached %04x:%04x (poll %d ms)\n",
		hdev->vendor, hdev->product, clamped_poll_interval());
	return 0;

/* ------------ error paths ---------------------------------------- */
err_leds:
#ifdef CONFIG_LEDS_CLASS
	if (leds_ok)
		for (i = 0; i < PIUIO_MAX_LEDS; ++i)
			led_classdev_unregister(&piu->leds[i].cdev);
	if (piu->out_urb)
		usb_free_urb(piu->out_urb);
#endif
err_hw:
	hid_hw_stop(hdev);
err_dec:
	atomic_dec(&piuio_device_count);
	return ret;
}

/* ------------------------------------------------------------------ */
/*                              remove()                              */
/* ------------------------------------------------------------------ */
static void piuio_remove(struct hid_device *hdev)
{
	struct piuio *piu = hid_get_drvdata(hdev);
#ifdef CONFIG_LEDS_CLASS
	int i;
#endif
	if (!piu)
		return;

	atomic_set(&piu->shutting_down, 1);
	hrtimer_cancel(&piu->poll_timer);
	cancel_work_sync(&piuio_poll_work);
	piuio_poll_ctx = NULL;

#ifdef CONFIG_LEDS_CLASS
	for (i = 0; i < PIUIO_MAX_LEDS && piu->leds; ++i)
		led_classdev_unregister(&piu->leds[i].cdev);
	if (piu->out_urb) {
		usb_kill_urb(piu->out_urb);
		usb_free_urb(piu->out_urb);
	}
#endif
	misc_deregister(&piu->misc);
	input_unregister_device(piu->idev);
	hid_hw_stop(hdev);
	atomic_dec(&piuio_device_count);
	DBG("removed\n");
}

/* ------------------------------------------------------------------ */
/*                        driver definition                           */
/* ------------------------------------------------------------------ */
static const struct hid_device_id piuio_ids[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD,
			 USB_PRODUCT_ID_BTNBOARD_LEGACY) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD,
			 USB_PRODUCT_ID_BTNBOARD_NEW) },
	{ }
};
MODULE_DEVICE_TABLE(hid, piuio_ids);

static struct hid_driver piuio_driver = {
	.name     = "piuio_hid",
	.id_table = piuio_ids,
	.probe    = piuio_probe,
	.remove   = piuio_remove,
};
module_hid_driver(piuio_driver);

MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO HID driver – legacy 0x1010 + polled 0x1020 boards");
MODULE_LICENSE("GPL v2");