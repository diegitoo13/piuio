/*
 * PIUIO HID interface driver
 * (Based on analysis of 0x1020 device traffic)
 * Overrides generic Linux HID by matching PIUIO boards and provides
 * legacy /dev/piuio0 char-device compatibility for existing apps.
 *
 * Supports legacy (0x1010) and newer (0x1020) product IDs.
 * Input is handled via timer polling using GET_REPORT control transfers.
 * Input format for 1020 requires 32 bytes (Report ID 0x30).
 * Output for 1020 uses Interrupt OUT endpoint (16 bytes, Report ID 0x13).
 * Legacy (1010) device support is minimal/unverified.
 * LED support is conditional on CONFIG_LEDS_CLASS.
 *
 * Legacy implementation basis:
 * Copyright (C) 2012-2014 Devin J. Pohly (djpohly+linux@gmail.com)
 *
 * HID integration, 0x1020 support, and modifications:
 * Copyright (C) 2025      Diego Acevedo <diego.acevedo.fernando@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#ifdef CONFIG_LEDS_CLASS
#include <linux/leds.h>
#endif
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/kconfig.h>
#include <linux/poll.h>

#include "piuio_hid.h"

/* ------------------------------------------------------------------------- */
/*                    tunable & parameter validation                         */
/* ------------------------------------------------------------------------- */

static int poll_interval_ms = 4;
module_param(poll_interval_ms, int, 0444);
MODULE_PARM_DESC(poll_interval_ms,
                 "Polling interval in milliseconds for input reports (1‑1000)");

static inline int clamped_poll_interval(void)
{
	/* sane bounds, 1 ms … 1 s */
	return clamp_val(poll_interval_ms, 1, 1000);
}

/* Prevent multiple device instances (btn board is unique in a cab) */
static atomic_t piuio_device_count = ATOMIC_INIT(0);

/* ------------------------------------------------------------------------- */
/*                     helpers & bit / key mapping                           */
/* ------------------------------------------------------------------------- */

static inline int piuio_keycode(unsigned int pin)
{
	if (pin >= PIUIO_NUM_INPUTS)
		return -EINVAL;

	if (pin < (BTN_GAMEPAD - BTN_JOYSTICK))
		return PIUIO_BTN_REG + pin;

	return PIUIO_BTN_EXTRA + (pin - (BTN_GAMEPAD - BTN_JOYSTICK));
}

/* ------------------------------------------------------------------------- */
/*                        forward declarations                               */
/* ------------------------------------------------------------------------- */
struct piuio;
static enum hrtimer_restart piuio_timer_cb(struct hrtimer *);
#ifdef CONFIG_LEDS_CLASS
static int piuio_send_output_interrupt(struct piuio *);
#endif

/* ------------------------------------------------------------------------- */
/*                          core structures                                  */
/* ------------------------------------------------------------------------- */

struct piuio {
	struct hid_device *hdev;
	struct usb_device *udev;
	struct input_dev  *idev;
	u8                 iface;
	char               phys[64];

	/* input polling */
	u8                 in_buf[PIUIO_INPUT_BUF_SIZE];
	struct hrtimer     poll_timer;
	atomic_t           shutting_down; /* set once remove() runs */

#ifdef CONFIG_LEDS_CLASS
	/* LED support */
	u8                 led_shadow[PIUIO_MAX_LEDS];
	struct piuio_led  *leds;
	spinlock_t         led_lock;       /* guards led_shadow */
#endif

	/* interrupt‑out for outputs */
	struct urb        *out_urb;
	u8                *out_buf;        /* devm allocated */
	unsigned int       out_pipe;
	spinlock_t         out_lock;       /* guards out_urb submission */
	atomic_t           out_active;

	/* misc */
	struct miscdevice  misc;
	char              *misc_name;      /* devm allocated */
};

#ifdef CONFIG_LEDS_CLASS
struct piuio_led {
	struct piuio        *parent;
	struct led_classdev  cdev;
	u8                   idx;
};
#endif

/* ------------------------------------------------------------------------- */
/*                           USB helpers                                     */
/* ------------------------------------------------------------------------- */

static int piuio_get_input_report(struct piuio *piu)
{
	const u8 report_id    = PIUIO_INPUT_REPORT_ID;
	const u16 report_len  = PIUIO_INPUT_SIZE_NEW;
	const u16 wValue      = (HID_INPUT_REPORT << 8) | report_id;
	int ret;

	ret = usb_control_msg(piu->udev, usb_rcvctrlpipe(piu->udev, 0),
			        HID_REQ_GET_REPORT,
			        USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			        wValue, piu->iface,
			        piu->in_buf, report_len,
			        msecs_to_jiffies(50));

	if (ret < 0)
		return ret; /* log later */

	if (ret != report_len)
		return -EIO;

	return 0;
}

static void piuio_push_input(struct piuio *piu)
{
	int i;
	struct input_dev *idev = piu->idev;

	for (i = 0; i < PIUIO_NUM_INPUTS; ++i) {
		int code = piuio_keycode(i);
		bool pressed = piu->in_buf[(i >> 3) + 1] & BIT(i & 7);
		input_report_key(idev, code, pressed);
	}
	input_sync(idev);
}

/* ------------------------------------------------------------------------- */
/*                      polling timer callback                               */
/* ------------------------------------------------------------------------- */

static enum hrtimer_restart piuio_timer_cb(struct hrtimer *timer)
{
	struct piuio *piu = container_of(timer, struct piuio, poll_timer);
	int err;

	if (atomic_read(&piu->shutting_down))
		return HRTIMER_NORESTART;

	err = piuio_get_input_report(piu);
	if (!err)
		piuio_push_input(piu);

	/* silently ignore errors except fatal */
	if (err == -ENODEV || err == -ESHUTDOWN)
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, ms_to_ktime(clamped_poll_interval()));
	return HRTIMER_RESTART;
}

/* ------------------------------------------------------------------------- */
/*                    LED handling (conditional)                             */
/* ------------------------------------------------------------------------- */
#ifdef CONFIG_LEDS_CLASS
static void piuio_out_complete(struct urb *urb)
{
	struct piuio *piu = urb->context;

	atomic_set(&piu->out_active, 0);

	if (urb->status &&
	    urb->status != -ENOENT && urb->status != -ESHUTDOWN && urb->status != -ECONNRESET)
		hid_warn(piu->hdev, "output URB error %d", urb->status);
}

static int piuio_queue_output(struct piuio *piu)
{
	unsigned long flags;
	int i, ret;

	spin_lock_irqsave(&piu->out_lock, flags);
	if (atomic_read(&piu->out_active)) {
		spin_unlock_irqrestore(&piu->out_lock, flags);
		return 0; /* already pending */
	}
	atomic_set(&piu->out_active, 1);
	spin_unlock_irqrestore(&piu->out_lock, flags);

	memset(piu->out_buf, 0, PIUIO_OUTPUT_SIZE_NEW);
	piu->out_buf[0] = PIUIO_OUTPUT_REPORT_ID;

	spin_lock_irqsave(&piu->led_lock, flags);
	for (i = 0; i < PIUIO_MAX_LEDS; ++i) {
		if (piu->led_shadow[i])
			piu->out_buf[1 + (i >> 3)] |= BIT(i & 7);
	}
	spin_unlock_irqrestore(&piu->led_lock, flags);

	usb_fill_int_urb(piu->out_urb, piu->udev, piu->out_pipe,
			piu->out_buf, PIUIO_OUTPUT_SIZE_NEW,
			piuio_out_complete, piu, 1);

	ret = usb_submit_urb(piu->out_urb, GFP_ATOMIC);
	if (ret) {
		atomic_set(&piu->out_active, 0);
		hid_err(piu->hdev, "usb_submit_urb failed %d", ret);
	}
	return ret;
}

static int piuio_led_set(struct led_classdev *cdev,
			 enum led_brightness value)
{
	struct piuio_led *ld = container_of(cdev, struct piuio_led, cdev);
	struct piuio *piu   = ld->parent;
	unsigned long flags;

	spin_lock_irqsave(&piu->led_lock, flags);
	piu->led_shadow[ld->idx] = !!value;
	spin_unlock_irqrestore(&piu->led_lock, flags);

	return piuio_queue_output(piu);
}
#endif /* CONFIG_LEDS_CLASS */

/* ------------------------------------------------------------------------- */
/*                     misc‑device legacy write                              */
/* ------------------------------------------------------------------------- */

static ssize_t piuio_misc_write(struct file *file, const char __user *buf,
			      size_t len, loff_t *ppos)
{
	struct miscdevice *m = file->private_data;
	struct piuio *piu    = dev_get_drvdata(m->this_device);
#ifdef CONFIG_LEDS_CLASS
	u8 tmp[PIUIO_LEGACY_SIZE];
	int i;
#endif

	if (!piu)
		return -ENODEV;

	if (len != PIUIO_LEGACY_SIZE)
		return -EINVAL;

#ifndef CONFIG_LEDS_CLASS
	return len; /* no outputs compiled in */
#else
	if (copy_from_user(tmp, buf, len))
		return -EFAULT;

	for (i = 0; i < PIUIO_MAX_LEDS && i < PIUIO_LEGACY_SIZE * 8; ++i)
		piu->led_shadow[i] = !!(tmp[i >> 3] & BIT(i & 7));

	if (piuio_queue_output(piu))
		return -EIO;

	return len;
#endif
}

static const struct file_operations piuio_misc_fops = {
	.owner = THIS_MODULE,
	.write = piuio_misc_write,
	.open  = simple_open,
	.llseek = no_llseek,
};

/* ------------------------------------------------------------------------- */
/*                              probe                                       */
/* ------------------------------------------------------------------------- */
static int piuio_probe(struct hid_device *hdev,
			const struct hid_device_id *id)
{
	struct piuio *piu;
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *ep_out = NULL;
	int i, ret;
#ifdef CONFIG_LEDS_CLASS
	int led_reg_ok = 0;
#endif

	if (atomic_inc_return(&piuio_device_count) > 1) {
		atomic_dec(&piuio_device_count);
		return -EBUSY;
	}

	intf = to_usb_interface(hdev->dev.parent);
	if (!intf) {
		ret = -ENODEV;
		goto err_count;
	}

	piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
	if (!piu) {
		ret = -ENOMEM;
		goto err_count;
	}

	hid_set_drvdata(hdev, piu);
	piu->hdev = hdev;
	piu->udev = interface_to_usbdev(intf);
	piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;

	atomic_set(&piu->shutting_down, 0);
	spin_lock_init(&piu->out_lock);
	atomic_set(&piu->out_active, 0);
#ifdef CONFIG_LEDS_CLASS
	spin_lock_init(&piu->led_lock);
#endif

	/* look for interrupt‑out endpoint */
	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; ++i) {
		struct usb_endpoint_descriptor *ep = &intf->cur_altsetting->endpoint[i].desc;
		if (usb_endpoint_is_int_out(ep)) {
			ep_out = ep;
			break;
		}
	}

	/* ---------------- input device via HID core ---------------- */
	ret = hid_parse(hdev);
	if (ret)
		goto err_count;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		goto err_count;

	/* allocate plain input device to push decoded buttons */
	piu->idev = devm_input_allocate_device(&hdev->dev);
	if (!piu->idev) {
		ret = -ENOMEM;
		goto err_stop_hw;
	}
	
	snprintf(piu->phys, sizeof(piu->phys), "%s/input0", hdev->phys ?: "piuio");
	piu->idev->name = "PIUIO Panel";
	piu->idev->phys = piu->phys;
	piu->idev->id   = hdev->id;
	piu->idev->dev.parent = &hdev->dev;

	set_bit(EV_KEY, piu->idev->evbit);
	for (i = 0; i < PIUIO_NUM_INPUTS; ++i) {
		int kc = piuio_keycode(i);
		if (kc > 0)
			set_bit(kc, piu->idev->keybit);
	}

	ret = input_register_device(piu->idev);
	if (ret)
		goto err_stop_hw;

	/* ---------------- LED / output set‑up ---------------- */
#ifdef CONFIG_LEDS_CLASS
	piu->leds = devm_kcalloc(&hdev->dev, PIUIO_MAX_LEDS,
				   sizeof(*piu->leds), GFP_KERNEL);
	if (!piu->leds) {
		ret = -ENOMEM;
		goto err_unreg_input;
	}

	for (i = 0; i < PIUIO_MAX_LEDS; ++i) {
		struct piuio_led *ld = &piu->leds[i];
		ld->parent = piu;
		ld->idx    = i;
		ld->cdev.name = devm_kasprintf(&hdev->dev, GFP_KERNEL,
					   "piuio:%s:led%02d", dev_name(&hdev->dev), i);
		ld->cdev.max_brightness          = 1;
		ld->cdev.brightness_set_blocking = piuio_led_set;
		ret = led_classdev_register(&hdev->dev, &ld->cdev);
		if (ret)
			goto err_leds;
	}
	led_reg_ok = 1;

	/* allocate interrupt‑out resources if endpoint present */
	if (ep_out) {
		piu->out_buf  = devm_kzalloc(&hdev->dev, PIUIO_OUTPUT_SIZE_NEW, GFP_KERNEL);
		piu->out_urb  = usb_alloc_urb(0, GFP_KERNEL);
		if (!piu->out_buf || !piu->out_urb) {
			ret = -ENOMEM;
			goto err_leds;
		}
		piu->out_pipe = usb_sndintpipe(piu->udev, ep_out->bEndpointAddress);
	}
#endif /* CONFIG_LEDS_CLASS */

	/* ---------------- miscdevice ---------------- */
	piu->misc_name = devm_kasprintf(&hdev->dev, GFP_KERNEL,
				    "piuio%d", piuio_device_count.counter);
	piu->misc.minor = MISC_DYNAMIC_MINOR;
	piu->misc.name  = piu->misc_name;
	piu->misc.fops  = &piuio_misc_fops;
	piu->misc.parent = &hdev->dev;

	ret = misc_register(&piu->misc);
	if (ret)
		goto err_leds;
	dev_set_drvdata(piu->misc.this_device, piu);

	/* ---------------- start polling timer ---------------- */
	hrtimer_init(&piu->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	piu->poll_timer.function = piuio_timer_cb;
	hrtimer_start(&piu->poll_timer, ms_to_ktime(clamped_poll_interval()),
		       HRTIMER_MODE_REL);

	return 0;

/* ---------------- error paths ---------------- */
err_leds:
#ifdef CONFIG_LEDS_CLASS
	if (led_reg_ok)
		for (i = 0; i < PIUIO_MAX_LEDS; ++i)
			led_classdev_unregister(&piu->leds[i].cdev);
	if (piu->out_urb)
		usb_free_urb(piu->out_urb);
#endif
err_unreg_input:
	input_unregister_device(piu->idev);
err_stop_hw:
	hid_hw_stop(hdev);
err_count:
	atomic_dec(&piuio_device_count);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                             remove                                        */
/* ------------------------------------------------------------------------- */
static void piuio_remove(struct hid_device *hdev)
{
	struct piuio *piu = hid_get_drvdata(hdev);
	int i;

	if (!piu)
		return;

	atomic_set(&piu->shutting_down, 1);
	hrtimer_cancel(&piu->poll_timer);

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
}

/* ------------------------------------------------------------------------- */
/*                         hid_driver struct                                 */
/* ------------------------------------------------------------------------- */
static const struct hid_device_id piuio_ids[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_LEGACY) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD_NEW)    },
	{ }
};
MODULE_DEVICE_TABLE(hid, piuio_ids);

static struct hid_driver piuio_driver = {
	.name    = "piuio_hid",
	.id_table = piuio_ids,
	.probe   = piuio_probe,
	.remove  = piuio_remove,
};
module_hid_driver(piuio_driver);

MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("PIUIO HID interface driver (hardened)");
MODULE_LICENSE("GPL v2");
