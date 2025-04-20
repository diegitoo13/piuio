/*
 * PIUIO HID interface driver – overrides generic Linux HID by matching PIUIO boards
 * and provides legacy /dev/piuio0 char-device compatibility for existing apps.
 *
 * Copyright (C) 2012‑2014 Devin J. Pohly
 * Copyright (C) 2025      Diego Acevedo <diego.acevedo.fernando@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/hid.h>

#define USB_VENDOR_ID_ANCHOR     0x0547
#define USB_PRODUCT_ID_PYTHON2   0x1002
#define USB_VENDOR_ID_BTNBOARD   0x0d2f
#define USB_PRODUCT_ID_BTNBOARD  0x1010

#define PIUIO_RPT_INPUT_ID       0x01
#define PIUIO_RPT_OUT_BASE       0x80
#define PIUIO_INPUT_SIZE         258
#define PIUIO_OUTPUT_CHUNK       16
#define PIUIO_LEGACY_SIZE        8
#define PIUIO_NUM_INPUTS         48

#define PIUIO_BTN_REG            BTN_JOYSTICK
#define PIUIO_BTN_EXTRA          BTN_TRIGGER_HAPPY

static int poll_interval_ms = 4;
module_param(poll_interval_ms, int, 0444);
MODULE_PARM_DESC(poll_interval_ms, "Polling interval in milliseconds");

struct piuio;
struct piuio_led {
    struct piuio *piu;
    struct led_classdev cdev;
    u8 idx;
};

struct piuio {
    struct hid_device    *hdev;
    struct usb_device    *udev;
    struct input_dev     *idev;
    u8                    iface;
    char                  phys[64];
    unsigned char         in_buf[PIUIO_INPUT_SIZE];
    unsigned long         prev[PIUIO_NUM_INPUTS/sizeof(unsigned long)];
    u8                    led_shadow[PIUIO_NUM_INPUTS];
    struct piuio_led     *led;
    struct hrtimer        timer;
    struct miscdevice     misc;
    spinlock_t            lock;
};

static inline int piuio_keycode(unsigned pin)
{
    if (pin < (BTN_GAMEPAD - BTN_JOYSTICK))
        return PIUIO_BTN_REG + pin;
    return PIUIO_BTN_EXTRA + (pin - (BTN_GAMEPAD - BTN_JOYSTICK));
}

static int piuio_get_report(struct piuio *piu)
{
    return usb_control_msg(
        piu->udev,
        usb_rcvctrlpipe(piu->udev, 0),
        HID_REQ_GET_REPORT,
        USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN,
        (HID_INPUT_REPORT << 8) | PIUIO_RPT_INPUT_ID,
        piu->iface,
        piu->in_buf, PIUIO_INPUT_SIZE,
        HZ);
}

static int piuio_set_report(struct piuio *piu, u8 rptid)
{
    int bank = rptid - PIUIO_RPT_OUT_BASE;
    u8 buf[PIUIO_OUTPUT_CHUNK + 1];
    int i;

    buf[0] = rptid;
    spin_lock(&piu->lock);
    for (i = 0; i < PIUIO_OUTPUT_CHUNK; i++) {
        int idx = bank * PIUIO_OUTPUT_CHUNK + i;
        buf[i+1] = (idx < PIUIO_NUM_INPUTS && piu->led_shadow[idx]) ? 1 : 0;
    }
    spin_unlock(&piu->lock);

    return usb_control_msg(
        piu->udev,
        usb_sndctrlpipe(piu->udev, 0),
        HID_REQ_SET_REPORT,
        USB_TYPE_CLASS | USB_RECIP_INTERFACE,
        (HID_OUTPUT_REPORT << 8) | rptid,
        piu->iface,
        buf, PIUIO_OUTPUT_CHUNK + 1,
        HZ);
}

static enum hrtimer_restart piuio_timer_cb(struct hrtimer *timer)
{
    struct piuio *piu = container_of(timer, struct piuio, timer);
    unsigned long changed[PIUIO_NUM_INPUTS/sizeof(unsigned long)];
    unsigned long *data;
    int i;

    if (piuio_get_report(piu) == PIUIO_INPUT_SIZE) {
        data = (unsigned long *)(piu->in_buf + 2);
        for (i = 0; i < PIUIO_NUM_INPUTS/sizeof(unsigned long); i++) {
            changed[i] = data[i] ^ piu->prev[i];
            piu->prev[i] = data[i];
        }
        for (i = 0; i < PIUIO_NUM_INPUTS; i++) {
            if (test_bit(i, changed)) {
                bool pressed = !test_bit(i, data);
                input_event(piu->idev, EV_MSC, MSC_SCAN, i+1);
                input_report_key(piu->idev, piuio_keycode(i), pressed);
            }
        }
        input_sync(piu->idev);
    }
    hrtimer_forward_now(&piu->timer, ms_to_ktime(poll_interval_ms));
    return HRTIMER_RESTART;
}

static void piuio_led_set(struct led_classdev *cdev,
                          enum led_brightness b)
{
    struct piuio_led *ld = container_of(cdev, struct piuio_led, cdev);
    struct piuio *piu = ld->piu;

    spin_lock(&piu->lock);
    piu->led_shadow[ld->idx] = (b > 0);
    spin_unlock(&piu->lock);

    piuio_set_report(piu, PIUIO_RPT_OUT_BASE + (ld->idx / PIUIO_OUTPUT_CHUNK));
}

static ssize_t piuio_dev_write(struct file *filp,
                               const char __user *buf,
                               size_t len, loff_t *off)
{
    struct piuio *piu = filp->private_data;
    u8 tmp[PIUIO_LEGACY_SIZE];
    int pin;

    if (len != PIUIO_LEGACY_SIZE)
        return -EINVAL;
    if (copy_from_user(tmp, buf, len))
        return -EFAULT;

    spin_lock(&piu->lock);
    memset(piu->led_shadow, 0, PIUIO_NUM_INPUTS);
    for (pin = 0; pin < PIUIO_LEGACY_SIZE*8 && pin < PIUIO_NUM_INPUTS; pin++)
        piu->led_shadow[pin] = !!(tmp[pin/8] & (1 << (pin%8)));
    spin_unlock(&piu->lock);

    for (pin = 0; pin < PIUIO_NUM_INPUTS; pin++)
        piuio_set_report(piu, PIUIO_RPT_OUT_BASE + (pin / PIUIO_OUTPUT_CHUNK));

    return len;
}

static const struct file_operations piuio_fops = {
    .owner   = THIS_MODULE,
    .write   = piuio_dev_write,
    .open    = simple_open,
    .llseek  = no_llseek,
};

static int piuio_probe(struct hid_device *hdev,
                       const struct hid_device_id *id)
{
    struct piuio *piu;
    struct input_dev *idev;
    struct usb_interface *intf;
    int i, ret;

    intf = to_usb_interface(hdev->dev.parent);
    piu = devm_kzalloc(&hdev->dev, sizeof(*piu), GFP_KERNEL);
    if (!piu)
        return -ENOMEM;
    hid_set_drvdata(hdev, piu);
    piu->hdev = hdev;
    piu->udev = interface_to_usbdev(intf);
    piu->iface = intf->cur_altsetting->desc.bInterfaceNumber;
    spin_lock_init(&piu->lock);

    snprintf(piu->phys, sizeof(piu->phys), "%s/input0", dev_name(&hdev->dev));
    idev = devm_input_allocate_device(&hdev->dev);
    if (!idev)
        return -ENOMEM;
    piu->idev = idev;
    idev->name = "PIUIO HID";
    idev->phys = piu->phys;
    idev->dev.parent = &hdev->dev;
    set_bit(EV_KEY, idev->evbit);
    for (i = 0; i < PIUIO_NUM_INPUTS; i++)
        set_bit(piuio_keycode(i), idev->keybit);
    set_bit(EV_MSC, idev->evbit);
    set_bit(MSC_SCAN, idev->mscbit);
    ret = input_register_device(idev);
    if (ret)
        return ret;

    piu->led = devm_kcalloc(&hdev->dev, PIUIO_NUM_INPUTS,
                             sizeof(*piu->led), GFP_KERNEL);
    if (!piu->led) {
        input_unregister_device(idev);
        return -ENOMEM;
    }
    for (i = 0; i < PIUIO_NUM_INPUTS; i++) {
        piu->led[i].piu = piu;
        piu->led[i].idx = i;
        piu->led[i].cdev.name = kasprintf(GFP_KERNEL, "piuio::output%u", i);
        piu->led[i].cdev.brightness_set = piuio_led_set;
        ret = led_classdev_register(&hdev->dev, &piu->led[i].cdev);
        if (ret)
            goto err_led;
    }

    hrtimer_init(&piu->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    piu->timer.function = piuio_timer_cb;
    hrtimer_start(&piu->timer, ms_to_ktime(poll_interval_ms), HRTIMER_MODE_REL);

    piu->misc.minor  = MISC_DYNAMIC_MINOR;
    piu->misc.name   = "piuio0";
    piu->misc.fops   = &piuio_fops;
    piu->misc.parent = &hdev->dev;
    ret = misc_register(&piu->misc);
    if (ret)
        goto err_misc;

    ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
    if (ret)
        hid_err(hdev, "hid_hw_start failed: %d", ret);
    return ret;

err_misc:
    hrtimer_cancel(&piu->timer);
err_led:
    while (--i >= 0)
        led_classdev_unregister(&piu->led[i].cdev);
    input_unregister_device(idev);
    return ret;
}

static void piuio_remove(struct hid_device *hdev)
{
    struct piuio *piu = hid_get_drvdata(hdev);

    misc_deregister(&piu->misc);
    hrtimer_cancel(&piu->timer);
    for (int i = 0; i < PIUIO_NUM_INPUTS; i++)
        led_classdev_unregister(&piu->led[i].cdev);
    input_unregister_device(piu->idev);
    hid_hw_stop(hdev);
}

static const struct hid_device_id piuio_ids[] = {
    { HID_USB_DEVICE(USB_VENDOR_ID_ANCHOR, USB_PRODUCT_ID_PYTHON2) },
    { HID_USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD) },
    {}
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
MODULE_DESCRIPTION("PIUIO HID interface with legacy compatibility");
MODULE_LICENSE("GPL v2");
