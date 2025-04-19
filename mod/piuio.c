/*
 * PIUIO interface driver – unified legacy (vendor) + modern HID boards
 *
 * Copyright (C) 2012‑2014 Devin J. Pohly
 * Copyright (C) 2025      Diego Acevedo <diego.acevedo.fernando@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
*
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/bitops.h>
#include <linux/leds.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/usb/input.h>

/*──────────────────── Common constants ───────────────────*/
#define USB_VENDOR_ID_ANCHOR     0x0547
#define USB_PRODUCT_ID_PYTHON2   0x1002
#define USB_VENDOR_ID_BTNBOARD   0x0d2f
#define USB_PRODUCT_ID_BTNBOARD  0x1010 /* modern boards share this */

/* Vendor‑specific request (legacy) */
#define PIUIO_VND_REQ 0xae
#define PIUIO_VND_VAL 0
#define PIUIO_VND_IDX 0
#define PIUIO_VND_SZ  8

/* HID constants (modern) */
#define PIUIO_HID_SET_IDLE     0x0a
#define PIUIO_HID_GET_REPORT   0x01
#define PIUIO_HID_SET_REPORT   0x09
#define PIUIO_HID_RPT_INPUT    0x01      /* 258‑byte feature report */
#define PIUIO_HID_RPT_OUT_BASE 0x80      /* 0x80…0x83 – 4× output banks */
#define PIUIO_HID_INPUT_SIZE   258
#define PIUIO_HID_OUTPUT_CHUNK 16

/* Input keycode ranges */
#define PIUIO_BTN_REG      BTN_JOYSTICK
#define PIUIO_NUM_REG      (BTN_GAMEPAD - BTN_JOYSTICK)
#define PIUIO_BTN_EXTRA    BTN_TRIGGER_HAPPY
#define PIUIO_NUM_EXTRA    (KEY_MAX - BTN_TRIGGER_HAPPY)
#define PIUIO_NUM_BTNS     (PIUIO_NUM_REG + PIUIO_NUM_EXTRA)

/*──────────────────── Per‑board parameters ───────────────────*/
struct piuio_devtype {
    const char **led_names;
    int  inputs;
    int  outputs;
    int  mplex;      /* >1 on legacy boards */
    int  mplex_bits; /* mux bit width          */
};

/* LED name tables (48 full, 8 button‑board) */
static const char *led_names_full[] = {
    "piuio::output0","piuio::output1","piuio::output2","piuio::output3",
    "piuio::output4","piuio::output5","piuio::output6","piuio::output7",
    "piuio::output8","piuio::output9","piuio::output10","piuio::output11",
    "piuio::output12","piuio::output13","piuio::output14","piuio::output15",
    "piuio::output16","piuio::output17","piuio::output18","piuio::output19",
    "piuio::output20","piuio::output21","piuio::output22","piuio::output23",
    "piuio::output24","piuio::output25","piuio::output26","piuio::output27",
    "piuio::output28","piuio::output29","piuio::output30","piuio::output31",
    "piuio::output32","piuio::output33","piuio::output34","piuio::output35",
    "piuio::output36","piuio::output37","piuio::output38","piuio::output39",
    "piuio::output40","piuio::output41","piuio::output42","piuio::output43",
    "piuio::output44","piuio::output45","piuio::output46","piuio::output47"
};
static const char *led_names_bb[] = {
    "piuio::bboutput0","piuio::bboutput1","piuio::bboutput2","piuio::bboutput3",
    "piuio::bboutput4","piuio::bboutput5","piuio::bboutput6","piuio::bboutput7"
};

static struct piuio_devtype piuio_dev_full = {
    .led_names  = led_names_full,
    .inputs     = 48,
    .outputs    = 48,
    .mplex      = 4,
    .mplex_bits = 2,
};
static struct piuio_devtype piuio_dev_bb = {
    .led_names  = led_names_bb,
    .inputs     = 8,
    .outputs    = 8,
    .mplex      = 1,
    .mplex_bits = 0,
};
static struct piuio_devtype piuio_dev_hid = {
    .led_names  = led_names_full,
    .inputs     = 48,
    .outputs    = 48,
    .mplex      = 1,
    .mplex_bits = 0,
};

/*──────────────────── Main state ───────────────────*/
struct piuio_led; /* fwd */

enum piuio_proto { PROTO_VENDOR, PROTO_HID };

struct piuio {
    /* static */
    struct piuio_devtype *type;
    enum piuio_proto proto;

    /* Linux framework */
    struct usb_device *udev;
    struct input_dev *idev;
    char phys[64];

    /* I/O buffers */
    unsigned char  in_buf[PIUIO_HID_INPUT_SIZE];
    unsigned char  out_buf[PIUIO_HID_OUTPUT_CHUNK];
    unsigned char  led_shadow[48/8];

    /* URBs (legacy) */
    struct urb *urb_in;
    struct urb *urb_out;
    struct usb_ctrlrequest cr_in;
    struct usb_ctrlrequest cr_out;

    /* legacy multiplexer state */
    int set;

    /* input change tracking */
    unsigned long prev_inputs[2];

    /* LEDs */
    struct piuio_led *led;

    /* HID polling */
    struct timer_list poll_timer;
};

/*──────────────────── Helper macros ───────────────────*/
static inline int keycode(unsigned pin)
{
    if (pin < PIUIO_NUM_REG)
        return PIUIO_BTN_REG + pin;
    return PIUIO_BTN_EXTRA + (pin - PIUIO_NUM_REG);
}

/*──────────────────── HID helpers ───────────────────*/
static int piuio_hid_set_idle(struct usb_device *udev)
{
    return usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
                           PIUIO_HID_SET_IDLE,
                           USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                           0, 0, NULL, 0, 100);
}

static int piuio_hid_get_report(struct piuio *piu)
{
    return usb_control_msg(piu->udev, usb_rcvctrlpipe(piu->udev, 0),
                           PIUIO_HID_GET_REPORT,
                           USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                           (0x03 << 8) | PIUIO_HID_RPT_INPUT,
                           0,
                           piu->in_buf, PIUIO_HID_INPUT_SIZE, 100);
}

static int piuio_hid_set_report(struct piuio *piu, u8 rpt_id, const void *buf)
{
    return usb_control_msg(piu->udev, usb_sndctrlpipe(piu->udev, 0),
                           PIUIO_HID_SET_REPORT,
                           USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                           (0x03 << 8) | rpt_id, 0,
                           (void *)buf, PIUIO_HID_OUTPUT_CHUNK, 100);
}

/*──────────────────── LED backend ───────────────────*/
struct piuio_led { struct piuio *piu; struct led_classdev cdev; u8 index; };

static void piuio_led_set(struct led_classdev *cdev, enum led_brightness b)
{
    struct piuio_led *led = container_of(cdev, struct piuio_led, cdev);
    struct piuio *piu = led->piu;
    u8 idx = led->index;

    if (b)
        __set_bit(idx, (unsigned long *)piu->led_shadow);
    else
        __clear_bit(idx, (unsigned long *)piu->led_shadow);

    /* Legacy outputs are actually sent in the periodic OUT URB, so only do
     * immediate writeback for HID boards. */
    if (piu->proto != PROTO_HID)
        return;

    /* HID output reports are banked 16 bits/bytes each. */
    u8 bank  = idx / 16;
    u8 rptid = PIUIO_HID_RPT_OUT_BASE + bank;
    memcpy(piu->out_buf, &piu->led_shadow[bank * 2], PIUIO_HID_OUTPUT_CHUNK);
    piuio_hid_set_report(piu, rptid, piu->out_buf);
}

/*──────────────────── Input handling ───────────────────*/
static void piuio_handle_inputs(struct piuio *piu, const unsigned long *data)
{
    unsigned long diff[2];
    unsigned long diff_idx;

    diff[0] = data[0] ^ piu->prev_inputs[0];
    diff[1] = data[1] ^ piu->prev_inputs[1];
    piu->prev_inputs[0] = data[0];
    piu->prev_inputs[1] = data[1];

    for_each_set_bit(diff_idx, diff, piu->type->inputs) {
        bool pressed = !test_bit(diff_idx, data); /* active‑low */
        input_event(piu->idev, EV_MSC, MSC_SCAN, diff_idx + 1);
        input_report_key(piu->idev, keycode(diff_idx), pressed);
    }
    input_sync(piu->idev);
}

/*──────────────────── URB callbacks (legacy path) ───────────────────*/
static void piuio_vendor_in_cb(struct urb *urb)
{
    struct piuio *piu = urb->context;

    if (!urb->status)
        piuio_handle_inputs(piu, (unsigned long *)piu->in_buf);

    usb_submit_urb(urb, GFP_ATOMIC);
}

static void piuio_vendor_out_cb(struct urb *urb)
{
    struct piuio *piu = urb->context;

    /* rotate multiplex bank */
    piu->set = (piu->set + 1) % piu->type->mplex;
    piu->out_buf[0] &= ~((1 << piu->type->mplex_bits) - 1);
    piu->out_buf[2] &= ~((1 << piu->type->mplex_bits) - 1);
    piu->out_buf[0] |= piu->set;
    piu->out_buf[2] |= piu->set;

    usb_submit_urb(urb, GFP_ATOMIC);
}

/*──────────────────── HID polling timer ───────────────────*/
static void piuio_hid_timer(struct timer_list *t)
{
    struct piuio *piu = from_timer(piu, t, poll_timer);

    if (piuio_hid_get_report(piu) == PIUIO_HID_INPUT_SIZE)
        /* +2: skip report ID + reserved byte */
        piuio_handle_inputs(piu, (unsigned long *)(piu->in_buf + 2));

    mod_timer(&piu->poll_timer, jiffies + msecs_to_jiffies(4));
}

/*──────────────────── Probe / Disconnect ───────────────────*/
static int piuio_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(intf);
    struct piuio *piu;
    struct input_dev *idev;
    int i, err;

    piu = kzalloc(sizeof(*piu), GFP_KERNEL);
    if (!piu)
        return -ENOMEM;

    /*
     * Detect which protocol path to use.
     *  – HID interface class => modern board
     *  – Otherwise look at VID/PID (button‑board vs full)
     */
    if (intf->cur_altsetting->desc.bInterfaceClass == USB_CLASS_HID) {
        piu->proto = PROTO_HID;
        piu->type  = &piuio_dev_hid;
    } else if (id->idVendor == USB_VENDOR_ID_BTNBOARD &&
               id->idProduct == USB_PRODUCT_ID_BTNBOARD) {
        piu->proto = PROTO_VENDOR; /* small button‑board */
        piu->type  = &piuio_dev_bb;
    } else {
        piu->proto = PROTO_VENDOR; /* full legacy */
        piu->type  = &piuio_dev_full;
    }

    piu->udev = udev;
    usb_make_path(udev, piu->phys, sizeof(piu->phys));
    strlcat(piu->phys, "/input0", sizeof(piu->phys));

    /*──────────────── input device ────────────────*/
    idev = input_allocate_device();
    if (!idev) { err = -ENOMEM; goto err_free_piu; }
    piu->idev = idev;

    idev->name       = "PIUIO input";
    idev->phys       = piu->phys;
    usb_to_input_id(udev, &idev->id);
    idev->dev.parent = &intf->dev;

    set_bit(EV_KEY, idev->evbit);
    set_bit(EV_MSC, idev->evbit);
    set_bit(MSC_SCAN, idev->mscbit);

    for (i = 0; i < piu->type->inputs; i++)
        set_bit(keycode(i), idev->keybit);

    input_set_drvdata(idev, piu);

    /*──────────────── LED registration ─────────────*/
    piu->led = kcalloc(piu->type->outputs, sizeof(*piu->led), GFP_KERNEL);
    if (!piu->led) { err = -ENOMEM; goto err_free_idev; }

    for (i = 0; i < piu->type->outputs; i++) {
        piu->led[i].piu            = piu;
        piu->led[i].index          = i;
        piu->led[i].cdev.name      = piu->type->led_names[i];
        piu->led[i].cdev.brightness_set = piuio_led_set;
        err = led_classdev_register(&intf->dev, &piu->led[i].cdev);
        if (err) goto err_unregister_leds;
    }

    /*──────────────── Protocol‑specific setup ─────*/
    if (piu->proto == PROTO_HID) {
        /* One‑shot idle handshake then start polling timer */
        piuio_hid_set_idle(udev);
        timer_setup(&piu->poll_timer, piuio_hid_timer, 0);
        mod_timer(&piu->poll_timer, jiffies + msecs_to_jiffies(4));
    } else {
        /* Legacy URBs */
        piu->urb_in  = usb_alloc_urb(0, GFP_KERNEL);
        piu->urb_out = usb_alloc_urb(0, GFP_KERNEL);
        if (!piu->urb_in || !piu->urb_out) { err = -ENOMEM; goto err_unregister_leds; }

        /* OUT URB (mux + outputs) */
        piu->cr_out.bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
        piu->cr_out.bRequest     = PIUIO_VND_REQ;
        piu->cr_out.wValue       = cpu_to_le16(PIUIO_VND_VAL);
        piu->cr_out.wIndex       = cpu_to_le16(PIUIO_VND_IDX);
        piu->cr_out.wLength      = cpu_to_le16(PIUIO_VND_SZ);
        usb_fill_control_urb(piu->urb_out, udev, usb_sndctrlpipe(udev, 0),
                             (void *)&piu->cr_out, piu->out_buf, PIUIO_VND_SZ,
                             piuio_vendor_out_cb, piu);

        /* IN URB (inputs) */
        piu->cr_in.bRequestType = USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
        piu->cr_in.bRequest     = PIUIO_VND_REQ;
        piu->cr_in.wValue       = cpu_to_le16(PIUIO_VND_VAL);
        piu->cr_in.wIndex       = cpu_to_le16(PIUIO_VND_IDX);
        piu->cr_in.wLength      = cpu_to_le16(PIUIO_VND_SZ);
        usb_fill_control_urb(piu->urb_in, udev, usb_rcvctrlpipe(udev, 0),
                             (void *)&piu->cr_in, piu->in_buf, PIUIO_VND_SZ,
                             piuio_vendor_in_cb, piu);

        usb_submit_urb(piu->urb_out, GFP_KERNEL);
        usb_submit_urb(piu->urb_in,  GFP_KERNEL);
    }

    /* register input last so userspace sees fully initialised device */
    err = input_register_device(idev);
    if (err) goto err_stop_proto;

    usb_set_intfdata(intf, piu);
    return 0;

err_stop_proto:
    if (piu->proto == PROTO_HID)
        del_timer_sync(&piu->poll_timer);
    else {
        if (piu->urb_in)  usb_kill_urb(piu->urb_in);
        if (piu->urb_out) usb_kill_urb(piu->urb_out);
    }
err_unregister_leds:
    while (--i >= 0)
        led_classdev_unregister(&piu->led[i].cdev);
    kfree(piu->led);
err_free_idev:
    input_free_device(idev);
err_free_piu:
    kfree(piu);
    return err;
}

static void piuio_disconnect(struct usb_interface *intf)
{
    struct piuio *piu = usb_get_intfdata(intf);
    int i;

    if (!piu) return;

    if (piu->proto == PROTO_HID)
        del_timer_sync(&piu->poll_timer);
    else {
        usb_kill_urb(piu->urb_in);
        usb_kill_urb(piu->urb_out);
        usb_free_urb(piu->urb_in);
        usb_free_urb(piu->urb_out);
    }

    for (i = 0; i < piu->type->outputs; i++)
        led_classdev_unregister(&piu->led[i].cdev);

    input_unregister_device(piu->idev);
    kfree(piu->led);
    kfree(piu);
}

/*────────── USB id table ─────────*/
static const struct usb_device_id piuio_ids[] = {
    { USB_DEVICE(USB_VENDOR_ID_ANCHOR,  USB_PRODUCT_ID_PYTHON2) },
    { USB_DEVICE(USB_VENDOR_ID_BTNBOARD, USB_PRODUCT_ID_BTNBOARD) },
    { }
};
MODULE_DEVICE_TABLE(usb, piuio_ids);

static struct usb_driver piuio_driver = {
    .name       = "piuio_unified",
    .probe      = piuio_probe,
    .disconnect = piuio_disconnect,
    .id_table   = piuio_ids,
};
module_usb_driver(piuio_driver);

MODULE_AUTHOR("Diego Acevedo");
MODULE_DESCRIPTION("Unified PIUIO I/O driver");
MODULE_LICENSE("GPL v2");
