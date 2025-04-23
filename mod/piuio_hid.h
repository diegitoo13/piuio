/* SPDX-License-Identifier: GPL-2.0 */
/* Public interface (driver-internal only) for the PIUIO HID driver */

#ifndef _PIUIO_HID_H
#define _PIUIO_HID_H

#include <linux/hid.h>
#include <linux/usb.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/atomic.h>
#include <linux/spinlock_types.h>
#ifdef CONFIG_LEDS_CLASS
#include <linux/leds.h>
#endif

/* ------------------------------------------------------------------ */
/*                     device-/protocol-level constants               */
/* ------------------------------------------------------------------ */
#define USB_VENDOR_ID_BTNBOARD          0x0d2f
#define USB_PRODUCT_ID_BTNBOARD_LEGACY  0x1010 /* original board  */
#define USB_PRODUCT_ID_BTNBOARD_NEW     0x1020 /* board in traces */

#define PIUIO_INPUT_REPORT_ID           0x30   /* GET_REPORT 0x0130 */
#define PIUIO_OUTPUT_REPORT_ID          0x13   /* Interrupt-OUT     */

#define PIUIO_INPUT_SIZE_NEW            32     /* incl. report ID   */
#define PIUIO_OUTPUT_SIZE_NEW           16
#define PIUIO_INPUT_BUF_SIZE            PIUIO_INPUT_SIZE_NEW

#define PIUIO_LEGACY_SIZE               8      /* write() payload   */

#define PIUIO_NUM_INPUTS                48
#ifdef CONFIG_LEDS_CLASS
#define PIUIO_MAX_LEDS                  48
#endif

#define PIUIO_BTN_REG                   BTN_JOYSTICK
#define PIUIO_BTN_EXTRA                 BTN_TRIGGER_HAPPY

/* ------------------------------------------------------------------ */
/*                           forward declarations                     */
/* ------------------------------------------------------------------ */
struct piuio;
#ifdef CONFIG_LEDS_CLASS
struct piuio_led {
	struct piuio        *parent;
	struct led_classdev  cdev;
	u8                   idx;
};
#endif

/* ------------------------------------------------------------------ */
/*                          main device structure                     */
/* ------------------------------------------------------------------ */
struct piuio {
	/* generic / identification */
	struct hid_device   *hdev;
	struct usb_device   *udev;
	struct input_dev    *idev;
	u8                   iface;
	char                 phys[64];

	/* ---- input polling (GET_REPORT) ---- */
	u8                   in_buf[PIUIO_INPUT_BUF_SIZE];
	struct hrtimer       poll_timer;
	atomic_t             shutting_down;

#ifdef CONFIG_LEDS_CLASS
	/* ---- LED / output bookkeeping ---- */
	u8                   led_shadow[PIUIO_MAX_LEDS];
	struct piuio_led    *leds;
	spinlock_t           led_lock;
#endif

	/* ---- interrupt-OUT resources (0x1020 only) ---- */
	struct urb          *out_urb;
	u8                  *out_buf;
	unsigned int         out_pipe;
	spinlock_t           out_lock;
	atomic_t             out_active;

	/* ---- legacy misc char-dev ---- */
	struct miscdevice    misc;
	char                *misc_name;
};

/* helper exported for sparse-forward reference in .c */
static inline int piuio_keycode(unsigned int pin)
{
	if (pin >= PIUIO_NUM_INPUTS)
		return -EINVAL;

	if (pin < (BTN_GAMEPAD - BTN_JOYSTICK))
		return PIUIO_BTN_REG + pin;

	return PIUIO_BTN_EXTRA + (pin - (BTN_GAMEPAD - BTN_JOYSTICK));
}

#endif /* _PIUIO_HID_H */