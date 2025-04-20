#ifndef _PIUIO_HID_H
#define _PIUIO_HID_H

#include <linux/hid.h>          // Basic HID types
#include <linux/leds.h>         // struct led_classdev
#include <linux/spinlock_types.h>// spinlock_t
#include <linux/miscdevice.h>   // struct miscdevice
#include <linux/usb.h>          // struct usb_device
#include <linux/input.h>        // struct input_dev
#include <linux/hrtimer.h>      // High-resolution timer
#include <linux/types.h>        // u8, etc.

/* Device and protocol definitions */
#define USB_VENDOR_ID_ANCHOR            0x0547
#define USB_PRODUCT_ID_PYTHON2          0x1002
#define USB_VENDOR_ID_BTNBOARD          0x0d2f
#define USB_PRODUCT_ID_BTNBOARD_LEGACY  0x1010 // Original/Legacy device ID
#define USB_PRODUCT_ID_BTNBOARD_NEW     0x1020 // New device ID (verified)

// Report IDs/sizes
#define PIUIO_INPUT_REPORT_ID    0x01 // Report ID for input (from GET_REPORT wValue 0x0301)
#define PIUIO_RPT_OUT_BASE       0x80 // Base Report ID for legacy output control SET_REPORTs
#define PIUIO_OUTPUT_CHUNK       16   // Size of data chunk for legacy output SET_REPORTs
#define PIUIO_LEGACY_SIZE        8    // Expected size for legacy /dev/piuio0 writes
#define PIUIO_NUM_INPUTS         48   // Max number of inputs expected across supported devices
#define PIUIO_MAX_LEDS           48   // Max number of LEDs expected across supported devices

// Input report size for the NEW (1020) device (based on python/descriptor)
#define PIUIO_INPUT_SIZE_NEW     16
#define PIUIO_INPUT_BUF_SIZE     (PIUIO_INPUT_SIZE_NEW + 2) // Buffer for GET_REPORT poll
// Assume legacy device might use a larger size (Original value, needs verification)
#define PIUIO_INPUT_SIZE_LEGACY  258

// Output report size for the NEW (1020) device based on descriptor and logs
#define PIUIO_OUTPUT_SIZE_NEW    16

// Input keycode mapping ranges
#define PIUIO_BTN_REG            BTN_JOYSTICK
#define PIUIO_BTN_EXTRA          BTN_TRIGGER_HAPPY

// HID Class Request Codes
#ifndef HID_REQ_SET_IDLE
#define HID_REQ_SET_IDLE                0x0A
#endif
#ifndef HID_REQ_GET_REPORT
#define HID_REQ_GET_REPORT              0x01
#endif
#ifndef HID_REQ_SET_REPORT
#define HID_REQ_SET_REPORT              0x09
#endif

// Forward declaration
struct piuio;

// LED Structure
struct piuio_led {
    struct piuio *piu;
    struct led_classdev cdev;
    u8 idx;
};

// Main Device Structure
struct piuio {
    struct hid_device    *hdev;
    struct usb_device    *udev;
    struct input_dev     *idev;
    u8                    iface;
    char                  phys[64];
    u8                    in_buf[PIUIO_INPUT_BUF_SIZE]; // Buffer for polling input
    unsigned long         prev_inputs[BITS_TO_LONGS(PIUIO_NUM_INPUTS)];
    u8                    led_shadow[PIUIO_MAX_LEDS];
    struct piuio_led     *led;
    struct hrtimer        timer;     // Timer for polling input
    struct miscdevice     misc;
    char                 *misc_name;
    spinlock_t            lock;      // Protects led_shadow
    u8                   *output_buf; // Buffer for interrupt OUT data
    struct urb           *output_urb; // URB for interrupt OUT data
    struct mutex          output_mutex; // Protect output_urb submission
};

#endif // _PIUIO_HID_H