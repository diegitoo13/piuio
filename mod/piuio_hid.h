#ifndef _PIUIO_HID_H
#define _PIUIO_HID_H

#include <linux/hid.h>          // Basic HID types
#include <linux/leds.h>         // struct led_classdev
#include <linux/spinlock_types.h>// spinlock_t
#include <linux/miscdevice.h>   // struct miscdevice
#include <linux/usb.h>          // struct usb_device
#include <linux/input.h>        // struct input_dev

/*
 * Device and protocol definitions
 */
#define USB_VENDOR_ID_ANCHOR            0x0547
#define USB_PRODUCT_ID_PYTHON2          0x1002
#define USB_VENDOR_ID_BTNBOARD          0x0d2f
#define USB_PRODUCT_ID_BTNBOARD_LEGACY  0x1010 // Original/Legacy device ID
#define USB_PRODUCT_ID_BTNBOARD_NEW     0x1020 // New device ID

// Report IDs/sizes - Subject to verification per device type
#define PIUIO_RPT_OUT_BASE       0x80 // Base Report ID for legacy output control SET_REPORTs
#define PIUIO_OUTPUT_CHUNK       16   // Size of data chunk for legacy output SET_REPORTs
#define PIUIO_LEGACY_SIZE        8    // Expected size for legacy /dev/piuio0 writes
#define PIUIO_NUM_INPUTS         48   // Max number of inputs expected across supported devices
#define PIUIO_MAX_LEDS           48   // Max number of LEDs expected across supported devices

// Input report size for the NEW (1020) device based on its HID descriptor
#define PIUIO_INPUT_SIZE_NEW     16

// Input keycode mapping ranges
#define PIUIO_BTN_REG            BTN_JOYSTICK
#define PIUIO_BTN_EXTRA          BTN_TRIGGER_HAPPY

// HID Class Request Codes
#ifndef HID_REQ_SET_IDLE
#define HID_REQ_SET_IDLE                0x0A
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
    struct hid_device    *hdev;      // HID device structure
    struct usb_device    *udev;      // USB device structure
    struct input_dev     *idev;      // Input device structure
    u8                    iface;     // Interface number
    char                  phys[64];  // Physical path string
    unsigned long         prev_inputs[BITS_TO_LONGS(PIUIO_NUM_INPUTS)]; // Previous input state bitmap
    u8                    led_shadow[PIUIO_MAX_LEDS]; // Shadow state for LEDs
    struct piuio_led     *led;       // Array of LED structures
    struct miscdevice     misc;      // Legacy /dev/piuioX interface
    char                 *misc_name; // Allocated name for misc device
    spinlock_t            lock;      // Protects led_shadow
};

// Function Prototypes (implicitly static if only used in .c)
// No prototypes needed here if all functions are static and defined before use in piuio_hid.c

#endif // _PIUIO_HID_H