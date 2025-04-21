#ifndef _PIUIO_HID_H
#define _PIUIO_HID_H

#include <linux/hid.h>          // Basic HID types
#ifdef CONFIG_LEDS_CLASS // Only include if LED support is configured
#include <linux/leds.h>         // struct led_classdev
#endif
#include <linux/spinlock_types.h>// spinlock_t
#include <linux/miscdevice.h>   // struct miscdevice
#include <linux/usb.h>          // struct usb_device, struct urb
#include <linux/input.h>        // struct input_dev
#include <linux/hrtimer.h>      // High-resolution timer
#include <linux/types.h>        // u8, etc.
#include <linux/mutex.h>        // struct mutex
#include <linux/workqueue.h>    // struct work_struct
#include <linux/atomic.h>       // atomic_t
#include <linux/device.h>       // Required for dev_get/set_drvdata prototypes
#include <linux/kconfig.h>      // For IS_ENABLED macro


/* Device and protocol definitions */
#define USB_VENDOR_ID_BTNBOARD          0x0d2f
#define USB_PRODUCT_ID_BTNBOARD_LEGACY  0x1010 // Original/Legacy device ID
#define USB_PRODUCT_ID_BTNBOARD_NEW     0x1020 // New device ID (verified)

// Report IDs/sizes
#define PIUIO_INPUT_REPORT_ID    0x01 // Report ID for input (from GET_REPORT wValue 0x0301)
#define PIUIO_RPT_OUT_BASE       0x80 // Base Report ID for legacy output control SET_REPORTs
#define PIUIO_OUTPUT_CHUNK       16   // Size of data chunk for legacy output SET_REPORTs
#define PIUIO_LEGACY_SIZE        8    // Expected size for legacy /dev/piuio0 writes
#define PIUIO_NUM_INPUTS         48   // Max number of inputs expected across supported devices
#ifdef CONFIG_LEDS_CLASS // Only define if LED support is configured
#define PIUIO_MAX_LEDS           48   // Max number of LEDs expected across supported devices
#endif

// Input report size for the NEW (1020) device (based on python/descriptor)
#define PIUIO_INPUT_SIZE_NEW     16
// Buffer size for GET_REPORT poll (needs space for report + potential overhead)
#define PIUIO_INPUT_BUF_SIZE     (PIUIO_INPUT_SIZE_NEW + 2)
// Output report size for the NEW (1020) device (based on descriptor/logs)
#define PIUIO_OUTPUT_SIZE_NEW    16

// Endpoint Addresses (from device descriptor analysis - informational)
#define PIUIO_INT_OUT_EP         0x02 // Interrupt OUT endpoint address for 1020

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

#ifdef CONFIG_LEDS_CLASS // Wrap LED specific structs/defs
// Forward declaration
struct piuio;

// LED Structure
struct piuio_led {
	struct piuio *piu;
	struct led_classdev cdev;
	u8 idx;
};
#endif // CONFIG_LEDS_CLASS

// Structure for legacy output work
struct piuio_legacy_output_work {
	struct work_struct work;
	struct piuio *piu;
	u8 report_id;
};

// Main Device Structure
struct piuio {
	struct hid_device    *hdev;
	struct usb_device    *udev;
	struct input_dev     *idev;
	u8                    iface;
	char                  phys[64]; // Physical path string

	/* Input Handling (Polling GET_REPORT) */
	u8                    in_buf[PIUIO_INPUT_BUF_SIZE]; // Buffer for polling input
	unsigned long         prev_inputs[BITS_TO_LONGS(PIUIO_NUM_INPUTS)]; // Previous input state
	struct hrtimer        timer;     // Timer for polling input

#ifdef CONFIG_LEDS_CLASS
	/* Output Handling (LEDs) */
	u8                    led_shadow[PIUIO_MAX_LEDS]; // Shadow state for LEDs
	struct piuio_led     *led;          // Array of LED structures (devm allocated)
	spinlock_t            led_lock;     // Protects led_shadow
#endif // CONFIG_LEDS_CLASS

	/* Output Handling (Interrupt OUT - 1020) */
	struct urb           *output_urb;   // URB for Interrupt OUT data (MANUAL alloc/free)
	u8                   *output_buf;   // Buffer for Interrupt OUT data (devm allocated)
	unsigned int          output_pipe;  // Pipe for Interrupt OUT
	spinlock_t            output_submit_lock; // Protects output URB submission state
	atomic_t              output_active;// Flag: is output URB currently submitted?

	/* Output Handling (Control Transfer - 1010 Legacy - via Workqueue) */
#ifdef CONFIG_LEDS_CLASS // Workqueue only needed for legacy LED output
	struct workqueue_struct *legacy_output_wq; // Workqueue for legacy output (manual alloc/destroy)
	struct mutex          legacy_output_mutex; // Protects legacy output work submission
#endif

	/* Misc Device */
	struct miscdevice     misc;         // Misc character device structure
	char                 *misc_name;    // Name for misc device (devm allocated)
};

#endif // _PIUIO_HID_H
