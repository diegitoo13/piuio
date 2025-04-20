# PIUIO HID Driver for Linux

This is a Linux kernel module driver for Andamiro PIUIO arcade I/O boards, interfacing via the standard HID subsystem.

## Features

* Provides input events for panels/buttons via the Linux input subsystem (e.g., `/dev/input/eventX`).
* Provides LED control via the Linux LED subsystem (`/sys/class/leds/`).
* Provides a legacy `/dev/piuioX` character device interface for compatibility with some older applications.
* Supports Product IDs `0x1010` (legacy) and `0x1020` (newer).

## Current Status (Work In Progress)

* **Device Matching:** Matches both `0d2f:1010` and `0d2f:1020` devices.
* **Initialization (`0x1020`):** Successfully sends the required `SET_IDLE` command.
* **Input Handling (`0x1020`):** Uses standard HID `raw_event` callback. Parses the known 16-byte input report size. **Note:** The exact bit-to-button mapping within the 16 bytes is currently assumed and may need refinement based on testing or the device's HID Report Descriptor.
* **Input Handling (`0x1010`):** The input report mechanism and format for the legacy device are currently **unknown**. Input events for this device are **ignored** until its protocol is determined.
* **Output/LED Handling:** Uses legacy `SET_REPORT` control transfers. This may be incorrect for the `0x1020` device and needs verification/potential update to use its defined 16-byte Output report (likely via the interrupt OUT endpoint).

## Compiling and Installing

1.  Ensure you have the kernel headers installed for your running kernel (e.g., `sudo apt install linux-headers-$(uname -r)` on Debian/Ubuntu).
2.  Navigate to the `mod` directory containing `piuio_hid.c`, `piuio_hid.h`, and `Makefile`.
3.  Run `make` to compile `piuio_hid.ko`.
4.  Run `sudo make install` to copy the module to `/lib/modules/$(uname -r)/updates/` and update module dependencies. (Requires root privileges).
5.  Load the module: `sudo modprobe piuio_hid`
6.  Unload the module: `sudo rmmod piuio_hid`

## Testing

* **Input:** Use tools like `evtest` (install if needed) on the relevant `/dev/input/eventX` device created when the driver binds.
* **Output (LEDs):** Control LEDs via `/sys/class/leds/`. Find the LED names associated with your device (e.g., `/sys/class/leds/piuio-0003:0D2F:1020.XXXX::outputY`) and echo 0 or 1 to the `brightness` file (e.g., `echo 1 | sudo tee /sys/class/leds/DEVICE_NAME::output3/brightness`). *Note: LED control might not work correctly for the 1020 device yet.*
* **Legacy Device:** Test applications using `/dev/piuio0` (or similar).

## Contributing / Further Work

* Determine the exact input report format for the `0x1020` device.
* Determine the input report mechanism (interrupt/control?) and format for the legacy `0x1010` device.
* Verify and correct the output (LED) control mechanism for both devices based on their HID descriptors and behavior.