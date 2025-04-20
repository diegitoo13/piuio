# PIUIO HID Driver for Linux

This is a Linux kernel module driver for Andamiro PIUIO arcade I/O boards, interfacing via the standard HID subsystem.

## Features

* Provides input events for panels/buttons via the Linux input subsystem (e.g., `/dev/input/eventX`).
* Provides LED control via the Linux LED subsystem (`/sys/class/leds/`).
* Provides a legacy `/dev/piuioX` character device interface for compatibility with some older applications.
* Supports Product IDs `0d2f:1010` (legacy) and `0d2f:1020` (newer).

## Current Status (Work In Progress)

* **Device Matching:** Matches both `0d2f:1010` and `0d2f:1020` devices.
* **Initialization (`1020`):** Sends the required `SET_IDLE` command during probe.
* **Input Handling (`1020`):** Uses **timer-based polling with GET_REPORT control transfers**, mimicking observed arcade software and Python script behavior. Parses the expected 16-byte input report size. **Note:** The exact bit-to-button mapping within the 16 bytes is currently assumed and requires verification via testing.
* **Input Handling (`1010`):** The input report mechanism and format for the legacy device are currently **unknown**. Input events for this device are **ignored** until its protocol is determined.
* **Output/LED Handling (`1020`):** Uses **Interrupt OUT transfers** via Endpoint 0x02, matching the HID descriptor and observed successful transfers. The exact format/mapping of the 16-byte output report still requires full verification.
* **Output/LED Handling (`1010`):** Attempts to use legacy `SET_REPORT` control transfers as a fallback (protocol unverified).

## Compiling and Installing

1.  Ensure you have the kernel headers installed for your running kernel (e.g., `sudo apt install linux-headers-$(uname -r)` on Debian/Ubuntu).
2.  Navigate to the `mod` directory containing `piuio_hid.c`, `piuio_hid.h`, and `Makefile`.
3.  Run `make` to compile `piuio_hid.ko`.
4.  Run `sudo make install` to copy the module to `/lib/modules/$(uname -r)/updates/` (or similar) and update module dependencies. (Requires root privileges).
5.  Load the module: `sudo modprobe piuio_hid`
6.  Unload the module: `sudo rmmod piuio_hid`

## Testing

* **Input (`1020` Device):** Use tools like `evtest` (install if needed) on the relevant `/dev/input/eventX` device created when the driver binds. Carefully verify if button presses map to the expected events.
* **Output (`1020` LEDs):** Control LEDs via `/sys/class/leds/`. Find the LED names associated with your device (e.g., `/sys/class/leds/piuio-0003:0D2F:1020.XXXX::outputY`) and echo 0 or 1 to the `brightness` file (e.g., `echo 1 | sudo tee /sys/class/leds/DEVICE_NAME::output3/brightness`). Verify if the correct LEDs light up.
* **Legacy Device (`1010`):** Functionality is currently limited/unknown. Test applications using `/dev/piuio0` (or similar) for the legacy write interface, but expect input to be non-functional.

## Contributing / Further Work

* **Determine the exact input report format (`1020`):** Analyze captured `GET_REPORT` response data to confirm the bit mapping.
* **Determine the input mechanism/format (`1010`):** Requires capturing data from a legacy device.
* **Verify/Correct Output Format (`1020`):** Confirm the exact format expected for the 16-byte Interrupt OUT report.
* **Verify/Correct Output Mechanism/Format (`1010`):** Determine how LEDs are controlled on the legacy device.