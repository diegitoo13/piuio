# PIUIO-1020 Linux HID Driver

A compact out‑of‑tree **kernel module** that turns the **Andamiro PIUIO USB I/O
board (product 0x1020)** into a standard Linux game‑pad and exposes its credit
counter.

> LED / lamp output is not implemented yet – this driver is **input‑only**.

---

## Features

| Feature | Status | Notes |
|---------|--------|-------|
| 10 buttons per player | Implemented | Delivered as normal **EV_KEY** events (`/dev/input/eventX`). |
| 16‑bit coin counter   | Implemented | Any change on byte 15 produces a single `BTN_MODE` pulse. |
| Board handshake       | Implemented | Sends `SET_IDLE` plus 19‑byte wake report (`0x81 FF …`). |
| LED / lamp control    | Planned     | Interrupt‑OUT format still being mapped. |

### Default button map

| Panel | Bit mask | Player 1 | Player 2 |
|-------|----------|----------|----------|
| Down‑Left  | `0x01` | `BTN_SOUTH` | `BTN_TR` |
| Down‑Right | `0x08` | `BTN_WEST`  | `BTN_SELECT` |
| Up‑Left    | `0x04` | `BTN_NORTH` | `BTN_START` |
| Up‑Right   | `0x02` | `BTN_EAST`  | `BTN_THUMBL` |
| Center     | `0x10` | `BTN_TL`    | `BTN_THUMBR` |

These key codes can be remapped later with `udev hwdb` or userspace tools.

---

## Quick installation

```bash
# 1. Get the source
git clone https://github.com/diegitoo13/piuio.git
cd piuio/mod        # contains Makefile + piuio_hid.c

# 2. Build
make                          # produces piuio_hid.ko

# 3. Install system‑wide (optional)
sudo make install             # copies to /lib/modules/$(uname -r)/updates
sudo depmod -a                # refresh module dependency graph

# 4. Load the driver
sudo modprobe piuio_hid       # or: sudo insmod piuio_hid.ko
```

To test:

```bash
dmesg | grep piuio_gp          # should log “Attached, polling …”
evtest /dev/input/eventX       # press panels and watch key events
```

Unload with:

```bash
sudo rmmod piuio_hid
```

Autoload at boot:

```bash
echo piuio_hid | sudo tee /etc/modules-load.d/piuio-hid.conf
```

### Module parameter

* `poll_ms=<n>` – polling interval in milliseconds (default 4, range 1‑1000).

Example:

```bash
sudo modprobe piuio_hid poll_ms=8
```

---

## Roadmap

* LED output: identify the 19‑byte Interrupt‑OUT payload (report `0x82`).
* Legacy board (0x1010): add support once protocol is captured.
* Interrupt‑IN path: replace EP0 polling when the raw report structure is confirmed.

---

## Contributing

USB traces, LED packet analysis and clean‑ups are welcome.  
Licensed under **GPL‑2.0**, same as the Linux kernel.

*Author: Diego Acevedo — 2025*