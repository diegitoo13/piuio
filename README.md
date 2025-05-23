
# PIUIO‑1020 Linux HID Driver

Small out‑of‑tree **kernel module** that turns the **Andamiro PIUIO USB I/O board (product 0x1020)** into a standard Linux game‑pad and exposes its coin counter.

LED / lamp output is not implemented yet – this driver is **input‑only**.

---

## Features

| Feature                | Status      | Notes                                                                                          |
|------------------------|-------------|------------------------------------------------------------------------------------------------|
| 10 buttons per player  | Implemented | Delivered as normal `EV_KEY` events (`/dev/input/eventX`).                                     |
| 16‑bit coin counter    | Implemented | Any change on byte 15 produces a single `BTN_MODE` pulse.                                      |
| Board handshake        | Implemented | Sends `SET_IDLE` plus 19‑byte wake report (`0x81 FF …`).                                       |
| LED / lamp control     | Planned     | Interrupt‑OUT format still being mapped.                                                       |

### Default button map

| Panel      | Bit mask | Player 1      | Player 2    |
|------------|----------|---------------|-------------|
| Down‑Left  | 0x01     | BTN_SOUTH     | BTN_TR      |
| Down‑Right | 0x08     | BTN_WEST      | BTN_SELECT  |
| Up‑Left    | 0x04     | BTN_NORTH     | BTN_START   |
| Up‑Right   | 0x02     | BTN_EAST      | BTN_THUMBL  |
| Center     | 0x10     | BTN_TL        | BTN_THUMBR  |

Key codes can be remapped later with `udev hwdb` or userspace tools.

---

## Quick install (copy‑paste)

```bash
# get source
git clone https://github.com/diegitoo13/piuio.git
cd piuio/mod

# build module
make

# install system‑wide (optional)
sudo make install
sudo depmod -a

# load module
sudo modprobe piuio_hid        # add poll_ms=8 if you want a different poll rate
```

To test:

```bash
dmesg | grep piuio_gp
evtest /dev/input/eventX
```

Unload:

```bash
sudo rmmod piuio_hid
```

Autoload at boot:

```bash
echo piuio_hid | sudo tee /etc/modules-load.d/piuio-hid.conf
```

### Module parameter

* `poll_ms=<n>` – polling interval in ms (default 4, range 1‑1000).

---

## Roadmap

* LED output: determine 19‑byte Interrupt‑OUT payload (report `0x82`).
* Legacy board 0x1010: add support once protocol is captured.
* Replace EP0 polling with interrupt‑IN when report structure is confirmed.

---

## License

GPL‑2.0 – same license as the Linux kernel.

Author: Diego Acevedo, 2025
