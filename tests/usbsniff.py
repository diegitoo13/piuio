#!/usr/bin/env python3
"""
PIUIO sniffer + *very* small lamp driver.

 • Polls feature‑report 0x01 (258 bytes) at 100 Hz and prints changes
 • Lets you switch individual pad lamps on/off:
      --p1 L on      (LEFT)
      --p1 ULDRC off (Up‑Left‑Down‑Right‑Centre at once)
      --p2 C on      (Player‑2 CENTRE)
   …or just run with no lamp options to sniff only.

Works on macOS/Linux/Windows, requires libusb + PyUSB.
"""

import argparse, sys, time, usb.core, usb.util

# ----------------------------- constants -----------------------------

DEFAULT_VID = 0x0D2F
DEFAULT_PID = 0x1020
INTERFACE        = 0
TIMEOUT_MS       = 500
POLL_REPORT_ID   = 0x01
POLL_REPORT_SIZE = 258           # bytes we saw in the trace
EP_OUT_ADDR      = 0x02          # HID interrupt‑OUT endpoint
BITMASK = {
    'L': 0,  # LEFT
    'R': 1,  # RIGHT
    'U': 2,  # UP
    'D': 3,  # DOWN
    'C': 4,  # CENTRE
}

# ------------------------ USB helper functions -----------------------

def find_device(vid, pid):
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    if dev is None:
        sys.exit(f"❌  No device {vid:04x}:{pid:04x} found")
    return dev

def detach(dev):
    try:
        if dev.is_kernel_driver_active(INTERFACE):
            dev.detach_kernel_driver(INTERFACE)
    except (NotImplementedError, AttributeError):
        pass   # not supported on this OS

def set_idle(dev):
    try:
        dev.ctrl_transfer(0x21, 0x0A, 0, INTERFACE, None, TIMEOUT_MS)
        print("✅  SET_IDLE accepted")
    except usb.core.USBError as e:
        if e.errno == 32:   # STALL
            print("⚠️  SET_IDLE stalled – continuing anyway")
        else:
            raise

def get_feature(dev, report_id, size):
    wValue = (0x03 << 8) | report_id
    return dev.ctrl_transfer(0xA1, 0x01, wValue, INTERFACE, size, TIMEOUT_MS)

# --------------------------- lamp helpers ----------------------------

def compute_masks(p1_str, p1_on, p2_str, p2_on):
    """
    Return two 16‑bit masks (active‑low) for P1 and P2.
    Bits that are *not* mentioned stay at 1 (lamp unchanged/off);
    bits that are mentioned are forced to the requested 0/1.
    """
    p1_mask = 0xFFFF
    p2_mask = 0xFFFF
    for ch in p1_str.upper():
        if ch not in BITMASK:
            sys.exit(f"Unknown lamp '{ch}' for P1")
        if p1_on:
            p1_mask &= ~(1 << BITMASK[ch])
        else:
            p1_mask |=  (1 << BITMASK[ch])

    for ch in p2_str.upper():
        if ch not in BITMASK:
            sys.exit(f"Unknown lamp '{ch}' for P2")
        if p2_on:
            p2_mask &= ~(1 << BITMASK[ch])
        else:
            p2_mask |=  (1 << BITMASK[ch])
    return p1_mask, p2_mask

def send_lamps(dev, p1_mask, p2_mask):
    """
    Build the 16‑byte HID packet we saw in the sniff and send it to ep 0x02.
    Only the first four bytes matter, the rest stay 0.
    """
    payload = bytearray(16)
    payload[0] =  p1_mask        & 0xFF
    payload[1] = (p1_mask >> 8)  & 0xFF
    payload[2] =  p2_mask        & 0xFF
    payload[3] = (p2_mask >> 8)  & 0xFF
    dev.write(EP_OUT_ADDR, payload, TIMEOUT_MS)

# ------------------------------ main ---------------------------------

def hexdump(buf, cols=16):
    it = (buf[i:i+cols] for i in range(0, len(buf), cols))
    return '\n'.join(' '.join(f'{b:02X}' for b in chunk) for chunk in it)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vid', type=lambda x:int(x,16), default=DEFAULT_VID)
    ap.add_argument('--pid', type=lambda x:int(x,16), default=DEFAULT_PID)
    ap.add_argument('--p1', nargs=2, metavar=('LAMPS','on|off'),
                    help="control P1 lamps: e.g. --p1 ULDRC on")
    ap.add_argument('--p2', nargs=2, metavar=('LAMPS','on|off'),
                    help="control P2 lamps: e.g. --p2 C off")
    args = ap.parse_args()

    dev = find_device(args.vid, args.pid)
    detach(dev)
    dev.set_configuration()
    set_idle(dev)

    # -----------------------------------------------------------------
    # 1) If lamp options were given, send them once and exit
    # -----------------------------------------------------------------
    if args.p1 or args.p2:
        p1_str, p1_on = args.p1 if args.p1 else ('', 'off')
        p2_str, p2_on = args.p2 if args.p2 else ('', 'off')
        p1_mask, p2_mask = compute_masks(
            p1_str, p1_on.lower() == 'on',
            p2_str, p2_on.lower() == 'on'
        )
        print(f"💡  Sending lamp packet  P1=0x{p1_mask:04X}  P2=0x{p2_mask:04X}")
        send_lamps(dev, p1_mask, p2_mask)
        return

    # -----------------------------------------------------------------
    # 2) Otherwise fall back to pure input sniffer
    # -----------------------------------------------------------------
    last = None
    print("🔍  Polling feature‑report 0x01 – Ctrl‑C to quit.")
    while True:
        data = get_feature(dev, POLL_REPORT_ID, POLL_REPORT_SIZE)
        if last is None or data != last:
            ts   = time.strftime('%H:%M:%S')
            diff = [i for i,(a,b) in enumerate(zip(data, last or data)) if a!=b]
            print(f'[{ts}] len={len(data)}  changed bytes: {diff[:32]}')
            print(hexdump(data[:64]))
            last = data
        time.sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋  Bye!")
