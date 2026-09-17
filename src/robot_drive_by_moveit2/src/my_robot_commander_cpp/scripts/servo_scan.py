#!/usr/bin/env python3
"""
servo_scan.py - ZP25S bus wiring checker for a two-bus driver board.

Does NOT need ROS: only python3 + pyserial.

Your current wiring:
  bus A (driver board port 1): servos 1, 3, 4, 5 daisy-chained
  bus B (driver board port 2): servo 2 alone

This script discovers the serial ports, asks every servo ID to report its
position, and prints which ID answers on which port. Reading (PRAD) is
completely safe - no motion. Use --move to also nudge each servo found.

Usage:
  python3 servo_scan.py                      # read-only wiring check
  python3 servo_scan.py --move               # also move each servo +/-15 deg
  python3 servo_scan.py --ports /dev/ttyACM0 /dev/ttyACM1
  python3 servo_scan.py --ids 1 2 3 4 5
"""

import argparse
import glob
import os
import re
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is missing. Install it with:")
    print("  sudo apt install -y python3-serial")
    sys.exit(1)

BAUD = 115200
REPLY_RE = re.compile(rb"#(\d{3})P(\d{3,5})!")
DEFAULT_IDS = [1, 2, 3, 4, 5]


def discover_ports():
    """Stable by-id names first, then the plain /dev names (deduplicated)."""
    candidates = []
    candidates += sorted(glob.glob("/dev/serial/by-id/*"))
    candidates += sorted(glob.glob("/dev/ttyACM*"))
    candidates += sorted(glob.glob("/dev/ttyUSB*"))
    seen = set()
    ports = []
    for p in candidates:
        real = os.path.realpath(p)
        if real in seen:
            continue
        seen.add(real)
        ports.append(p)
    return ports


def open_port(port):
    return serial.Serial(port, BAUD, timeout=0.02)


def query_position(ser, servo_id, timeout=0.15):
    """Send #<ID>PRAD! and return the P value, or None if no reply."""
    ser.reset_input_buffer()
    ser.write(("#%03dPRAD!" % servo_id).encode("ascii"))
    ser.flush()

    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(1)
        if not chunk:
            continue
        buf += chunk
        if chunk == b"!":
            m = REPLY_RE.search(buf)
            if m and int(m.group(1)) == servo_id:
                return int(m.group(2))
            buf = b""
    return None


def send_position(ser, servo_id, p_value, duration_ms):
    ser.write(("#%03dP%04dT%04d!" % (servo_id, p_value, duration_ms)).encode("ascii"))
    ser.flush()
    time.sleep(duration_ms / 1000.0 + 0.15)


def main():
    ap = argparse.ArgumentParser(description="ZP25S two-bus wiring checker")
    ap.add_argument("--ports", nargs="+", help="serial ports to scan")
    ap.add_argument("--ids", nargs="+", type=int, default=DEFAULT_IDS)
    ap.add_argument("--move", action="store_true",
                    help="after scanning, nudge every servo found (+/-15 deg)")
    ap.add_argument("--move-time", type=int, default=600, help="move duration ms")
    args = ap.parse_args()

    ports = args.ports or discover_ports()
    if not ports:
        print("No serial ports found (/dev/ttyACM* or /dev/ttyUSB*).")
        print("Check the USB cable between the driver board and the Orange Pi.")
        sys.exit(1)

    print("Ports found:")
    for p in ports:
        print("  ", p)
    print()

    # id -> [(port, P value), ...]
    found = {}

    for port in ports:
        try:
            ser = open_port(port)
        except Exception as exc:
            print("[%s] cannot open: %s" % (port, exc))
            print("       (check permissions: sudo usermod -aG dialout $USER)")
            continue

        print("[%s] scanning IDs %s ..." % (port, args.ids))
        with ser:
            for sid in args.ids:
                p = query_position(ser, sid)
                if p is None:
                    print("   id %03d : no reply" % sid)
                else:
                    print("   id %03d : P = %d" % (sid, p))
                    found.setdefault(sid, []).append((port, p))

        print()

    # ---- summary -----------------------------------------------------------
    print("=" * 60)
    print("SUMMARY - which servo answers on which bus")
    print("=" * 60)
    if not found:
        print("No servo replied on any port.")
        print("Things to check:")
        print("  1. driver board power (servos need their own supply)")
        print("  2. TX/RX wiring, TTL level (not RS232)")
        print("  3. baud rate 115200")
        print("  4. servo IDs already assigned")
        sys.exit(2)

    for sid in sorted(found):
        entries = found[sid]
        for port, p in entries:
            print("  id %03d  ->  %s   (P = %d)" % (sid, port, p))
        if len(entries) > 1:
            print("     WARNING: id %03d answered on %d ports "
                  "(buses may be wired together)" % (sid, len(entries)))

    print()
    print("Expected wiring:")
    print("  bus A: ids 1, 3, 4, 5")
    print("  bus B: id 2")

    # ---- optional motion test ---------------------------------------------
    if args.move:
        print()
        print("Motion test: each servo +/-15 deg (%.1f deg/step -> %.0f steps)"
              % (120.0 / 900.0, 15.0 / (120.0 / 900.0)))
        steps = int(round(15.0 / (120.0 / 900.0)))
        for sid in sorted(found):
            for port, p0 in found[sid]:
                ser = open_port(port)
                with ser:
                    print("  id %03d on %s : %d -> %d -> %d"
                          % (sid, port, p0, p0 + steps, p0))
                    send_position(ser, sid, p0 + steps, args.move_time)
                    send_position(ser, sid, p0, args.move_time)
        print("Motion test done.")


if __name__ == "__main__":
    main()
