#!/usr/bin/env python3
"""
servo_bus_check.py - ZP25S bus connectivity scan (READ-ONLY, nothing moves).

Purpose:
  The arm's servos are wired as two groups on two ports of the driver board
  (1/3/4/5 on one port, 2 on another). This tool finds out whether those two
  ports are the SAME serial bus (parallel connectors) or TWO separate buses,
  and which servo IDs answer on each port.

  It only sends '#<ID>PRAD!' (read position). No motion command is ever sent,
  so it is safe to run with the arm powered and assembled.

Usage:
  python3 servo_bus_check.py                     # scan auto-detected ports
  python3 servo_bus_check.py /dev/ttyACM0 /dev/ttyACM1
  python3 servo_bus_check.py --ids 1-6           # limit ID range
  python3 servo_bus_check.py --move 2 1600 800   # OPTIONAL motion test (careful!)
"""

import argparse
import glob
import sys
import time

import serial

BAUD = 115200
READ_TIMEOUT_S = 0.05


def candidate_ports():
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    return ports


def read_reply(ser, wait_s=0.25):
    """Read bytes until '!' or timeout. Returns the decoded string."""
    buf = b""
    t0 = time.time()
    while time.time() - t0 < wait_s:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            if b"!" in buf:
                break
        else:
            time.sleep(0.005)
    return buf.decode("ascii", errors="replace")


def query_position(ser, servo_id):
    """Send '#<ID>PRAD!' and return the P value, or None if no valid reply."""
    ser.reset_input_buffer()
    ser.write(f"#{servo_id:03d}PRAD!".encode("ascii"))
    ser.flush()
    reply = read_reply(ser)
    if "P" not in reply:
        return None, reply
    try:
        value = int(reply.split("P")[1].rstrip("!").strip())
    except (ValueError, IndexError):
        return None, reply
    return value, reply


def parse_ids(text):
    ids = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            ids.extend(range(int(a), int(b) + 1))
        elif part:
            ids.append(int(part))
    return ids


def scan(ports, ids):
    results = {}
    for port in ports:
        print(f"\n=== {port} ===")
        try:
            ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT_S)
        except Exception as exc:
            print(f"  open failed - {exc}")
            results[port] = {}
            continue

        time.sleep(0.3)
        ser.reset_input_buffer()

        found = {}
        for sid in ids:
            value, raw = query_position(ser, sid)
            if value is not None:
                found[sid] = value
                print(f"  ID {sid:03d}  ->  P{value:<6} (raw: {raw.strip()!r})")
            else:
                tail = f" (raw: {raw.strip()!r})" if raw.strip() else ""
                print(f"  ID {sid:03d}  ->  no reply{tail}")
            time.sleep(0.03)

        ser.close()
        results[port] = found
        print(f"  found on {port}: {sorted(found.keys())}")

    return results


def summarise(results):
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    for port, found in results.items():
        print(f"  {port}: IDs {sorted(found.keys())}")

    usable = [p for p, f in results.items() if f]
    if len(usable) < 2:
        if usable:
            print("\nOnly one port answered. Either the other port has no servos,")
            print("or both ports are the same bus and share this device node.")
        return

    sets = [set(results[p].keys()) for p in usable]
    if sets[0] == sets[1]:
        print("\n==> SAME BUS: both ports see the same IDs.")
        print("    The two connectors are the same serial bus (parallel).")
        print("    Your existing single-port driver works as-is.")
    elif sets[0].isdisjoint(sets[1]):
        print("\n==> TWO SEPARATE BUSES: each port sees different IDs.")
        print("    The driver must talk to BOTH ports (one serial port per group).")
    else:
        print("\n==> PARTIALLY OVERLAPPING: unusual, check wiring/termination.")


def motion_test(port, servo_id, p_value, duration_ms):
    """Optional single-servo motion test. Moves ONE servo only."""
    print(f"\n=== MOTION TEST on {port}: ID {servo_id}, P{p_value}, T{duration_ms}ms ===")
    print("WARNING: the arm WILL move. Keep clear / be ready to cut power.")
    ser = serial.Serial(port, BAUD, timeout=READ_TIMEOUT_S)
    time.sleep(0.3)
    ser.reset_input_buffer()

    before, _ = query_position(ser, servo_id)
    print(f"  current P before: {before}")

    ser.write(f"#{servo_id:03d}P{p_value:04d}T{duration_ms:04d}!".encode("ascii"))
    ser.flush()
    time.sleep(duration_ms / 1000.0 + 0.5)

    after, _ = query_position(ser, servo_id)
    print(f"  current P after : {after}")

    # back to the position it started from
    if before is not None:
        ser.write(f"#{servo_id:03d}P{before:04d}T{duration_ms:04d}!".encode("ascii"))
        ser.flush()
        time.sleep(duration_ms / 1000.0 + 0.3)
        print(f"  returned to P{before}")

    ser.close()


def main():
    ap = argparse.ArgumentParser(description="ZP25S bus connectivity check")
    ap.add_argument("ports", nargs="*", help="serial ports (default: auto-detect)")
    ap.add_argument("--ids", default="1-6", help="ID list/range, e.g. 1-6 or 1,2,5")
    ap.add_argument("--move", nargs=3, metavar=("ID", "P", "MS"),
                    help="optional motion test: move one servo to P for MS")
    args = ap.parse_args()

    ids = parse_ids(args.ids)
    ports = args.ports or candidate_ports()

    if not ports:
        print("No /dev/ttyACM* or /dev/ttyUSB* device found.")
        print("Check the USB cable, then: ls -l /dev/ttyACM* /dev/ttyUSB*")
        return 1

    print(f"Scanning ports: {ports}")
    print(f"ID range: {ids}")
    print("(read-only: only PRAD is sent, no motor motion)\n")

    results = scan(ports, ids)
    summarise(results)

    if args.move:
        servo_id, p_value, duration_ms = (int(x) for x in args.move)
        if not ports:
            print("no port for motion test")
            return 1
        motion_test(ports[0], servo_id, p_value, duration_ms)

    return 0


if __name__ == "__main__":
    sys.exit(main())
