#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Raw GNSS serial monitor for GN43RFA/RTK debugging.

This tool prints raw NMEA lines and decodes the fields we care about:
GGA quality, satellite count, RMC status/mode, and THS heading validity.
"""

import argparse
import os
import sys
import time
from datetime import datetime

import serial
from serial.tools import list_ports


QUALITY_TEXT = {
    "0": "invalid",
    "1": "single GPS",
    "2": "DGPS",
    "4": "RTK fixed",
    "5": "RTK float",
    "6": "estimated",
}

MODE_TEXT = {
    "A": "autonomous",
    "D": "differential",
    "E": "estimated",
    "F": "RTK float",
    "M": "manual",
    "N": "not valid",
    "R": "RTK fixed",
    "S": "simulator",
}


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("[PORTS] none")
        return []
    print("[PORTS]")
    for p in ports:
        print(f"  {p.device:8s} {p.description} {p.hwid}")
    return ports


def checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return None
    body, checksum = line[1:].split("*", 1)
    checksum = checksum[:2]
    value = 0
    for ch in body:
        value ^= ord(ch)
    try:
        expected = int(checksum, 16)
    except ValueError:
        return False
    return value == expected


def split_nmea(line):
    if not line.startswith("$"):
        return None
    body = line[1:].split("*", 1)[0]
    return body.split(",")


def parse_line(line):
    fields = split_nmea(line)
    if not fields or not fields[0]:
        return None

    sentence = fields[0]
    kind = sentence[-3:]
    ok = checksum_ok(line)
    prefix = "OK" if ok is True else "BAD" if ok is False else "NOCHK"

    try:
        if kind == "GGA":
            quality = fields[6] if len(fields) > 6 else ""
            sats = fields[7] if len(fields) > 7 else ""
            hdop = fields[8] if len(fields) > 8 else ""
            alt = fields[9] if len(fields) > 9 else ""
            text = QUALITY_TEXT.get(quality, "unknown")
            return (
                "GGA",
                f"[{prefix}] {sentence} quality={quality}({text}) "
                f"sats={sats} hdop={hdop} alt={alt}",
            )

        if kind == "RMC":
            status = fields[2] if len(fields) > 2 else ""
            speed = fields[7] if len(fields) > 7 else ""
            course = fields[8] if len(fields) > 8 else ""
            mode = fields[12] if len(fields) > 12 else ""
            mode_text = MODE_TEXT.get(mode, "unknown") if mode else "none"
            return (
                "RMC",
                f"[{prefix}] {sentence} status={status} mode={mode}({mode_text}) "
                f"speed_kn={speed} course={course}",
            )

        if kind == "THS":
            heading = fields[1] if len(fields) > 1 else ""
            status = fields[2] if len(fields) > 2 else ""
            return (
                "THS",
                f"[{prefix}] {sentence} heading={heading} status={status}",
            )
    except Exception as exc:
        return ("ERR", f"parse error: {exc}")

    return None


def open_log_file(save_dir):
    os.makedirs(save_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(save_dir, f"gnss_raw_{ts}.txt")
    return path, open(path, "w", encoding="utf-8", buffering=1)


def monitor(port, baud, seconds, raw, save_dir):
    log_path, log_file = open_log_file(save_dir)
    print(f"[OPEN] {port} @ {baud}, {seconds:.1f}s")
    print(f"[SAVE] {log_path}")
    counts = {"GGA": 0, "RMC": 0, "THS": 0, "ERR": 0, "RAW": 0}
    last = {}
    start = time.time()

    with log_file:
        with serial.Serial(port=port, baudrate=baud, timeout=0.2) as ser:
            while time.time() - start < seconds:
                data = ser.readline()
                if not data:
                    continue
                line = data.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                ts = time.time() - start
                log_file.write(f"{ts:.3f} {line}\n")

                parsed = parse_line(line)
                if parsed:
                    key, text = parsed
                    counts[key] = counts.get(key, 0) + 1
                    last[key] = text
                    print(f"[{ts:7.3f}] {text}")
                elif raw:
                    counts["RAW"] += 1
                    print(f"[{ts:7.3f}] RAW {line}")

    print("[DONE]")
    print("  counts:", " ".join(f"{k}={v}" for k, v in counts.items()))
    for key in ("GGA", "RMC", "THS"):
        if key in last:
            print(f"  last {key}: {last[key]}")
    print(f"  file: {log_path}")
    return log_path


def main(argv=None):
    parser = argparse.ArgumentParser(description="Raw GNSS serial debugger")
    parser.add_argument("--list", action="store_true", help="list serial ports")
    parser.add_argument("--port", default="", help="serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--raw", action="store_true", help="also print unparsed lines")
    parser.add_argument("--save-dir", default="track_data")
    args = parser.parse_args(argv)

    if args.list or not args.port:
        ports = list_serial_ports()
        if not args.port:
            if len(ports) == 1:
                print(f"[HINT] one port found: --port {ports[0].device}")
            return 0

    try:
        monitor(args.port, args.baud, args.seconds, args.raw, args.save_dir)
    except serial.SerialException as exc:
        print(f"[ERROR] serial open/read failed: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[STOP]")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
