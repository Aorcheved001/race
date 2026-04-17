# -*- coding: utf-8 -*-
"""
Serial Reader - AURIX TC377 Wireless UART Monitor
COM24 Wireless UART - 115200bps - Data logging

Usage:
    python serial_reader.py              # Default COM24:115200
    python serial_reader.py --port COM24 # Specify port
    python serial_reader.py --save       # Save to log file
"""

import sys
import os
import time
import argparse
import logging
from datetime import datetime
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


class SerialReader:
    def __init__(self, port="COM36", baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.running = False
        self.line_buffer = deque(maxlen=1000)
        self.stats = {
            "lines_received": 0,
            "bytes_received": 0,
            "errors": 0,
            "start_time": None,
        }

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("[INFO] No COM ports found")
            return []

        print("\n[INFO] Available COM ports:")
        print("-" * 50)
        for p in ports:
            marker = " <-- TARGET" if p.device == self.port else ""
            print(f"  {p.device:<8} {p.description or 'N/A':<30}{marker}")
        print("-" * 50)
        return ports

    def connect(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self.stats["start_time"] = time.time()
            self.running = True
            print(
                f"[OK] Connected {self.port} @ {self.baudrate}bps "
                f"(timeout={self.timeout}s)"
            )
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Failed to open {self.port}: {e}")
            print("\n[HELP] Check:")
            print("  1. Is the device plugged in?")
            print("  2. Is another app using this port?")
            print("  3. Run with --list to see available ports")
            return False

    def disconnect(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"\n[INFO] Disconnected from {self.port}")

    def read_line(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return None

        try:
            line = self.serial_conn.readline()
            if not line:
                return None

            self.stats["bytes_received"] += len(line)

            try:
                decoded = line.decode("utf-8", errors="replace").strip()
            except Exception:
                decoded = repr(line)

            if decoded:
                self.stats["lines_received"] += 1
                self.line_buffer.append(decoded)
                return decoded
            return None
        except serial.SerialException as e:
            self.stats["errors"] += 1
            print(f"[ERROR] Read error: {e}")
            return None

    def read_lines(self, count=0, callback=None):
        lines_read = 0
        while self.running:
            if count > 0 and lines_read >= count:
                break

            line = self.read_line()
            if line is not None:
                lines_read += 1
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                if callback:
                    callback(timestamp, line)
                else:
                    print(f"  [{timestamp}] {line}")
            else:
                time.sleep(0.001)

        return lines_read

    def get_stats(self):
        elapsed = 0
        if self.stats["start_time"]:
            elapsed = time.time() - self.stats["start_time"]

        rate = 0
        if elapsed > 0:
            rate = self.stats["lines_received"] / elapsed

        return {
            "lines": self.stats["lines_received"],
            "bytes": self.stats["bytes_received"],
            "errors": self.stats["errors"],
            "elapsed_s": round(elapsed, 1),
            "lines_per_sec": round(rate, 1),
        }

    def print_stats(self):
        s = self.get_stats()
        print(
            f"\n{'='*50}\n"
            f"  Statistics:\n"
            f"  Lines received : {s['lines']}\n"
            f"  Bytes received: {s['bytes']}\n"
            f"  Errors         : {s['errors']}\n"
            f"  Elapsed        : {s['elapsed_s']}s\n"
            f"  Rate           : {s['lines_per_sec']} lines/s\n"
            f"{'='*50}"
        )


def save_to_file(log_path, reader, duration=0, max_lines=0):
    """Read and save to file with optional duration/line limit."""
    print(f"[INFO] Saving to: {log_path}")
    print("[INFO] Press Ctrl+C to stop\n")

    start = time.time()
    line_count = 0

    with open(log_path, "w", encoding="utf-8", buffering=1) as f:
        f.write(f"# Serial Log - {datetime.now().isoformat()}\n")
        f.write(f"# Port: {reader.port} @ {reader.baudrate}bps\n")
        f.write("#" + "=" * 60 + "\n")

        def write_callback(timestamp, line):
            nonlocal line_count
            line_count += 1
            formatted = f"[{timestamp}] {line}"
            print(formatted)
            f.write(formatted + "\n")

        try:
            reader.read_lines(count=max_lines, callback=write_callback)
        except KeyboardInterrupt:
            pass

    print(f"\n[INFO] Saved {line_count} lines to {log_path}")


def main():
    parser = argparse.ArgumentParser(
        description="AURIX TC377 Wireless UART Reader",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python serial_reader.py                  # Default COM24:115200
  python serial_reader.py --port COM24     # Specify port
  python serial_reader.py --baud 9600      # Custom baud rate
  python serial_reader.py --list           # List available ports
  python serial_reader.py --save           # Save to log file
  python serial_reader.py --save -t 30     # Save for 30 seconds
  python serial_reader.py --save -n 500    # Save 500 lines then exit
        """,
    )

    parser.add_argument(
        "--port", "-p", default="COM24",
        help="Serial port (default: COM24)",
    )
    parser.add_argument(
        "--baud", "-b", type=int, default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--list", "-l", action="store_true",
        help="List available COM ports and exit",
    )
    parser.add_argument(
        "--save", "-s", action="store_true",
        help="Save output to log file",
    )
    parser.add_argument(
        "--time", "-t", type=int, default=0,
        help="Duration in seconds (0=infinite)",
    )
    parser.add_argument(
        "--lines", "-n", type=int, default=0,
        help="Max lines to read (0=infinite)",
    )
    parser.add_argument(
        "--output", "-o", default=None,
        help="Output log file path (default: auto-generated)",
    )
    parser.add_argument(
        "--quiet", "-q", action="store_true",
        help="Suppress statistics on exit",
    )

    args = parser.parse_args()

    reader = SerialReader(port=args.port, baudrate=args.baud)

    if args.list:
        reader.list_ports()
        return

    if not reader.connect():
        sys.exit(1)

    try:
        if args.save:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = args.output or f"serial_log_{args.port}_{timestamp_str}.txt"

            if args.time > 0:
                import threading
                timer = threading.Timer(args.time, lambda: setattr(reader, "running", False))
                timer.start()

            save_to_file(log_path, reader, duration=args.time, max_lines=args.lines)
        else:
            print(f"\n[INFO] Reading from {args.port} (Ctrl+C to stop)\n")
            print("-" * 60)
            reader.read_lines(count=args.lines)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        if not args.quiet:
            reader.print_stats()
        reader.disconnect()


if __name__ == "__main__":
    main()
