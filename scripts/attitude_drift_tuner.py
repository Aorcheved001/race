#!/usr/bin/env python3
# -*- coding: ascii -*-
"""
One-click static attitude drift capture and analysis.

Flow:
1. Capture attitude logs from the selected serial port (default COM36)
2. Save the log to a file
3. Run analyze_drift.py for roll / pitch / yaw drift analysis
"""

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def build_default_log_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("data") / f"att_drift_{timestamp}.txt"


def run_command(command):
    result = subprocess.run(command, text=True)
    if result.returncode != 0:
        sys.exit(result.returncode)


def main():
    parser = argparse.ArgumentParser(description="Capture and analyze static attitude drift")
    parser.add_argument("--port", "-p", default="COM36", help="Serial port (default: COM36)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--time", "-t", type=int, default=60, help="Capture duration in seconds (default: 60)")
    parser.add_argument("--output", "-o", default=None, help="Output log file path")
    args = parser.parse_args()

    log_path = Path(args.output) if args.output else build_default_log_path()
    log_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[ATT_TUNER] Start capture: port={args.port}, duration={args.time}s")
    run_command([
        sys.executable,
        "scripts/serial_reader.py",
        "--port", args.port,
        "--baud", str(args.baud),
        "--time", str(args.time),
        "--save",
        "--output", str(log_path),
    ])

    print(f"[ATT_TUNER] Capture done, start analysis: {log_path}")
    run_command([
        sys.executable,
        "scripts/analyze_drift.py",
        str(log_path),
    ])


if __name__ == "__main__":
    main()
