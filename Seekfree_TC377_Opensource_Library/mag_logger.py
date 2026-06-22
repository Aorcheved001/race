# -*- coding: utf-8 -*-
"""Collect magnetometer x,y,z lines from the car for offline calibration.

Supports two modes:
  --mode raw         : C 端应调用 imu_mag_send_raw_data_to_pc()
  --mode calibrated  : C 端应调用 imu_mag_send_calibrated_data_to_pc()

File name and header are automatically set based on mode.
"""

import argparse
import json
import os
import re
import time
from datetime import datetime
from pathlib import Path

import serial


MODE_INFO = {
    "raw": {
        "prefix": "mag_raw",
        "c_function": "imu_mag_send_raw_data_to_pc()",
        "header_comment": "raw magnetometer x,y,z from imu_mag_send_raw_data_to_pc()",
        "data_field": "imu660.data_Raw.mag_x/y/z",
    },
    "calibrated": {
        "prefix": "mag_calibrated",
        "c_function": "imu_mag_send_calibrated_data_to_pc()",
        "header_comment": "calibrated magnetometer x,y,z from imu_mag_send_calibrated_data_to_pc()",
        "data_field": "imu660.data_Ripen.mag_x/y/z",
    },
}

DEFAULT_HEADER_PATH = Path("code/ins/calibration_params.h")


def detect_data_mode(data_rows, declared_mode):
    """Heuristic check: warn if data characteristics don't match declared mode.

    For RAW mode: data center should be far from origin (near hard_iron offset).
    For CALIBRATED mode: data center should be near origin.
    """
    if len(data_rows) < 50:
        return None

    import numpy as np
    D = np.array(data_rows[-200:], dtype=float)
    valid = np.where(np.linalg.norm(D, axis=1) > 50)[0]
    if len(valid) < 20:
        return None
    D = D[valid]
    cx, cy = D[:, 0].mean(), D[:, 1].mean()
    dist_from_origin = np.sqrt(cx ** 2 + cy ** 2)

    if declared_mode == "raw" and dist_from_origin < 100:
        return (
            f"WARNING: suspected calibrated data!\n"
            f"  Declared mode: RAW, but data center ({cx:.0f}, {cy:.0f}) is near origin (dist={dist_from_origin:.0f})\n"
            f"  Please verify C-side is calling imu_mag_send_raw_data_to_pc()"
        )
    if declared_mode == "calibrated" and dist_from_origin > 500:
        return (
            f"WARNING: suspected raw data!\n"
            f"  Declared mode: CALIBRATED, but data center ({cx:.0f}, {cy:.0f}) is far from origin (dist={dist_from_origin:.0f})\n"
            f"  Please verify C-side is calling imu_mag_send_calibrated_data_to_pc()"
        )
    return None


def main():
    parser = argparse.ArgumentParser(description="Log magnetometer x,y,z serial output.")
    parser.add_argument("--port", default="COM24")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dir", default="track_data")
    parser.add_argument("--min-samples", type=int, default=1000)
    parser.add_argument("--mode", choices=["raw", "calibrated"], default="raw",
                        help="Data mode: raw for calibration fitting, calibrated for verification")
    args = parser.parse_args()

    info = MODE_INFO[args.mode]
    os.makedirs(args.dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = os.path.join(args.dir, f"{info['prefix']}_{timestamp}.txt")
    meta_path = os.path.join(args.dir, f"{info['prefix']}_{timestamp}.meta.json")

    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    csv_re = re.compile(rf"^\s*(?:\[[^\]]*\]\s*)?({number})\s*,\s*({number})\s*,\s*({number})\s*$")

    print(f"[MODE] {args.mode.upper()}")
    print(f"[PORT] {args.port} @ {args.baud}")
    print(f"[SAVE] {save_path}")
    print(f"[C_FUNC] Expected C-side call: {info['c_function']}")
    print(f"[DATA]   Data source: {info['data_field']}")
    if args.mode == "raw":
        print("[RECV] Push the car in a continuous, smooth, horizontal rotation. Do NOT stop at each 90 deg.")
    else:
        print("[RECV] Rotate ~90 deg and hold 2-3 seconds each stop. Complete one full circle clockwise.")
    print("[RECV] Ctrl+C to stop.\n")

    count = 0
    t0 = time.time()
    recent_rows = []
    mode_warning_shown = False

    try:
        with open(save_path, "w", encoding="utf-8", buffering=1) as f:
            # Write header with mode info
            f.write(f"# {info['header_comment']}\n")
            f.write(f"# mode: {args.mode}\n")
            f.write(f"# c_function: {info['c_function']}\n")
            f.write(f"# data_field: {info['data_field']}\n")
            f.write(f"# timestamp: {timestamp}\n")
            f.write(f"# port: {args.port} @ {args.baud}\n")
            f.write(f"# calibration_params: {DEFAULT_HEADER_PATH}\n")

            while True:
                ser = None
                try:
                    ser = serial.Serial(args.port, args.baud, timeout=0.1)
                    print(f"[OPEN] {args.port}")
                    while True:
                        line = ser.readline().decode("ascii", errors="ignore").strip()
                        if not line:
                            continue
                        match = csv_re.match(line)
                        if not match:
                            continue
                        f.write(f"{match.group(1)},{match.group(2)},{match.group(3)}\n")
                        f.flush()
                        count += 1

                        # Collect recent rows for mode detection
                        recent_rows.append([float(match.group(1)), float(match.group(2)), float(match.group(3))])
                        if len(recent_rows) > 300:
                            recent_rows = recent_rows[-300:]

                        # Check mode consistency after 200 samples
                        if not mode_warning_shown and count == 200:
                            warning = detect_data_mode(recent_rows, args.mode)
                            if warning:
                                print(warning)
                                mode_warning_shown = True

                        if count % 50 == 0:
                            elapsed = time.time() - t0
                            print(f"[{count:5d}] {match.group(1):>9},{match.group(2):>9},{match.group(3):>9}  {elapsed:5.1f}s")
                except serial.SerialException as exc:
                    print(f"[WARN] Serial error: {exc}. Reconnecting in 1s...")
                    time.sleep(1.0)
                finally:
                    if ser is not None and ser.is_open:
                        ser.close()
    except KeyboardInterrupt:
        print("")

    elapsed = time.time() - t0
    print(f"[STOP] {count} samples in {elapsed:.1f}s")
    print(f"[FILE] {save_path}")

    if count < args.min_samples:
        print(f"[WARN] Fewer than {args.min_samples} samples. More coverage is recommended.")

    # Write metadata JSON
    meta = {
        "mode": args.mode,
        "c_function": info["c_function"],
        "data_field": info["data_field"],
        "port": args.port,
        "baud": args.baud,
        "timestamp": timestamp,
        "samples": count,
        "duration_s": round(elapsed, 1),
        "file": os.path.basename(save_path),
        "calibration_params": str(DEFAULT_HEADER_PATH),
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f"[META] {meta_path}")


if __name__ == "__main__":
    main()
