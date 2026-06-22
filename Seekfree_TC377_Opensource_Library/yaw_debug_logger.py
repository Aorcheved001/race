# -*- coding: utf-8 -*-
"""Collect and analyze yaw debug frames from firmware.

Expected firmware line:
  yawdbg:yaw_gyro_deg,yaw_mag_raw_deg,yaw_mag_rel_deg,yaw_fused_deg,mag_x,mag_y,mag_z
  yawrtk:seq,tick_ms,phase,yaw_gyro,mag_yaw_raw,mag_yaw_rel,mag_yaw_corr_rel,
         yaw_fused,rtk_yaw_ins,rtk_yaw_ref,rtk_valid,mag_x,mag_y,mag_z
"""

import argparse
import math
import re
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports


DATA_DIR = Path("track_data")
YAWDBG_RE = re.compile(
    r"yawdbg:\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)"
)
YAWRTK_RE = re.compile(
    r"yawrtk:\s*"
    r"(\d+)\s*,\s*"
    r"(\d+)\s*,\s*"
    r"([A-Za-z0-9_]+)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"(\d+)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)"
)


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        print(f"{port.device}: {port.description}")
    return ports


def choose_port(requested):
    if requested:
        return requested
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found.")
    for port in ports:
        desc = f"{port.description} {port.hwid}".lower()
        if "2e3c" in desc or "5740" in desc or "usb" in desc:
            return port.device
    return ports[0].device


def parse_file(path):
    rows = []
    modes = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = YAWRTK_RE.search(line)
            if m:
                rows.append([
                    float(m.group(4)), float(m.group(5)), float(m.group(6)),
                    float(m.group(7)), float(m.group(8)), float(m.group(9)),
                    float(m.group(10)), float(m.group(11)), float(m.group(12)),
                    float(m.group(13)), float(m.group(14)),
                ])
                modes.append(m.group(3))
                continue
            m = YAWDBG_RE.search(line)
            if m:
                # yaw_gyro, mag_raw, mag_rel, mag_corr_rel, yaw_fused,
                # rtk_ins, rtk_ref, rtk_valid, mag_x, mag_y, mag_z
                rows.append([
                    float(m.group(1)), float(m.group(2)), float(m.group(3)),
                    float(m.group(3)), float(m.group(4)), 0.0, 0.0, 0.0,
                    float(m.group(5)), float(m.group(6)), float(m.group(7)),
                ])
                modes.append("LEGACY")
    if not rows:
        raise ValueError("No yawdbg/yawrtk frames found.")
    return np.asarray(rows, dtype=float), modes


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def fit_circle_xy(xy):
    x = xy[:, 0]
    y = xy[:, 1]
    a = np.column_stack([2.0 * x, 2.0 * y, np.ones(len(xy))])
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    cx, cy, c = sol
    r = np.sqrt(np.maximum(0.0, c + cx * cx + cy * cy))
    rr = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    return np.array([cx, cy]), r, rr


def analyze(path):
    data, modes = parse_file(path)
    yaw_gyro = data[:, 0]
    yaw_mag_raw = data[:, 1]
    yaw_mag_rel = data[:, 2]
    yaw_mag_corr = data[:, 3]
    yaw_fused = data[:, 4]
    rtk_yaw_ref = data[:, 6]
    rtk_valid = data[:, 7].astype(int)
    mag = data[:, 8:11]

    diff_mag_fused = wrap_deg(yaw_mag_rel - yaw_fused)
    diff_mag_corr_fused = wrap_deg(yaw_mag_corr - yaw_fused)
    diff_gyro_fused = wrap_deg(yaw_gyro - yaw_fused)
    # Bit0=RTK position valid/aligned source, bit1=RTK yaw valid.
    # rtk_yaw_ref is meaningful for mag-yaw calibration only when both are set.
    valid_rtk = (rtk_valid & 0x03) == 0x03
    fused_unwrap = np.degrees(np.unwrap(np.radians(yaw_fused)))
    mag_rel_unwrap = np.degrees(np.unwrap(np.radians(yaw_mag_rel)))
    mag_corr_unwrap = np.degrees(np.unwrap(np.radians(yaw_mag_corr)))
    gyro_unwrap = np.degrees(np.unwrap(np.radians(yaw_gyro)))

    center, radius, rr = fit_circle_xy(mag[:, :2])
    print(f"File: {path}")
    print(f"Samples: {len(data)}")
    print(f"Modes: {', '.join(sorted(set(modes)))}")
    print("")
    print("Yaw span:")
    print(f"  gyro:    {gyro_unwrap.min():.1f} .. {gyro_unwrap.max():.1f}  span={gyro_unwrap.max() - gyro_unwrap.min():.1f} deg")
    print(f"  mag_rel: {mag_rel_unwrap.min():.1f} .. {mag_rel_unwrap.max():.1f}  span={mag_rel_unwrap.max() - mag_rel_unwrap.min():.1f} deg")
    print(f"  mag_cor: {mag_corr_unwrap.min():.1f} .. {mag_corr_unwrap.max():.1f}  span={mag_corr_unwrap.max() - mag_corr_unwrap.min():.1f} deg")
    print(f"  fused:   {fused_unwrap.min():.1f} .. {fused_unwrap.max():.1f}  span={fused_unwrap.max() - fused_unwrap.min():.1f} deg")
    print("")
    print("Yaw difference:")
    print(f"  mag_rel - fused: mean={diff_mag_fused.mean():.2f} deg, std={diff_mag_fused.std():.2f}, max_abs={np.max(np.abs(diff_mag_fused)):.2f}")
    print(f"  mag_cor - fused: mean={diff_mag_corr_fused.mean():.2f} deg, std={diff_mag_corr_fused.std():.2f}, max_abs={np.max(np.abs(diff_mag_corr_fused)):.2f}")
    print(f"  gyro - fused:    mean={diff_gyro_fused.mean():.2f} deg, std={diff_gyro_fused.std():.2f}, max_abs={np.max(np.abs(diff_gyro_fused)):.2f}")
    if np.any(valid_rtk):
        d_corr_rtk = wrap_deg(yaw_mag_corr[valid_rtk] - rtk_yaw_ref[valid_rtk])
        d_fused_rtk = wrap_deg(yaw_fused[valid_rtk] - rtk_yaw_ref[valid_rtk])
        print(f"  mag_cor - rtk:   mean={d_corr_rtk.mean():.2f} deg, std={d_corr_rtk.std():.2f}, max_abs={np.max(np.abs(d_corr_rtk)):.2f}")
        print(f"  fused - rtk:     mean={d_fused_rtk.mean():.2f} deg, std={d_fused_rtk.std():.2f}, max_abs={np.max(np.abs(d_fused_rtk)):.2f}")
    print("")
    print("Calibrated mag XY:")
    print(f"  center=({center[0]:.2f}, {center[1]:.2f}), offset={np.linalg.norm(center):.1f}")
    print(f"  radius={rr.mean():.1f} +/- {rr.std():.1f}, cv={rr.std() / rr.mean() * 100.0:.2f}%")
    print(f"  robust p1..p99={np.percentile(rr, 1):.1f} .. {np.percentile(rr, 99):.1f}")
    print(f"  z={mag[:,2].min():.1f} .. {mag[:,2].max():.1f}, span={mag[:,2].max() - mag[:,2].min():.1f}")


def collect(port, baud, duration):
    DATA_DIR.mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = DATA_DIR / f"yaw_debug_{ts}.txt"
    count = 0
    start = time.time()

    with serial.Serial(port, baudrate=baud, timeout=1.0) as ser, out.open("w", encoding="utf-8") as f:
        f.write("# yaw debug data\n")
        f.write("# format: yawdbg:yaw_gyro_deg,yaw_mag_raw_deg,yaw_mag_rel_deg,yaw_fused_deg,mag_x,mag_y,mag_z\n")
        f.write(f"# port: {port}\n")
        f.write(f"# baud: {baud}\n")
        f.write(f"# timestamp: {ts}\n")
        print(f"Collecting {port} @ {baud}. Output: {out}")
        try:
            while duration <= 0.0 or time.time() - start < duration:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue
                if YAWDBG_RE.search(raw) or YAWRTK_RE.search(raw):
                    f.write(raw + "\n")
                    count += 1
                    if count % 200 == 0:
                        print(f"samples={count}")
        except KeyboardInterrupt:
            pass

    print(f"Saved: {out} ({count} samples)")
    if count > 0:
        analyze(out)


def main():
    parser = argparse.ArgumentParser(description="Collect/analyze yawdbg serial frames.")
    parser.add_argument("--port", help="Serial port, for example COM24.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds, 0 means until Ctrl+C.")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--analyze", help="Analyze an existing yaw_debug_*.txt file.")
    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return
    if args.analyze:
        analyze(Path(args.analyze))
        return
    collect(choose_port(args.port), args.baud, args.duration)


if __name__ == "__main__":
    main()
