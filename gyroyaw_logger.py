#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Collect gyroyaw frames from firmware.

Supported lines:
  gyroyaw:seq,z_bias_deg,rp_bias_deg,roll_deg,pitch_deg
  gyroyaw:seq,z_bias_deg,rp_bias_deg,yaw_model_deg,roll_deg,pitch_deg
  gyroyaw:seq,z_bias_deg,rp_bias_deg,yaw_model_deg,roll_deg,pitch_deg,steer_deg,v_mps
"""

import argparse
import csv
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import serial


RE_GYROYAW = re.compile(r"gyroyaw:\s*(\d+)\s*,\s*(.*)")


def parse_gyroyaw(line):
    m = RE_GYROYAW.search(line)
    if not m:
        return None
    parts = [x.strip() for x in m.group(2).split(",")]
    try:
        values = [float(x) for x in parts]
    except ValueError:
        return None

    seq = int(m.group(1))
    if len(values) == 4:
        z_bias, rp_bias, roll, pitch = values
        return seq, z_bias, rp_bias, "", roll, pitch, "", ""
    if len(values) >= 5:
        z_bias, rp_bias, yaw_model, roll, pitch = values[:5]
        steer = values[5] if len(values) >= 6 else ""
        speed = values[6] if len(values) >= 7 else ""
        return seq, z_bias, rp_bias, yaw_model, roll, pitch, steer, speed
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM24")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out-dir", default=r"Seekfree_TC377_Opensource_Library\track_data")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"gyroyaw_{ts}.csv"
    raw_path = out_dir / f"gyroyaw_{ts}_raw.txt"

    print(f"[PORT] {args.port} @ {args.baud}", flush=True)
    print(f"[SAVE] {csv_path}", flush=True)
    print("[RECV] gyroyaw:seq,z_bias_deg,rp_bias_deg,yaw_model_deg,roll_deg,pitch_deg[,steer_deg,v_mps]", flush=True)
    print("[RECV] Press Ctrl+C to stop.", flush=True)

    count = 0
    raw_count = 0
    t0 = time.time()
    last_seq = None
    lost = 0

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser, \
            csv_path.open("w", encoding="utf-8", newline="") as cf, \
            raw_path.open("w", encoding="utf-8") as rf:
        writer = csv.writer(cf)
        writer.writerow(["t_s", "seq", "z_bias_deg", "rp_bias_deg", "yaw_model_deg", "roll_deg", "pitch_deg", "steer_deg", "v_mps"])
        try:
            while True:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                now = time.time() - t0
                rf.write(f"{now:.6f},{line}\n")
                raw_count += 1
                parsed = parse_gyroyaw(line)
                if parsed is None:
                    continue
                seq, z_bias, rp_bias, yaw_model, roll, pitch, steer, speed = parsed
                if last_seq is not None:
                    step = (seq - last_seq) & 0xFFFF
                    if step > 1:
                        lost += step - 1
                last_seq = seq
                writer.writerow([
                    f"{now:.6f}",
                    seq,
                    f"{z_bias:.4f}",
                    f"{rp_bias:.4f}",
                    "" if yaw_model == "" else f"{yaw_model:.4f}",
                    f"{roll:.4f}",
                    f"{pitch:.4f}",
                    "" if steer == "" else f"{steer:.4f}",
                    "" if speed == "" else f"{speed:.4f}",
                ])
                count += 1
                if count % 50 == 0:
                    model_text = "model=   n/a " if yaw_model == "" else f"model={yaw_model:+8.2f} "
                    steer_text = "" if steer == "" else f"steer={steer:+7.2f} "
                    speed_text = "" if speed == "" else f"v={speed:+6.3f} "
                    print(
                        f"[{count}] z={z_bias:+8.2f} rp={rp_bias:+8.2f} "
                        f"{model_text}roll={roll:+7.2f} pitch={pitch:+7.2f} "
                        f"{steer_text}{speed_text}lost={lost}",
                        flush=True,
                    )
        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.time() - t0
            print(f"[STOP] rows={count}, raw={raw_count}, lost={lost}, elapsed={elapsed:.1f}s", flush=True)
            print(f"[FILE] {csv_path}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr, flush=True)
        raise
