#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Collect and analyze steering PID remote-control step tests.

Firmware line:
  steerpid:seq,target,current,error,derivative,pwm,raw,diff,gyro_z,
           diff_delta,diff_ff,omega_target,omega_actual,v,tick_l,tick_r
"""

import argparse
import csv
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


RE_STEERPID = re.compile(r"steerpid:\s*(\d+)\s*,\s*([^\r\n\x00]*)")
RE_STEERPID_FULL = re.compile(
    r"steerpid:\s*(\d+)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+)\s*,\s*"
    r"([-+]?\d+)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*,\s*"
    r"([-+]?\d+)\s*,\s*"
    r"([-+]?\d+)"
)
FIELDS = [
    "seq", "target_deg", "current_deg", "error_deg", "derivative_deg_s",
    "pwm", "raw", "diff", "gyro_z_rad_s", "diff_delta", "diff_ff",
    "omega_target_rad_s", "omega_actual_rad_s", "v_mps", "tick_l", "tick_r",
]


def parse_line(line, t_s=None):
    m = RE_STEERPID.search(line)
    if not m:
        return None
    parts = [x.strip() for x in m.group(2).split(",")]
    if len(parts) < 15:
        return None
    try:
        values = [float(x) for x in parts[:15]]
    except ValueError:
        return None
    row = {
        "t_s": t_s,
        "seq": int(m.group(1)),
        "target_deg": values[0],
        "current_deg": values[1],
        "error_deg": values[2],
        "derivative_deg_s": values[3],
        "pwm": values[4],
        "raw": int(values[5]),
        "diff": int(values[6]),
        "gyro_z_rad_s": values[7],
        "diff_delta": values[8],
        "diff_ff": values[9],
        "omega_target_rad_s": values[10],
        "omega_actual_rad_s": values[11],
        "v_mps": values[12],
        "tick_l": int(values[13]),
        "tick_r": int(values[14]),
    }
    return row


def load_file(path):
    path = Path(path)
    rows = []
    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
            reader = csv.DictReader(f)
            for r in reader:
                row = {}
                for k in FIELDS:
                    if k in {"seq", "raw", "diff", "tick_l", "tick_r"}:
                        row[k] = int(float(r[k]))
                    elif k == "t_s":
                        row[k] = float(r[k]) if r[k] else None
                    else:
                        row[k] = float(r[k])
                row["t_s"] = float(r["t_s"]) if r.get("t_s") else None
                rows.append(row)
        return rows

    text = path.read_text(encoding="utf-8", errors="ignore")
    for m in RE_STEERPID_FULL.finditer(text):
        values = [float(m.group(i)) for i in range(2, 17)]
        rows.append({
            "t_s": None,
            "seq": int(m.group(1)),
            "target_deg": values[0],
            "current_deg": values[1],
            "error_deg": values[2],
            "derivative_deg_s": values[3],
            "pwm": values[4],
            "raw": int(values[5]),
            "diff": int(values[6]),
            "gyro_z_rad_s": values[7],
            "diff_delta": values[8],
            "diff_ff": values[9],
            "omega_target_rad_s": values[10],
            "omega_actual_rad_s": values[11],
            "v_mps": values[12],
            "tick_l": int(values[13]),
            "tick_r": int(values[14]),
        })
    return rows


def save_csv(rows, path):
    with Path(path).open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["t_s"] + FIELDS)
        writer.writeheader()
        for r in rows:
            out = dict(r)
            out["t_s"] = "" if r["t_s"] is None else f"{r['t_s']:.6f}"
            writer.writerow(out)


def collect(port, baud, duration, out_dir):
    if serial is None:
        raise RuntimeError("pyserial not installed. Run: pip install pyserial")
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = out_dir / f"steerpid_{ts}_raw.txt"
    csv_path = out_dir / f"steerpid_{ts}.csv"

    print(f"[PORT] {port} @ {baud}")
    print(f"[SAVE] {csv_path}")
    print("[ACTION] 先回中静止2秒，再依次做：右转保持2秒、回中2秒、左转保持2秒、回中2秒。")
    print("[RECV] steerpid:seq,target,current,error,derivative,pwm,raw,diff,gyro_z,diff_delta,diff_ff,omega_target,omega_actual,v,tick_l,tick_r")
    print("[STOP] Ctrl+C 停止后自动分析。")

    rows = []
    t0 = time.time()
    try:
        ser_ctx = serial.Serial(port, baud, timeout=1.0)
    except Exception as exc:
        print(f"[ERROR] 无法打开 {port}: {exc}")
        if list_ports is not None:
            ports = list(list_ports.comports())
            if ports:
                print("[PORTS] 当前可用串口：")
                for p in ports:
                    print(f"  {p.device}: {p.description}")
            else:
                print("[PORTS] 当前系统没有枚举到任何串口。检查USB/无线串口供电、驱动、端口号。")
        raise

    with ser_ctx as ser, raw_path.open("w", encoding="utf-8") as rf:
        try:
            while True:
                if duration is not None and time.time() - t0 >= duration:
                    break
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                t_s = time.time() - t0
                rf.write(f"{t_s:.6f},{line}\n")
                row = parse_line(line, t_s=t_s)
                if row is None:
                    continue
                rows.append(row)
                if len(rows) % 10 == 0:
                    print(
                        f"[{len(rows):4d}] tgt={row['target_deg']:+6.2f} cur={row['current_deg']:+6.2f} "
                        f"err={row['error_deg']:+6.2f} pwm={row['pwm']:+6.0f} raw={row['raw']:4d} "
                        f"diff={row['diff']:+5d}"
                    )
        except KeyboardInterrupt:
            pass

    save_csv(rows, csv_path)
    print(f"[FILE] {csv_path}")
    return rows, csv_path


def mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def std(vals):
    if not vals:
        return 0.0
    m = mean(vals)
    return math.sqrt(sum((x - m) ** 2 for x in vals) / len(vals))


def detect_steps(rows, target_threshold=1.0):
    if len(rows) < 10:
        return []
    segments = []
    start = 0
    last_target = rows[0]["target_deg"]
    for i in range(1, len(rows)):
        target = rows[i]["target_deg"]
        if abs(target - last_target) >= target_threshold:
            if i - start >= 3:
                segments.append((start, i - 1))
            start = i
            last_target = target
    if len(rows) - start >= 3:
        segments.append((start, len(rows) - 1))

    merged = []
    for a, b in segments:
        tgt = mean([r["target_deg"] for r in rows[a:b + 1]])
        if abs(tgt) < 0.5 and b - a < 5:
            continue
        merged.append((a, b))
    return merged


def analyze_segment(rows, a, b):
    seg = rows[a:b + 1]
    t0 = seg[0]["t_s"] if seg[0]["t_s"] is not None else 0.04 * a
    times = [(r["t_s"] if r["t_s"] is not None else 0.04 * (a + i)) - t0 for i, r in enumerate(seg)]
    target = mean([r["target_deg"] for r in seg[:max(1, min(5, len(seg)))]])
    final_part = seg[max(0, int(len(seg) * 0.7)):]
    final_current = mean([r["current_deg"] for r in final_part])
    final_error = mean([r["error_deg"] for r in final_part])
    final_error_std = std([r["error_deg"] for r in final_part])
    abs_target = abs(target)

    if target >= 0:
        peak = max(r["current_deg"] for r in seg)
        overshoot = max(0.0, peak - target)
    else:
        peak = min(r["current_deg"] for r in seg)
        overshoot = max(0.0, target - peak)

    rise_time = None
    settle_time = None
    if abs_target >= 2.0:
        start_current = seg[0]["current_deg"]
        low = start_current + 0.1 * (target - start_current)
        high = start_current + 0.9 * (target - start_current)
        t10 = None
        t90 = None
        for t, r in zip(times, seg):
            cur = r["current_deg"]
            if t10 is None and ((target >= start_current and cur >= low) or (target < start_current and cur <= low)):
                t10 = t
            if t90 is None and ((target >= start_current and cur >= high) or (target < start_current and cur <= high)):
                t90 = t
                break
        if t10 is not None and t90 is not None:
            rise_time = max(0.0, t90 - t10)

        tol = max(0.8, abs_target * 0.05)
        for i, (t, r) in enumerate(zip(times, seg)):
            if all(abs(x["error_deg"]) <= tol for x in seg[i:]):
                settle_time = t
                break

    pwm_vals = [r["pwm"] for r in seg]
    pwm_sat_ratio = sum(1 for x in pwm_vals if abs(x) >= 7400) / len(pwm_vals)
    reached = abs(final_error) <= max(0.8, abs_target * 0.05)

    return {
        "a": a,
        "b": b,
        "n": len(seg),
        "duration_s": times[-1] if times else 0.0,
        "target_deg": target,
        "final_current_deg": final_current,
        "final_error_deg": final_error,
        "final_error_std": final_error_std,
        "overshoot_deg": overshoot,
        "rise_time_s": rise_time,
        "settle_time_s": settle_time,
        "pwm_max": max(abs(x) for x in pwm_vals),
        "pwm_sat_ratio": pwm_sat_ratio,
        "raw_min": min(r["raw"] for r in seg),
        "raw_max": max(r["raw"] for r in seg),
        "diff_min": min(r["diff"] for r in seg),
        "diff_max": max(r["diff"] for r in seg),
        "reached": reached,
    }


def analyze(rows):
    segments = detect_steps(rows)
    return [analyze_segment(rows, a, b) for a, b in segments]


def print_report(results):
    print("\n" + "=" * 86)
    print("遥控转向 PID 阶跃分析")
    print("=" * 86)
    if not results:
        print("没有检测到目标角阶跃。请确认固件发送 steerpid，并且遥控目标角有变化。")
        return
    for i, r in enumerate(results, 1):
        direction = "右转" if r["target_deg"] > 0.5 else ("左转" if r["target_deg"] < -0.5 else "回中")
        rise = "n/a" if r["rise_time_s"] is None else f"{r['rise_time_s']:.2f}s"
        settle = "n/a" if r["settle_time_s"] is None else f"{r['settle_time_s']:.2f}s"
        status = "到位" if r["reached"] else "未到位"
        print(
            f"[{i}] {direction} target={r['target_deg']:+.2f} deg, duration={r['duration_s']:.2f}s, "
            f"final={r['final_current_deg']:+.2f} deg, steady_err={r['final_error_deg']:+.2f}±{r['final_error_std']:.2f} deg, {status}"
        )
        print(
            f"    rise={rise}, settle={settle}, overshoot={r['overshoot_deg']:.2f} deg, "
            f"pwm_max={r['pwm_max']:.0f}, pwm_sat={r['pwm_sat_ratio']*100:.1f}%, "
            f"raw={r['raw_min']}..{r['raw_max']}, diff={r['diff_min']}..{r['diff_max']}"
        )
        if not r["reached"] and r["pwm_sat_ratio"] > 0.2:
            print("    判断: PWM长时间打满仍不到位，优先查机械卡滞/供电/驱动能力，或降低目标/提高限幅。")
        elif not r["reached"]:
            print("    判断: 没到位但PWM未明显打满，优先调大KP或检查目标角是否被遥控死区/限幅吃掉。")
        elif r["overshoot_deg"] > max(1.0, abs(r["target_deg"]) * 0.08):
            print("    判断: 有明显超调，优先增大KD或降低KP。")
    print("=" * 86 + "\n")


def plot(rows, results, out_path):
    if plt is None or not rows:
        return None
    xs = [r["t_s"] if r["t_s"] is not None else 0.04 * i for i, r in enumerate(rows)]
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(xs, [r["target_deg"] for r in rows], label="目标角")
    axes[0].plot(xs, [r["current_deg"] for r in rows], label="实际角")
    axes[0].set_ylabel("转向角/deg")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(xs, [r["error_deg"] for r in rows], label="误差")
    axes[1].plot(xs, [r["pwm"] / 1000.0 for r in rows], label="PWM/1000")
    axes[1].set_ylabel("误差/PWM")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(xs, [r["gyro_z_rad_s"] for r in rows], label="gyro_z")
    axes[2].plot(xs, [r["omega_target_rad_s"] for r in rows], label="omega_target")
    axes[2].plot(xs, [r["omega_actual_rad_s"] for r in rows], label="omega_actual")
    axes[2].set_ylabel("角速度/rad/s")
    axes[2].set_xlabel("时间/s")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    for res in results:
        x0 = xs[res["a"]]
        x1 = xs[res["b"]]
        for ax in axes:
            ax.axvspan(x0, x1, color="tab:green", alpha=0.08)
    fig.suptitle("遥控转向 PID 阶跃响应")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main():
    parser = argparse.ArgumentParser(description="Steering PID remote step analyzer")
    parser.add_argument("--file", help="Read existing steerpid txt/csv")
    parser.add_argument("--port", default="COM24")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--out-dir", default=r"Seekfree_TC377_Opensource_Library\track_data")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--rebuild-csv", action="store_true", help="When reading raw txt, save recovered steerpid csv next to it")
    args = parser.parse_args()

    if args.file:
        rows = load_file(args.file)
        source = Path(args.file)
        if args.rebuild_csv:
            rebuilt = source.with_name(source.stem.replace("_raw", "") + "_recovered.csv")
            save_csv(rows, rebuilt)
            print(f"[RECOVERED] {rebuilt}")
    else:
        rows, source = collect(args.port, args.baud, args.duration, args.out_dir)

    results = analyze(rows)
    print_report(results)

    if not args.no_plot:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        plot_path = out_dir / f"{source.stem}_plot.png"
        if plot(rows, results, plot_path):
            print(f"[PLOT] {plot_path}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise
