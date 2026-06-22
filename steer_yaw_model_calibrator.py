#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Front-steer yaw model calibration helper.

It reads or collects gyroyaw frames and compares gyro yaw with the bicycle
model yaw. The script unwraps angles, detects the best continuous turn window,
and reports the scale needed for the front steering model.
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
except ImportError:
    serial = None

try:
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial Unicode MS", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


RE_CONSOLE = re.compile(
    r"\[\s*(\d+)\]\s*"
    r"z=\s*([-+]?\d+(?:\.\d+)?)\S*\s+"
    r"rp=\s*([-+]?\d+(?:\.\d+)?)\S*\s+"
    r"model=\s*([-+]?\d+(?:\.\d+)?)\S*\s+"
    r"roll=\s*([-+]?\d+(?:\.\d+)?)\S*\s+"
    r"pitch=\s*([-+]?\d+(?:\.\d+)?)"
)
RE_GYROYAW = re.compile(r"gyroyaw:\s*(\d+)\s*,\s*(.*)")


def parse_float(value):
    if value is None or value == "":
        return None
    return float(value)


def parse_gyroyaw_line(line, sample_index=None, t_s=None):
    m = RE_GYROYAW.search(line)
    if not m:
        m = RE_CONSOLE.search(line)
        if not m:
            return None
        return {
            "t_s": t_s,
            "seq": int(m.group(1)),
            "z": float(m.group(2)),
            "rp": float(m.group(3)),
            "model": float(m.group(4)),
            "roll": float(m.group(5)),
            "pitch": float(m.group(6)),
            "steer": None,
            "v": None,
        }

    parts = [x.strip() for x in m.group(2).split(",")]
    try:
        values = [float(x) for x in parts]
    except ValueError:
        return None
    if len(values) < 5:
        return None
    return {
        "t_s": t_s,
        "seq": int(m.group(1)) if sample_index is None else sample_index,
        "z": values[0],
        "rp": values[1],
        "model": values[2],
        "roll": values[3],
        "pitch": values[4],
        "steer": values[5] if len(values) >= 6 else None,
        "v": values[6] if len(values) >= 7 else None,
    }


def load_rows(path):
    path = Path(path)
    text = path.read_text(encoding="utf-8", errors="ignore")
    rows = []

    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
            reader = csv.DictReader(f)
            for i, row in enumerate(reader):
                if "yaw_model_deg" not in row or row.get("yaw_model_deg", "") == "":
                    continue
                rows.append({
                    "t_s": parse_float(row.get("t_s")),
                    "seq": int(float(row.get("seq", i))),
                    "z": float(row["z_bias_deg"]),
                    "rp": float(row["rp_bias_deg"]),
                    "model": float(row["yaw_model_deg"]),
                    "roll": float(row["roll_deg"]),
                    "pitch": float(row["pitch_deg"]),
                    "steer": parse_float(row.get("steer_deg")),
                    "v": parse_float(row.get("v_mps")),
                })
        return rows

    for i, line in enumerate(text.splitlines()):
        row = parse_gyroyaw_line(line, sample_index=i)
        if row is not None:
            rows.append(row)
    return rows


def collect_rows(port, baud, duration, out_dir):
    if serial is None:
        raise RuntimeError("pyserial is not installed")
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = out_dir / f"steer_yaw_cal_{ts}_raw.txt"
    csv_path = out_dir / f"steer_yaw_cal_{ts}.csv"

    rows = []
    t0 = time.time()
    print(f"[PORT] {port} @ {baud}")
    print(f"[SAVE] {csv_path}")
    print("[ACTION] 先静止 3 秒，然后固定一个前轮转角，低速匀速跑完整 360 度，结束后再静止 3 秒。")
    print("[RECV] gyroyaw:seq,z_bias_deg,rp_bias_deg,yaw_model_deg,roll_deg,pitch_deg[,steer_deg,v_mps]")
    print("[STOP] Ctrl+C 可提前停止，停止后自动分析。")

    with serial.Serial(port, baud, timeout=1.0) as ser, \
            raw_path.open("w", encoding="utf-8") as rf:
        try:
            while True:
                if duration is not None and time.time() - t0 >= duration:
                    break
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                t_s = time.time() - t0
                rf.write(f"{t_s:.6f},{line}\n")
                row = parse_gyroyaw_line(line, t_s=t_s)
                if row is None:
                    continue
                rows.append(row)
                if len(rows) % 50 == 0:
                    print(
                        f"[{len(rows):5d}] z={row['z']:+8.2f} rp={row['rp']:+8.2f} "
                        f"model={row['model']:+8.2f} roll={row['roll']:+7.2f} pitch={row['pitch']:+7.2f}"
                    )
        except KeyboardInterrupt:
            pass

    save_csv(rows, csv_path)
    print(f"[FILE] {csv_path}")
    return rows, csv_path


def save_csv(rows, path):
    with Path(path).open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["i", "t_s", "seq", "z_bias_deg", "rp_bias_deg", "yaw_model_deg", "roll_deg", "pitch_deg", "steer_deg", "v_mps"])
        for i, r in enumerate(rows):
            writer.writerow([
                i,
                "" if r["t_s"] is None else f"{r['t_s']:.6f}",
                r["seq"],
                f"{r['z']:.6f}",
                f"{r['rp']:.6f}",
                f"{r['model']:.6f}",
                f"{r['roll']:.6f}",
                f"{r['pitch']:.6f}",
                "" if r["steer"] is None else f"{r['steer']:.6f}",
                "" if r["v"] is None else f"{r['v']:.6f}",
            ])


def unwrap_deg(values):
    if not values:
        return []
    out = [values[0]]
    offset = 0.0
    prev = values[0]
    for value in values[1:]:
        delta = value - prev
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        out.append(value + offset)
        prev = value
    return out


def median(values):
    values = sorted(values)
    n = len(values)
    if n == 0:
        return 0.0
    mid = n // 2
    return values[mid] if n % 2 else 0.5 * (values[mid - 1] + values[mid])


def mean(values):
    return sum(values) / len(values) if values else 0.0


def std(values):
    if not values:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((x - m) ** 2 for x in values) / len(values))


def detect_turn_window(rows):
    rp = [r["rp_u"] for r in rows]
    model = [r["model_u"] for r in rows]
    motion = []
    for i in range(1, len(rows)):
        d = max(abs(rp[i] - rp[i - 1]), abs(model[i] - model[i - 1]))
        if d >= 0.05:
            motion.append(i)
    if not motion:
        return 0, len(rows) - 1

    groups = []
    start = prev = motion[0]
    gap_limit = max(3, len(rows) // 200)
    for idx in motion[1:]:
        if idx <= prev + gap_limit:
            prev = idx
        else:
            groups.append((max(0, start - 1), prev))
            start = prev = idx
    groups.append((max(0, start - 1), prev))

    def score(g):
        a, b = g
        return abs(rows[b]["rp_u"] - rows[a]["rp_u"])

    return max(groups, key=score)


def linear_fit(x, y):
    n = len(x)
    if n < 2:
        return 0.0, 0.0, 0.0
    mx = mean(x)
    my = mean(y)
    den = sum((v - mx) ** 2 for v in x)
    if abs(den) < 1e-12:
        return 0.0, my, 0.0
    slope = sum((x[i] - mx) * (y[i] - my) for i in range(n)) / den
    intercept = my - slope * mx
    residuals = [y[i] - (slope * x[i] + intercept) for i in range(n)]
    return slope, intercept, math.sqrt(mean([r * r for r in residuals]))


def analyze(rows, wheelbase_m):
    if len(rows) < 20:
        raise RuntimeError("not enough gyroyaw rows")

    z_u = unwrap_deg([r["z"] for r in rows])
    rp_u = unwrap_deg([r["rp"] for r in rows])
    model_u = unwrap_deg([r["model"] for r in rows])
    for i, r in enumerate(rows):
        r["z_u"] = z_u[i]
        r["rp_u"] = rp_u[i]
        r["model_u"] = model_u[i]

    a, b = detect_turn_window(rows)
    win = rows[a:b + 1]
    z_delta = win[-1]["z_u"] - win[0]["z_u"]
    rp_delta = win[-1]["rp_u"] - win[0]["rp_u"]
    model_delta = win[-1]["model_u"] - win[0]["model_u"]
    ref_delta = rp_delta if abs(rp_delta) >= abs(z_delta) * 0.5 else z_delta

    x = [r["rp_u"] - win[0]["rp_u"] for r in win]
    y = [r["model_u"] - win[0]["model_u"] for r in win]
    fit_slope, fit_intercept, fit_rmse = linear_fit(x, y)

    scale_needed = ref_delta / model_delta if abs(model_delta) > 1e-9 else float("inf")
    model_ratio = model_delta / ref_delta if abs(ref_delta) > 1e-9 else 0.0
    effective_wheelbase = wheelbase_m * model_ratio
    wheelbase_for_match = wheelbase_m / scale_needed if scale_needed not in (0.0, float("inf")) else wheelbase_m

    roll_vals = [r["roll"] for r in win]
    pitch_vals = [r["pitch"] for r in win]
    steer_vals = [r["steer"] for r in win if r["steer"] is not None]
    speed_vals = [r["v"] for r in win if r["v"] is not None]

    return {
        "start": a,
        "end": b,
        "start_seq": win[0]["seq"],
        "end_seq": win[-1]["seq"],
        "n": len(win),
        "z_delta": z_delta,
        "rp_delta": rp_delta,
        "model_delta": model_delta,
        "ref_delta": ref_delta,
        "model_ratio": model_ratio,
        "scale_needed": scale_needed,
        "effective_wheelbase": effective_wheelbase,
        "wheelbase_for_match": wheelbase_for_match,
        "fit_slope": fit_slope,
        "fit_intercept": fit_intercept,
        "fit_rmse": fit_rmse,
        "roll_mean": mean(roll_vals),
        "roll_std": std(roll_vals),
        "pitch_mean": mean(pitch_vals),
        "pitch_std": std(pitch_vals),
        "steer_mean": mean(steer_vals) if steer_vals else None,
        "steer_std": std(steer_vals) if steer_vals else None,
        "speed_mean": mean(speed_vals) if speed_vals else None,
        "speed_std": std(speed_vals) if speed_vals else None,
    }


def plot(rows, result, out_path):
    if plt is None:
        return None
    out_path = Path(out_path)
    xs = list(range(len(rows)))
    a = result["start"]
    b = result["end"]
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(xs, [r["rp_u"] - rows[a]["rp_u"] for r in rows], label="陀螺yaw展开(rp)")
    axes[0].plot(xs, [r["model_u"] - rows[a]["model_u"] for r in rows], label="车辆模型yaw展开")
    axes[0].axvspan(a, b, color="tab:green", alpha=0.12, label="自动选取整圈窗口")
    axes[0].set_ylabel("角度/deg")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(xs, [((r["model_u"] - rows[a]["model_u"]) - (r["rp_u"] - rows[a]["rp_u"])) for r in rows], color="tab:red")
    axes[1].axvspan(a, b, color="tab:green", alpha=0.12)
    axes[1].set_ylabel("model-rp/deg")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(xs, [r["roll"] for r in rows], label="roll")
    axes[2].plot(xs, [r["pitch"] for r in rows], label="pitch")
    axes[2].axvspan(a, b, color="tab:green", alpha=0.12)
    axes[2].set_ylabel("姿态/deg")
    axes[2].set_xlabel("样本序号")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle("前轮转向车辆模型 yaw 标定分析")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def print_report(result, wheelbase_m, plot_path=None):
    print("\n" + "=" * 72)
    print("前轮转向 yaw 模型标定结果")
    print("=" * 72)
    print(f"自动窗口: samples[{result['start']}:{result['end'] + 1}], n={result['n']}, seq {result['start_seq']} -> {result['end_seq']}")
    print(f"陀螺 z yaw:      {result['z_delta']:+.2f} deg")
    print(f"RP补偿 yaw:      {result['rp_delta']:+.2f} deg  <- 主参考")
    print(f"车辆模型 yaw:    {result['model_delta']:+.2f} deg")
    print(f"模型/陀螺比例:   {result['model_ratio']:.5f}")
    print(f"建议模型放大:    {result['scale_needed']:.5f} 倍")
    print(f"线性拟合比例:    {result['fit_slope']:.5f}, 截距 {result['fit_intercept']:+.2f} deg, RMSE {result['fit_rmse']:.3f} deg")
    print(f"当前轴距:        {wheelbase_m:.4f} m")
    print(f"若只改等效轴距:  {result['wheelbase_for_match']:.4f} m")
    print(f"当前模型等效为:  {result['effective_wheelbase']:.4f} m 的转向效果")
    print(f"roll: mean={result['roll_mean']:+.2f} deg, std={result['roll_std']:.2f} deg")
    print(f"pitch: mean={result['pitch_mean']:+.2f} deg, std={result['pitch_std']:.2f} deg")
    if result["steer_mean"] is not None:
        print(f"steer: mean={result['steer_mean']:+.2f} deg, std={result['steer_std']:.2f} deg")
    if result["speed_mean"] is not None:
        print(f"v: mean={result['speed_mean']:+.3f} m/s, std={result['speed_std']:.3f} m/s")
    print("\n判断:")
    if result["model_ratio"] > 0:
        print("  - 符号正确：模型 yaw 与陀螺 yaw 同方向。")
    else:
        print("  - 符号错误：模型 yaw 与陀螺 yaw 反方向，需要先检查正负号。")
    if abs(result["scale_needed"] - 1.0) < 0.01:
        print("  - 比例很好，前轮模型误差小于 1%。")
    else:
        print("  - 比例有系统误差，优先用同一动作再采左/右各一圈确认，然后再改 C 参数。")
    print("\n本轮建议:")
    print(f"  - 暂定把车辆模型 yaw 乘以 {result['scale_needed']:.4f}，或把 INS 等效轴距从 {wheelbase_m:.3f}m 改到 {result['wheelbase_for_match']:.3f}m。")
    print("  - 如果下一轮左转、右转比例不同，不要改轴距，改左右前轮角度映射或加左右独立比例。")
    if plot_path:
        print(f"\n图表: {plot_path}")
    print("=" * 72 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Calibrate front-steer yaw model from gyroyaw data")
    parser.add_argument("--file", help="Read pasted console txt or gyroyaw csv")
    parser.add_argument("--port", default="COM24", help="Serial port for collection")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--out-dir", default=r"Seekfree_TC377_Opensource_Library\track_data")
    parser.add_argument("--wheelbase", type=float, default=0.78)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    source_path = None
    if args.file:
        rows = load_rows(args.file)
        source_path = Path(args.file)
    else:
        rows, source_path = collect_rows(args.port, args.baud, args.duration, args.out_dir)

    result = analyze(rows, args.wheelbase)
    plot_path = None
    if not args.no_plot:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_name = f"steer_yaw_cal_{ts}.png"
        if source_path is not None:
            plot_name = f"{source_path.stem}_steer_yaw_cal.png"
        plot_path = Path(args.out_dir) / plot_name
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        plot(rows, result, plot_path)
    print_report(result, args.wheelbase, plot_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise
