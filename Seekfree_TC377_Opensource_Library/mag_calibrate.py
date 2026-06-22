# -*- coding: utf-8 -*-
"""Offline magnetometer calibration for the car's raw IMU963RA mag data.

Accepted input formats:
  x,y,z
  [timestamp] x,y,z

Firmware applies the generated matrix as:
  corrected = soft_iron * (raw - hard_iron)

For a full-size vehicle, use the default 2D mode. It fits only the horizontal
XY ellipse from normal driving data and preserves the existing Z parameters.
"""

import argparse
import re
from pathlib import Path

import numpy as np


DEFAULT_HEADER = Path("code/ins/calibration_params.h")


def load_mag_data(path):
    rows = []
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    csv_re = re.compile(rf"^\s*(?:\[[^\]]*\]\s*)?({number})\s*,\s*({number})\s*,\s*({number})\s*$")
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            match = csv_re.match(line)
            if match:
                rows.append([float(match.group(1)), float(match.group(2)), float(match.group(3))])
    if not rows:
        raise ValueError("No magnetometer samples found. Expected lines like: x,y,z")
    return np.asarray(rows, dtype=float)


def _read_define(text, name):
    match = re.search(rf"#define\s+{re.escape(name)}\s+([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)f?", text)
    if not match:
        raise ValueError(f"Missing define: {name}")
    return float(match.group(1))


def load_header(path):
    text = Path(path).read_text(encoding="utf-8")
    bias = np.array([
        _read_define(text, "MAG_HARD_IRON_X"),
        _read_define(text, "MAG_HARD_IRON_Y"),
        _read_define(text, "MAG_HARD_IRON_Z"),
    ], dtype=float)
    soft_iron = np.array([
        [_read_define(text, "MAG_SOFT_IRON_XX"), _read_define(text, "MAG_SOFT_IRON_XY"), _read_define(text, "MAG_SOFT_IRON_XZ")],
        [_read_define(text, "MAG_SOFT_IRON_YX"), _read_define(text, "MAG_SOFT_IRON_YY"), _read_define(text, "MAG_SOFT_IRON_YZ")],
        [_read_define(text, "MAG_SOFT_IRON_ZX"), _read_define(text, "MAG_SOFT_IRON_ZY"), _read_define(text, "MAG_SOFT_IRON_ZZ")],
    ], dtype=float)
    field = _read_define(text, "MAG_FIELD_STRENGTH")
    return bias, soft_iron, field


def filter_valid_samples(data):
    return data[np.linalg.norm(data, axis=1) > 50.0]


def fit_ellipse_2d(data):
    x, y = data[:, 0], data[:, 1]
    design = np.column_stack([x * x, 2.0 * x * y, y * y, 2.0 * x, 2.0 * y])
    coeff, *_ = np.linalg.lstsq(design, np.ones(len(data)), rcond=None)
    q = np.array([[coeff[0], coeff[1]], [coeff[1], coeff[2]]], dtype=float)
    u = coeff[3:5]

    center = -np.linalg.solve(q, u)
    rhs = 1.0 - center @ u
    if rhs <= 0.0:
        q = -q
        u = -u
        center = -np.linalg.solve(q, u)
        rhs = 1.0 - center @ u
    if rhs <= 0.0:
        raise ValueError("Invalid ellipse fit: translated radius term is not positive")

    q_norm = 0.5 * (q / rhs + (q / rhs).T)
    eigvals, eigvecs = np.linalg.eigh(q_norm)
    if np.any(eigvals <= 0.0):
        raise ValueError(f"Invalid ellipse fit: non-positive eigenvalues {eigvals}")

    radii = 1.0 / np.sqrt(eigvals)
    field_strength = float(np.mean(radii))
    soft_xy = eigvecs @ np.diag(field_strength / radii) @ eigvecs.T
    soft_xy = 0.5 * (soft_xy + soft_xy.T)
    return center, soft_xy, field_strength, radii


def build_2d_calibration(data, preserve_header):
    old_bias, old_soft, old_field = load_header(preserve_header)
    center_xy, soft_xy, field_strength, radii = fit_ellipse_2d(data)

    bias = old_bias.copy()
    bias[0:2] = center_xy

    soft_iron = np.zeros((3, 3), dtype=float)
    soft_iron[0:2, 0:2] = soft_xy
    soft_iron[2, 2] = old_soft[2, 2] if abs(old_soft[2, 2]) > 1e-6 else 1.0

    return bias, soft_iron, field_strength if field_strength > 0.0 else old_field, radii


def fit_ellipsoid(data):
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    design = np.column_stack([
        x * x, y * y, z * z,
        2.0 * x * y, 2.0 * x * z, 2.0 * y * z,
        2.0 * x, 2.0 * y, 2.0 * z,
    ])
    coeff, *_ = np.linalg.lstsq(design, np.ones(len(data)), rcond=None)
    q = np.array([
        [coeff[0], coeff[3], coeff[4]],
        [coeff[3], coeff[1], coeff[5]],
        [coeff[4], coeff[5], coeff[2]],
    ], dtype=float)
    u = coeff[6:9]

    bias = -np.linalg.solve(q, u)
    rhs = 1.0 - bias @ u
    if rhs <= 0.0:
        q = -q
        u = -u
        bias = -np.linalg.solve(q, u)
        rhs = 1.0 - bias @ u
    if rhs <= 0.0:
        raise ValueError("Invalid ellipsoid fit: translated radius term is not positive")

    q_norm = 0.5 * (q / rhs + (q / rhs).T)
    eigvals, eigvecs = np.linalg.eigh(q_norm)
    if np.any(eigvals <= 0.0):
        raise ValueError(f"Invalid ellipsoid fit: non-positive eigenvalues {eigvals}")

    radii = 1.0 / np.sqrt(eigvals)
    field_strength = float(np.mean(radii))
    soft_iron = eigvecs @ np.diag(field_strength / radii) @ eigvecs.T
    soft_iron = 0.5 * (soft_iron + soft_iron.T)
    return bias, soft_iron, field_strength, radii


def apply_calibration(data, bias, soft_iron):
    return (data - bias) @ soft_iron.T


def quality_report(data, corrected):
    norms = np.linalg.norm(corrected, axis=1)
    residual = float(np.std(norms) / np.mean(norms) * 100.0)
    ratio = float(np.max(norms) / np.min(norms))

    centered = data - np.mean(data, axis=0)
    raw_norms = np.linalg.norm(centered, axis=1)
    elevation = np.degrees(np.arcsin(np.clip(centered[:, 2] / (raw_norms + 1e-12), -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(corrected[:, 1], corrected[:, 0]))
    yaw_hist, _ = np.histogram(yaw, bins=np.linspace(-180.0, 180.0, 37))
    yaw_coverage = float(np.sum(yaw_hist > 0) / 36.0 * 100.0)

    score = 100.0
    score -= min(50.0, residual * 8.0)
    score -= min(25.0, abs(ratio - 1.0) * 80.0)
    score -= max(0.0, 85.0 - yaw_coverage) * 0.3
    score = max(0.0, min(100.0, score))
    return {
        "norm_mean": float(np.mean(norms)),
        "norm_std": float(np.std(norms)),
        "residual_percent": residual,
        "max_min_ratio": ratio,
        "yaw_coverage_percent": yaw_coverage,
        "elevation_min": float(np.min(elevation)),
        "elevation_max": float(np.max(elevation)),
        "score": score,
    }


def quality_report_2d(data, corrected):
    xy = corrected[:, :2]
    radii = np.linalg.norm(xy, axis=1)
    residual = float(np.std(radii) / np.mean(radii) * 100.0)
    ratio = float(np.max(radii) / np.min(radii))
    yaw_metrics = yaw_coverage_metrics_2d(xy)
    yaw_coverage = yaw_metrics["yaw_coverage_percent"]
    score = 100.0
    score -= min(55.0, residual * 10.0)
    score -= min(30.0, abs(ratio - 1.0) * 100.0)
    score -= max(0.0, 90.0 - yaw_coverage) * 0.4
    score -= max(0.0, yaw_metrics["yaw_largest_gap_deg"] - 30.0) * 0.45
    score -= max(0.0, 330.0 - yaw_metrics["yaw_circular_span_deg"]) * 0.25
    score = max(0.0, min(100.0, score))
    report = {
        "norm_mean": float(np.mean(radii)),
        "norm_std": float(np.std(radii)),
        "residual_percent": residual,
        "max_min_ratio": ratio,
        "elevation_min": 0.0,
        "elevation_max": 0.0,
        "score": score,
    }
    report.update(yaw_metrics)
    return report


def yaw_coverage_metrics_2d(xy, bins=36):
    yaw = np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))
    hist, _ = np.histogram(yaw, bins=np.linspace(-180.0, 180.0, bins + 1))
    occupied = int(np.sum(hist > 0))

    wrapped = np.sort(np.mod(yaw, 360.0))
    if len(wrapped) > 1:
        gaps = np.diff(np.concatenate([wrapped, [wrapped[0] + 360.0]]))
        largest_gap = float(np.max(gaps))
    else:
        largest_gap = 360.0

    unwrapped = np.degrees(np.unwrap(np.radians(yaw)))
    if len(unwrapped) > 1:
        unwrapped_span = float(np.max(unwrapped) - np.min(unwrapped))
        net_turn = float(abs(unwrapped[-1] - unwrapped[0]))
        total_turn = float(np.sum(np.abs(np.diff(unwrapped))))
    else:
        unwrapped_span = 0.0
        net_turn = 0.0
        total_turn = 0.0

    nonzero = hist[hist > 0]
    balance = float(nonzero.min() / nonzero.max()) if len(nonzero) else 0.0
    return {
        "yaw_coverage_percent": float(occupied / bins * 100.0),
        "yaw_bins_occupied": occupied,
        "yaw_bin_count": bins,
        "yaw_bin_min": int(hist.min()) if len(hist) else 0,
        "yaw_bin_max": int(hist.max()) if len(hist) else 0,
        "yaw_bin_balance": balance,
        "yaw_largest_gap_deg": largest_gap,
        "yaw_circular_span_deg": float(max(0.0, 360.0 - largest_gap)),
        "yaw_unwrapped_span_deg": unwrapped_span,
        "yaw_net_turn_deg": net_turn,
        "yaw_total_turn_deg": total_turn,
    }


def yaw_coverage_2d(data):
    xy = data[:, :2]
    centered = xy - np.mean(xy, axis=0)
    yaw = np.degrees(np.arctan2(centered[:, 1], centered[:, 0]))
    hist, _ = np.histogram(yaw, bins=np.linspace(-180.0, 180.0, 37))
    return float(np.sum(hist > 0) / 36.0 * 100.0), hist


def calibrate_2d_window(data, preserve_header):
    bias, soft_iron, field_strength, radii = build_2d_calibration(data, preserve_header)
    corrected = apply_calibration(data, bias, soft_iron)
    report = quality_report_2d(data, corrected)
    return bias, soft_iron, field_strength, radii, report


def find_best_2d_window(data, preserve_header, min_samples=2000, windows=None, stride_fraction=0.20):
    if windows is None:
        windows = [3000, 5000, 8000, 12000, 16000, 24000]

    candidates = []
    n = len(data)
    for size in windows:
        if size < min_samples or size > n:
            continue
        stride = max(200, int(size * stride_fraction))
        starts = list(range(0, n - size + 1, stride))
        if starts[-1] != n - size:
            starts.append(n - size)

        for start in starts:
            end = start + size
            window = data[start:end]
            try:
                bias, soft_iron, field_strength, radii, report = calibrate_2d_window(window, preserve_header)
            except (np.linalg.LinAlgError, ValueError, FloatingPointError):
                continue

            if report["yaw_bins_occupied"] < 34:
                continue
            if report["yaw_largest_gap_deg"] > 30.0:
                continue
            if report["yaw_circular_span_deg"] < 330.0:
                continue
            if report["yaw_unwrapped_span_deg"] < 330.0:
                continue
            if report["residual_percent"] > 3.0:
                continue
            if report["max_min_ratio"] > 1.15:
                continue

            score = report["score"]
            score += min(10.0, report["yaw_coverage_percent"] / 10.0)
            score += min(8.0, report["yaw_circular_span_deg"] / 45.0)
            score += min(4.0, report["yaw_bin_balance"] * 8.0)
            score -= max(0.0, report["max_min_ratio"] - 1.08) * 80.0
            score -= max(0.0, report["residual_percent"] - 1.8) * 8.0
            score += min(5.0, size / 5000.0)
            candidates.append({
                "score": score,
                "start": start,
                "end": end,
                "size": size,
                "bias": bias,
                "soft_iron": soft_iron,
                "field_strength": field_strength,
                "radii": radii,
                "report": report,
            })

    if not candidates:
        raise ValueError("No usable 2D calibration window found. Try a cleaner full-circle push run.")

    candidates.sort(key=lambda item: item["score"], reverse=True)
    return candidates[0], candidates[:8]


def header_text(bias, soft_iron, field_strength, sample_count, report, mode):
    return f"""/* Magnetometer calibration params - generated by mag_calibrate.py */
/* Mode: {mode.upper()}, Samples: {sample_count}, Quality: {report['score']:.1f}/100 */

#define MAG_HARD_IRON_X  {bias[0]:.6f}f
#define MAG_HARD_IRON_Y  {bias[1]:.6f}f
#define MAG_HARD_IRON_Z  {bias[2]:.6f}f

#define MAG_SOFT_IRON_XX {soft_iron[0, 0]:.6f}f
#define MAG_SOFT_IRON_XY {soft_iron[0, 1]:.6f}f
#define MAG_SOFT_IRON_XZ {soft_iron[0, 2]:.6f}f
#define MAG_SOFT_IRON_YX {soft_iron[1, 0]:.6f}f
#define MAG_SOFT_IRON_YY {soft_iron[1, 1]:.6f}f
#define MAG_SOFT_IRON_YZ {soft_iron[1, 2]:.6f}f
#define MAG_SOFT_IRON_ZX {soft_iron[2, 0]:.6f}f
#define MAG_SOFT_IRON_ZY {soft_iron[2, 1]:.6f}f
#define MAG_SOFT_IRON_ZZ {soft_iron[2, 2]:.6f}f

#define MAG_FIELD_STRENGTH {field_strength:.6f}f
"""


def print_report(data, bias, soft_iron, field_strength, radii, report, mode):
    print(f"Data points: {len(data)}")
    print(f"Raw X: {data[:,0].min():.1f} .. {data[:,0].max():.1f}")
    print(f"Raw Y: {data[:,1].min():.1f} .. {data[:,1].max():.1f}")
    print(f"Raw Z: {data[:,2].min():.1f} .. {data[:,2].max():.1f}")
    print("")
    print(f"Calibration result ({mode.upper()}):")
    print(f"  Hard iron: X={bias[0]:.3f}, Y={bias[1]:.3f}, Z={bias[2]:.3f}")
    print("  Z parameters are preserved in 2D mode." if mode == "2d" else f"  Ellipsoid radii: {radii[0]:.3f}, {radii[1]:.3f}, {radii[2]:.3f}")
    if mode == "2d":
        print(f"  XY ellipse radii: {radii[0]:.3f}, {radii[1]:.3f}")
    print(f"  Field strength: {field_strength:.3f}")
    print("  Soft iron:")
    for row in soft_iron:
        print(f"    {row[0]: .6f}, {row[1]: .6f}, {row[2]: .6f}")
    print("")
    print("Quality:")
    print(f"  Residual: {report['residual_percent']:.3f}%")
    print(f"  Max/min radius ratio: {report['max_min_ratio']:.4f}")
    print(f"  Yaw coverage: {report['yaw_coverage_percent']:.1f}%")
    if mode == "2d" and "yaw_largest_gap_deg" in report:
        print(f"  Yaw bins: {report['yaw_bins_occupied']}/{report['yaw_bin_count']}")
        print(f"  Largest yaw gap: {report['yaw_largest_gap_deg']:.1f} deg")
        print(f"  Circular yaw span: {report['yaw_circular_span_deg']:.1f} deg")
        print(f"  Unwrapped yaw span: {report['yaw_unwrapped_span_deg']:.1f} deg")
    if mode != "2d":
        print(f"  Elevation range: {report['elevation_min']:.1f} .. {report['elevation_max']:.1f} deg")
    print(f"  Score: {report['score']:.1f}/100")


def main():
    parser = argparse.ArgumentParser(description="Fit magnetometer hard/soft iron calibration.")
    parser.add_argument("logfile", help="Magnetometer log file, one raw x,y,z sample per line.")
    parser.add_argument("--mode", choices=["2d", "3d"], default="2d", help="2d is recommended for full-size vehicle driving data.")
    parser.add_argument("--preserve-header", default=str(DEFAULT_HEADER), help="Header used to preserve Z params in 2D mode.")
    parser.add_argument("--best-window", action="store_true", help="Find the best continuous 2D calibration window.")
    parser.add_argument("--window-out", help="Write the selected window samples to this file.")
    parser.add_argument("--write-header", nargs="?", const=str(DEFAULT_HEADER), help="Write calibration header.")
    args = parser.parse_args()

    data = filter_valid_samples(load_mag_data(args.logfile))
    if len(data) < 100:
        raise SystemExit("Need at least 100 samples; 1000+ with broad 3D coverage is recommended.")

    if args.mode == "2d" and args.best_window:
        best, top = find_best_2d_window(data, args.preserve_header)
        data = data[best["start"]:best["end"]]
        bias = best["bias"]
        soft_iron = best["soft_iron"]
        field_strength = best["field_strength"]
        radii = best["radii"]
        report = best["report"]
        print("Best windows:")
        for idx, item in enumerate(top, 1):
            rep = item["report"]
            print(
                f"  {idx}. samples[{item['start']}:{item['end']}] n={item['size']} "
                f"score={rep['score']:.1f} residual={rep['residual_percent']:.2f}% "
                f"ratio={rep['max_min_ratio']:.3f} coverage={rep['yaw_coverage_percent']:.1f}% "
                f"bins={rep['yaw_bins_occupied']}/{rep['yaw_bin_count']} "
                f"gap={rep['yaw_largest_gap_deg']:.1f}deg span={rep['yaw_unwrapped_span_deg']:.1f}deg"
            )
        print("")
        if args.window_out:
            out = Path(args.window_out)
            out.parent.mkdir(parents=True, exist_ok=True)
            with out.open("w", encoding="utf-8") as f:
                f.write("# selected 2D calibration window x,y,z\n")
                for row in data:
                    f.write(f"{row[0]:.6f},{row[1]:.6f},{row[2]:.6f}\n")
            print(f"Selected window written: {out}")
            print("")
    elif args.mode == "2d":
        bias, soft_iron, field_strength, radii = build_2d_calibration(data, args.preserve_header)
        corrected = apply_calibration(data, bias, soft_iron)
        report = quality_report_2d(data, corrected)
    else:
        bias, soft_iron, field_strength, radii = fit_ellipsoid(data)
        corrected = apply_calibration(data, bias, soft_iron)
        report = quality_report(data, corrected)
    print_report(data, bias, soft_iron, field_strength, radii, report, args.mode)

    text = header_text(bias, soft_iron, field_strength, len(data), report, args.mode)
    print("")
    print("Header preview:")
    print(text)

    if args.write_header:
        out = Path(args.write_header)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
        print(f"Wrote: {out}")


if __name__ == "__main__":
    main()
