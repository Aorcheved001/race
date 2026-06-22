# -*- coding: utf-8 -*-
"""Verify raw magnetometer data with the firmware calibration header."""

import argparse
import re
from pathlib import Path

import numpy as np

from mag_calibrate import filter_valid_samples, load_mag_data, quality_report, quality_report_2d


DEFAULT_HEADER = Path("code/ins/calibration_params.h")


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


def main():
    parser = argparse.ArgumentParser(description="Verify magnetometer calibration quality.")
    parser.add_argument("logfile", help="Raw magnetometer x,y,z log.")
    parser.add_argument("--header", default=str(DEFAULT_HEADER), help="Firmware calibration header.")
    parser.add_argument("--mode", choices=["2d", "3d"], default="2d", help="2d checks horizontal vehicle calibration.")
    args = parser.parse_args()

    data = filter_valid_samples(load_mag_data(args.logfile))
    bias, soft_iron, field = load_header(args.header)
    corrected = (data - bias) @ soft_iron.T
    norms = np.linalg.norm(corrected[:, :2], axis=1) if args.mode == "2d" else np.linalg.norm(corrected, axis=1)
    report = quality_report_2d(data, corrected) if args.mode == "2d" else quality_report(data, corrected)

    print(f"Data points: {len(data)}")
    print(f"Header: {args.header}")
    print(f"Hard iron: X={bias[0]:.3f}, Y={bias[1]:.3f}, Z={bias[2]:.3f}")
    print("Soft iron:")
    for row in soft_iron:
        print(f"  {row[0]: .6f}, {row[1]: .6f}, {row[2]: .6f}")
    print("")
    print(f"After calibration ({args.mode.upper()}):")
    print(f"  Norm mean: {np.mean(norms):.3f}")
    print(f"  Norm std: {np.std(norms):.3f}")
    print(f"  Field target: {field:.3f}")
    print(f"  Residual: {report['residual_percent']:.3f}%")
    print(f"  Max/min radius ratio: {report['max_min_ratio']:.4f}")
    print(f"  Yaw coverage: {report['yaw_coverage_percent']:.1f}%")
    if args.mode == "2d" and "yaw_largest_gap_deg" in report:
        print(f"  Yaw bins: {report['yaw_bins_occupied']}/{report['yaw_bin_count']}")
        print(f"  Largest yaw gap: {report['yaw_largest_gap_deg']:.1f} deg")
        print(f"  Circular yaw span: {report['yaw_circular_span_deg']:.1f} deg")
        print(f"  Unwrapped yaw span: {report['yaw_unwrapped_span_deg']:.1f} deg")
    if args.mode != "2d":
        print(f"  Elevation range: {report['elevation_min']:.1f} .. {report['elevation_max']:.1f} deg")
    print(f"  Score: {report['score']:.1f}/100")

    ratio_limit = 1.20 if args.mode == "3d" else 1.15
    if report["residual_percent"] > 5.0 or report["max_min_ratio"] > ratio_limit:
        raise SystemExit("Calibration quality is poor. Re-collect more full-heading driving data.")


if __name__ == "__main__":
    main()
