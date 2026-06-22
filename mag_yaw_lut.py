#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MagYaw LUT Builder & Corner Cutting Analyzer
=============================================

Task 1: Build magnetometer yaw correction lookup table (LUT)
  - Read track_plotter saved INS data (I: x,y,yaw_gyro,yaw_mag,yaw_ekf)
  - Read RTK data (R: x_ins,y_ins,yaw_ins,valid_flags,...,yaw_tn)
  - Compare RTK dual-antenna yaw (truth) vs magnetometer yaw
  - Build 1-degree resolution LUT (360 entries)
  - Output: C code for TC377 + Python dict for track_plotter

Task 2: Analyze corner cutting phenomenon
  - Compare DRIVE (reference) vs FOLLOW (actual) trajectories
  - Detect corners by curvature threshold
  - Compute signed lateral deviation (positive = outside of curve)
  - Segment analysis: first half vs second half (drift detection)
  - Curvature-deviation correlation

Usage:
  python mag_yaw_lut.py --dir track_data/ --build-lut
  python mag_yaw_lut.py --dir track_data/ --analyze-corner
  python mag_yaw_lut.py --dir track_data/ --show-yaw
  python mag_yaw_lut.py --dir track_data/ --all

Data files (in track_data/):
  - subj3_drive_*.txt       (DRIVE reference path)
  - subj3_follow_*.txt      (FOLLOW trajectory)
  - *_rtk.txt               (RTK data)
  - *_ins.txt               (INS data)
"""

import os
import sys
import math
import argparse
import numpy as np
from datetime import datetime

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# =====================================================================
# Data Loading
# =====================================================================

def parse_header(filepath):
    """Parse file header comments, return dict of key:value pairs."""
    meta = {}
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line.startswith('#'):
                break
            content = line[1:].strip()
            if ':' in content:
                key, val = content.split(':', 1)
                meta[key.strip()] = val.strip()
    return meta


def load_drive_data(filepath):
    """Load DRIVE reference path data.
    Format: index,x,y,yaw(rad),speed_dir
    Returns: [(idx, x, y, yaw, speed_dir), ...]
    """
    pts = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split(',')
            if len(parts) >= 5:
                try:
                    pts.append((int(parts[0]), float(parts[1]), float(parts[2]),
                               float(parts[3]), float(parts[4])))
                except ValueError:
                    continue
    return pts


def load_follow_data(filepath):
    """Load FOLLOW trajectory data.
    Format: idx,x,y,yaw,target_idx
    Returns: [(idx, x, y, yaw, target_idx), ...]
    """
    pts = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split(',')
            if len(parts) >= 4:
                try:
                    idx = int(parts[0])
                    x, y, yaw = float(parts[1]), float(parts[2]), float(parts[3])
                    target = int(parts[4]) if len(parts) > 4 else 0
                    pts.append((idx, x, y, yaw, target))
                except ValueError:
                    continue
    return pts


def load_rtk_data(filepath):
    """Load RTK data.
    Format: x_ins,y_ins,yaw_ins,valid_flags,phase,x_enu,y_enu,yaw_tn[,ant_raw,px_ins,py_ins]
    Returns: [(x_ins, y_ins, yaw_ins, vf, phase, x_enu, y_enu, yaw_tn), ...]
    """
    pts = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split(',')
            if len(parts) >= 8:
                try:
                    x_ins = float(parts[0])
                    y_ins = float(parts[1])
                    yaw_ins = float(parts[2])
                    vf = int(parts[3])
                    phase = parts[4]
                    x_enu = float(parts[5])
                    y_enu = float(parts[6])
                    yaw_tn = float(parts[7])
                    pts.append((x_ins, y_ins, yaw_ins, vf, phase, x_enu, y_enu, yaw_tn))
                except ValueError:
                    continue
    return pts


def load_ins_data(filepath):
    """Load INS data.
    Format: x,y,yaw_gyro_deg,yaw_mag_deg,yaw_ekf_deg
    Returns: [(x, y, yaw_gyro_deg, yaw_mag_deg, yaw_ekf_deg), ...]
    """
    pts = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split(',')
            if len(parts) >= 5:
                try:
                    pts.append((float(parts[0]), float(parts[1]),
                               float(parts[2]), float(parts[3]), float(parts[4])))
                except ValueError:
                    continue
    return pts


def load_yawrtk_data(filepath):
    """Load yawrtk data from log file.
    Format:
      yawrtk:seq,tick_ms,phase,yaw_gyro,mag_yaw_raw,mag_yaw_rel,
             mag_yaw_corr_rel,yaw_fused,rtk_yaw_ins,rtk_yaw_ref,
             rtk_valid,mag_x,mag_y,mag_z
    Returns: [(seq, tick_ms, phase, yaw_gyro, mag_yaw_raw, mag_yaw_rel,
              mag_yaw_corr_rel, yaw_fused, rtk_yaw_ins, rtk_yaw_ref,
              rtk_valid, mag_x, mag_y, mag_z), ...]
    """
    pts = []
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            if not line.startswith('yawrtk:'):
                continue
            parts = line[len('yawrtk:'):].split(',')
            if len(parts) >= 14:
                try:
                    pts.append((int(parts[0]), int(parts[1]), parts[2],
                               float(parts[3]), float(parts[4]), float(parts[5]),
                               float(parts[6]), float(parts[7]), float(parts[8]),
                               float(parts[9]), int(parts[10]), float(parts[11]),
                               float(parts[12]), float(parts[13])))
                except ValueError:
                    continue
    return pts


def build_mag_yaw_lut_from_yawrtk(yawrtk_data, bin_size_deg=1):
    """Build magnetometer yaw correction LUT from yawrtk data.

    Uses RTK dual-antenna yaw as ground truth and builds the additive
    correction used by the firmware:
      mag_yaw_corr = normalize(mag_yaw + lut[mag_yaw])
    This is the preferred method when yawrtk data is available.

    Args:
        yawrtk_data: [(seq, tick, phase, yaw_gyro, mag_yaw_raw, mag_yaw_rel,
                       mag_yaw_corr_rel, yaw_fused, rtk_yaw_ins, rtk_yaw_ref,
                       rtk_valid, mag_x, mag_y, mag_z), ...]
        bin_size_deg: LUT angular bin size (degrees)

    Returns:
        lut: dict, {mag_yaw_deg_bin: (mean_correction, median_correction, std, count)}
    """
    if not yawrtk_data:
        print("[LUT] No yawrtk data, cannot build LUT")
        return {}

    # Filter: use only data where RTK position and yaw are both valid.
    # rtk_yaw_ref is meaningful after RTK-to-INS alignment, which depends on
    # a valid position. Bit0=pos_valid, bit1=yaw_valid.
    valid_pairs = []
    for p in yawrtk_data:
        rtk_valid = p[10]
        if (rtk_valid & 0x03) != 0x03:
            continue
        mag_yaw_rel = p[5]   # mag_yaw_rel (degrees, relative to startup)
        rtk_yaw_ref = p[9]   # RTK yaw in the same relative INS frame
        # Additive correction for C firmware:
        #   corrected = mag + correction
        correction = rtk_yaw_ref - mag_yaw_rel
        while correction > 180: correction -= 360
        while correction < -180: correction += 360
        valid_pairs.append((mag_yaw_rel, correction))

    if not valid_pairs:
        print("[LUT] No RTK entries with both position and yaw valid in yawrtk data")
        return {}

    print(f"[LUT] Using {len(valid_pairs)} yawrtk samples with valid RTK yaw")

    # Bin by magnetometer yaw angle
    n_bins = int(360 / bin_size_deg)
    bins = {i: [] for i in range(n_bins)}

    for mag_yaw, err in valid_pairs:
        yaw_norm = mag_yaw % 360
        bin_idx = int(yaw_norm / bin_size_deg) % n_bins
        bins[bin_idx].append(err)

    # Compute statistics per bin
    lut = {}
    for bin_idx in range(n_bins):
        if bins[bin_idx]:
            arr = np.array(bins[bin_idx])
            median_err = float(np.median(arr))
            mean_err = float(np.mean(arr))
            std_err = float(np.std(arr))
            count = len(arr)
            lut[bin_idx] = (mean_err, median_err, std_err, count)
        else:
            lut[bin_idx] = (0.0, 0.0, 0.0, 0)

    return lut


def find_data_sets(directory):
    """Find paired data sets in directory.
    Returns: [{'follow': path, 'follow_rtk': path, 'follow_ins': path,
               'drive': path, 'drive_rtk': path, 'drive_ins': path}, ...]
    """
    datasets = []
    if not os.path.isdir(directory):
        return datasets

    files = os.listdir(directory)

    # Find follow files
    follow_files = sorted([f for f in files if f.startswith('subj3_follow_') and f.endswith('.txt')
                          and '_rtk' not in f and '_ins' not in f])

    for ff in follow_files:
        base = ff.replace('.txt', '')
        rtk_file = os.path.join(directory, f"{base}_rtk.txt")
        ins_file = os.path.join(directory, f"{base}_ins.txt")

        ds = {
            'follow': os.path.join(directory, ff),
            'follow_rtk': rtk_file if os.path.exists(rtk_file) else None,
            'follow_ins': ins_file if os.path.exists(ins_file) else None,
        }

        # Check for yawrtk data file
        yawrtk_file = os.path.join(directory, f"{base}_yawrtk.txt")
        if os.path.exists(yawrtk_file):
            ds['follow_yawrtk'] = yawrtk_file

        # Find matching drive file
        drive_files = sorted([f for f in files if f.startswith('subj3_drive_') and f.endswith('.txt')
                             and '_rtk' not in f and '_ins' not in f])

        for df in drive_files:
            drive_base = df.replace('.txt', '')
            drive_rtk = os.path.join(directory, f"{drive_base}_rtk.txt")
            drive_ins = os.path.join(directory, f"{drive_base}_ins.txt")

            if os.path.exists(drive_rtk) or os.path.exists(drive_ins):
                ds['drive'] = os.path.join(directory, df)
                ds['drive_rtk'] = drive_rtk if os.path.exists(drive_rtk) else None
                ds['drive_ins'] = drive_ins if os.path.exists(drive_ins) else None
                break

        datasets.append(ds)

    return datasets


# =====================================================================
# LUT Building
# =====================================================================

def build_mag_yaw_lut(ins_data, rtk_data, bin_size_deg=1):
    """Build magnetometer yaw correction lookup table.

    Uses RTK dual-antenna yaw as ground truth, compares with INS magnetometer yaw.
    Time alignment: assumes data is sorted by time, uses linear interpolation.

    Args:
        ins_data: [(x, y, yaw_gyro_deg, yaw_mag_deg, yaw_ekf_deg), ...]
        rtk_data: [(x_ins, y_ins, yaw_ins, vf, phase, x_enu, y_enu, yaw_tn), ...]
        bin_size_deg: LUT angular bin size (degrees)

    Returns:
        lut: dict, {mag_yaw_deg_bin: (mean_correction, median_correction, std, count)}
    """
    if not ins_data or not rtk_data:
        print("[LUT] Missing INS or RTK data, cannot build LUT")
        return {}

    # Filter valid RTK data (pos_valid + yaw_valid)
    # RTK tuple: (x_ins, y_ins, yaw_ins, vf, phase, x_enu, y_enu, yaw_tn)
    # p[3]=valid_flags, p[7]=yaw_tn (degrees, from C: yaw_tn*57.29578)
    valid_rtk_yaw = [p[7] for p in rtk_data if (p[3] & 3) == 3]

    if not valid_rtk_yaw:
        print("[LUT] RTK data has no pos+yaw valid entries, cannot build LUT")
        return {}

    # RTK yaw_tn from file is already in DEGREES (C code sends yaw_tn*57.29578)
    rtk_yaw_deg_list = valid_rtk_yaw  # already in degrees, no conversion needed

    # INS data: yaw_mag_deg is already in degrees
    ins_mag_deg_list = [p[3] for p in ins_data]  # yaw_mag_deg

    # Time alignment: INS and RTK have different sampling rates
    # Simple approach: assume data is sorted by time, use linear interpolation
    n_ins = len(ins_mag_deg_list)
    n_rtk = len(rtk_yaw_deg_list)

    # Interpolate RTK yaw to match INS data points
    rtk_indices = np.linspace(0, n_rtk - 1, n_ins)
    rtk_yaw_interp = np.interp(rtk_indices, range(n_rtk), rtk_yaw_deg_list)

    # Compute additive correction: rtk_yaw - mag_yaw (normalize to [-180, 180])
    errors = []
    for i in range(n_ins):
        mag_yaw = ins_mag_deg_list[i]
        rtk_yaw = rtk_yaw_interp[i]
        err = rtk_yaw - mag_yaw
        # Normalize to [-180, 180]
        while err > 180:
            err -= 360
        while err < -180:
            err += 360
        errors.append((mag_yaw, err))

    # Bin by magnetometer yaw angle
    n_bins = int(360 / bin_size_deg)
    bins = {i: [] for i in range(n_bins)}

    for mag_yaw, err in errors:
        # Normalize mag_yaw to [0, 360)
        yaw_norm = mag_yaw % 360
        bin_idx = int(yaw_norm / bin_size_deg) % n_bins
        bins[bin_idx].append(err)

    # Compute statistics per bin
    lut = {}
    for bin_idx in range(n_bins):
        if bins[bin_idx]:
            arr = np.array(bins[bin_idx])
            # Use median for robustness against outliers
            median_err = float(np.median(arr))
            mean_err = float(np.mean(arr))
            std_err = float(np.std(arr))
            count = len(arr)
            lut[bin_idx] = (mean_err, median_err, std_err, count)
        else:
            lut[bin_idx] = (0.0, 0.0, 0.0, 0)

    return lut


def apply_lut_correction(mag_yaw_deg, lut, bin_size_deg=1):
    """Apply LUT correction to magnetometer yaw.

    Args:
        mag_yaw_deg: raw magnetometer yaw (degrees)
        lut: lookup table
        bin_size_deg: bin size

    Returns:
        corrected_yaw_deg: corrected yaw (degrees)
    """
    yaw_norm = mag_yaw_deg % 360
    bin_idx = int(yaw_norm / bin_size_deg) % int(360 / bin_size_deg)

    if bin_idx in lut and lut[bin_idx][3] > 0:
        # Use median correction
        correction = lut[bin_idx][1]  # median error
        corrected = mag_yaw_deg + correction
        return corrected
    return mag_yaw_deg


def generate_c_lut_code(lut, bin_size_deg=1, var_name="mag_yaw_lut"):
    """Generate C code for LUT array.

    Args:
        lut: lookup table
        bin_size_deg: bin size
        var_name: C variable name

    Returns:
        C code string
    """
    n_bins = int(360 / bin_size_deg)

    code = f"""// ====================================================================
// Magnetometer correction lookup table (Auto-generated by mag_yaw_lut.py)
// Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
// LUT resolution: {bin_size_deg}deg, total {n_bins} entries
// Usage: corrected_yaw = mag_yaw + mag_yaw_lut[bin_index]
//        bin_index = (((int)(mag_yaw_deg) % 360) + 360) % 360 / {bin_size_deg}
// ====================================================================
const float {var_name}[{n_bins}] = {{
"""

    for i in range(n_bins):
        if i in lut and lut[i][3] > 0:
            val = lut[i][1]  # use median
        else:
            val = 0.0
        if i % 10 == 0:
            code += "    "
        code += f"{val:+.4f}f"
        if i < n_bins - 1:
            code += ", "
        if (i + 1) % 10 == 0:
            code += f"  // {i-8}-{i}deg\n"

    code += "};\n"

    # Add usage example
    code += f"""
// Usage example:
// float mag_yaw_deg = ...;  // calibrated magnetometer yaw (degrees)
// int bin_idx = (((int)(mag_yaw_deg) % 360) + 360) % 360 / {bin_size_deg};
// float corrected_yaw_deg = mag_yaw_deg + {var_name}[bin_idx];
"""
    return code


def generate_python_lut(lut, bin_size_deg=1):
    """Generate Python LUT dict."""
    n_bins = int(360 / bin_size_deg)
    py_lut = {}
    for i in range(n_bins):
        if i in lut and lut[i][3] > 0:
            py_lut[i] = lut[i][1]  # median correction
        else:
            py_lut[i] = 0.0
    return py_lut


# =====================================================================
# Path Analysis
# =====================================================================

def compute_heading_changes(pts, window=5):
    """Compute heading and curvature for a path.

    Args:
        pts: [(x, y), ...] path points
        window: smoothing window size

    Returns:
        headings: [heading_deg, ...] heading at each point
        curvatures: [curvature, ...] curvature at each point (deg/m)
    """
    if len(pts) < 3:
        return [], []

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    # Compute heading
    headings = []
    for i in range(len(pts)):
        if i == 0:
            dx = xs[1] - xs[0]
            dy = ys[1] - ys[0]
        elif i == len(pts) - 1:
            dx = xs[-1] - xs[-2]
            dy = ys[-1] - ys[-2]
        else:
            dx = xs[i+1] - xs[i-1]
            dy = ys[i+1] - ys[i-1]
        headings.append(math.degrees(math.atan2(dy, dx)))

    # Compute curvature (heading change / arc length)
    curvatures = [0.0]
    for i in range(1, len(pts)):
        da = headings[i] - headings[i-1]
        while da > 180:
            da -= 360
        while da < -180:
            da += 360
        ds = math.sqrt((xs[i]-xs[i-1])**2 + (ys[i]-ys[i-1])**2)
        if ds > 1e-6:
            curvatures.append(da / ds)  # deg/m
        else:
            curvatures.append(0.0)

    # Smooth
    if window > 1 and len(curvatures) > window:
        kernel = np.ones(window) / window
        curvatures = np.convolve(curvatures, kernel, mode='same').tolist()

    return headings, curvatures


def detect_corners(pts, curvature_threshold=15.0, min_corner_pts=5, max_merge_gap=3):
    """Detect corner regions in path.

    Args:
        pts: [(x, y), ...] path points
        curvature_threshold: curvature threshold (deg/m)
        min_corner_pts: minimum consecutive points for a corner
        max_merge_gap: merge adjacent corners separated by fewer points

    Returns:
        corners: [(start_idx, end_idx, peak_curvature, direction), ...]
            direction: +1 = left turn, -1 = right turn (from peak_curvature sign)
    """
    _, curvatures = compute_heading_changes(pts)
    if not curvatures:
        return []

    # Filter extreme curvatures (path discontinuities produce fake curvature)
    # Typical vehicle corner curvature < 100 deg/m; >200 is data artifact
    filtered_curvatures = []
    for c in curvatures:
        if abs(c) > 200:
            filtered_curvatures.append(0.0)
        else:
            filtered_curvatures.append(c)

    corners = []
    in_corner = False
    start_idx = 0
    peak_curv = 0.0

    for i, c in enumerate(filtered_curvatures):
        if abs(c) > curvature_threshold:
            if not in_corner:
                in_corner = True
                start_idx = i
                peak_curv = c
            else:
                if abs(c) > abs(peak_curv):
                    peak_curv = c
        else:
            if in_corner:
                end_idx = i
                if end_idx - start_idx >= min_corner_pts:
                    # direction from peak curvature sign (not first entry sign)
                    direction = 1 if peak_curv > 0 else -1
                    corners.append((start_idx, end_idx, peak_curv, direction))
                in_corner = False
                peak_curv = 0.0

    # Handle last corner
    if in_corner:
        end_idx = len(filtered_curvatures) - 1
        if end_idx - start_idx >= min_corner_pts:
            direction = 1 if peak_curv > 0 else -1
            corners.append((start_idx, end_idx, peak_curv, direction))

    # Merge adjacent corners (gap < max_merge_gap points)
    if len(corners) > 1:
        merged = [corners[0]]
        for s, e, pk, d in corners[1:]:
            prev_s, prev_e, prev_pk, prev_d = merged[-1]
            if s - prev_e <= max_merge_gap:
                new_peak = prev_pk if abs(prev_pk) > abs(pk) else pk
                new_dir = 1 if new_peak > 0 else -1
                merged[-1] = (prev_s, e, new_peak, new_dir)
            else:
                merged.append((s, e, pk, d))
        corners = merged

    return corners


def compute_signed_lateral_deviation(car_pts, ref_pts):
    """Compute signed lateral deviation of car from reference path.

    Positive = car is to the left of path direction (outside of right turn)
    Negative = car is to the right of path direction (inside of right turn)

    Args:
        car_pts: [(x, y), ...] car positions
        ref_pts: [(x, y), ...] reference path

    Returns:
        deviations: [signed_dev, ...] signed lateral deviation
    """
    deviations = []

    for cx, cy in car_pts:
        min_dist = float('inf')
        best_proj = None
        best_idx = 0

        for i in range(len(ref_pts) - 1):
            x1, y1 = ref_pts[i]
            x2, y2 = ref_pts[i+1]
            dx, dy = x2 - x1, y2 - y1
            seg_len_sq = dx*dx + dy*dy

            if seg_len_sq < 1e-12:
                dist = math.sqrt((cx - x1)**2 + (cy - y1)**2)
                proj = (x1, y1)
                idx = i
            else:
                t = max(0, min(1, ((cx - x1) * dx + (cy - y1) * dy) / seg_len_sq))
                px, py = x1 + t * dx, y1 + t * dy
                dist = math.sqrt((cx - px)**2 + (cy - py)**2)
                proj = (px, py)
                idx = i

            if dist < min_dist:
                min_dist = dist
                best_proj = proj
                best_idx = idx

        # Compute sign: cross product determines which side of path the car is on
        if best_proj is not None and best_idx < len(ref_pts) - 1:
            seg_dx = ref_pts[best_idx + 1][0] - ref_pts[best_idx][0]
            seg_dy = ref_pts[best_idx + 1][1] - ref_pts[best_idx][1]
            cross = seg_dx * (cy - best_proj[1]) - seg_dy * (cx - best_proj[0])
            signed_dist = min_dist if cross > 0 else -min_dist
        else:
            signed_dist = 0.0

        deviations.append(signed_dist)

    return deviations


def analyze_corner_cutting(drive_pts, follow_pts, curvature_threshold=15.0):
    """Analyze corner cutting phenomenon.

    Handles reverse FOLLOW mode: determines which portion of ref path
    the car actually drove, and only analyzes that portion.

    Args:
        drive_pts: DRIVE reference path [(idx, x, y, yaw, speed_dir), ...]
        follow_pts: FOLLOW trajectory [(idx, x, y, yaw, target_idx), ...]
        curvature_threshold: curvature threshold for corner detection (deg/m)

    Returns:
        analysis dict with all results
    """
    # Extract car positions (idx=0) and reference path
    car_pts = [(p[1], p[2]) for p in follow_pts if p[0] == 0]
    ref_pts = [(p[1], p[2]) for p in drive_pts]

    if not car_pts or not ref_pts:
        return {'error': 'No car or reference points'}

    # --- Determine which portion of ref path the car actually drove ---
    def nearest_ref_idx(px, py):
        best_i, best_d = 0, 1e18
        for i, (rx, ry) in enumerate(ref_pts):
            d = (px - rx)**2 + (py - ry)**2
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    car_start_ri = nearest_ref_idx(car_pts[0][0], car_pts[0][1])
    car_end_ri = nearest_ref_idx(car_pts[-1][0], car_pts[-1][1])

    # ref range the car actually drove (handle both forward and reverse)
    ref_lo = max(0, min(car_start_ri, car_end_ri) - 5)
    ref_hi = min(len(ref_pts) - 1, max(car_start_ri, car_end_ri) + 5)

    # Slice the driven portion of ref path
    driven_ref = ref_pts[ref_lo:ref_hi + 1]

    # Detect corners ONLY on the driven portion
    corners = detect_corners(driven_ref, curvature_threshold)

    # Offset corner indices back to full ref_pts coordinate
    corners_full = [(s + ref_lo, e + ref_lo, pk, d) for s, e, pk, d in corners]

    # Compute lateral deviation against driven portion only
    deviations = compute_signed_lateral_deviation(car_pts, driven_ref)

    # --- Global stats ---
    abs_devs = [abs(d) for d in deviations]
    max_dev = max(abs_devs) if abs_devs else 0
    mean_dev = float(np.mean(deviations)) if deviations else 0
    rms_dev = float(np.sqrt(np.mean(np.array(deviations)**2))) if deviations else 0
    outside_pct = sum(1 for d in deviations if d > 0.05) / len(deviations) * 100 if deviations else 0

    # --- Per-corner stats ---
    corner_stats = []
    for ci, (start_idx_full, end_idx_full, peak_curv, direction) in enumerate(corners_full):
        start_local = start_idx_full - ref_lo
        end_local = end_idx_full - ref_lo
        center_local = (start_local + end_local) // 2
        if center_local < len(driven_ref):
            corner_center = driven_ref[center_local]
        else:
            corner_center = driven_ref[-1]

        # Find car points near this corner
        corner_car_devs = []
        for i, (cx, cy) in enumerate(car_pts):
            dist_to_corner = math.sqrt((cx - corner_center[0])**2 + (cy - corner_center[1])**2)
            if dist_to_corner < 2.0 and i < len(deviations):
                corner_car_devs.append(deviations[i])

        if corner_car_devs:
            corner_stats.append({
                'ref_start': start_idx_full,
                'ref_end': end_idx_full,
                'peak_curvature': peak_curv,
                'direction': 'left' if direction > 0 else 'right',
                'mean_dev': float(np.mean(corner_car_devs)),
                'max_dev': float(max(abs(d) for d in corner_car_devs)),
                'n_points': len(corner_car_devs),
                'center': corner_center,
            })

    # --- Straight segment stats ---
    corner_car_indices = set()
    for cs in corner_stats:
        for i, (cx, cy) in enumerate(car_pts):
            dist = math.sqrt((cx - cs['center'][0])**2 + (cy - cs['center'][1])**2)
            if dist < 2.0:
                corner_car_indices.add(i)
    straight_devs = [deviations[i] for i in range(len(deviations)) if i not in corner_car_indices]

    analysis = {
        'total_car_pts': len(car_pts),
        'total_ref_pts': len(ref_pts),
        'driven_ref_range': (ref_lo, ref_hi),
        'driven_ref_pts': len(driven_ref),
        'max_deviation': max_dev,
        'mean_deviation': mean_dev,
        'rms_deviation': rms_dev,
        'outside_pct': outside_pct,
        'n_corners': len(corners_full),
        'corner_stats': corner_stats,
        'straight_mean_dev': float(np.mean(straight_devs)) if straight_devs else 0,
        'straight_rms_dev': float(np.sqrt(np.mean(np.array(straight_devs)**2))) if straight_devs else 0,
        'deviations': deviations,
        'car_pts': car_pts,
        'ref_pts': driven_ref,
        'full_ref_pts': ref_pts,
        'corners': corners_full,
    }

    # --- Curvature-deviation correlation ---
    # Compute per-point curvature on driven ref path
    _, driven_curvatures = compute_heading_changes(driven_ref, window=5)

    # For each car point, find nearest ref point and its curvature
    curv_bins = {5: [], 10: [], 15: [], 25: []}
    for i, (cx, cy) in enumerate(car_pts):
        # Find nearest ref point
        min_d = 1e18
        best_ri = 0
        for ri, (rx, ry) in enumerate(driven_ref):
            d = (cx-rx)**2 + (cy-ry)**2
            if d < min_d:
                min_d = d
                best_ri = ri
        local_curv = abs(driven_curvatures[best_ri]) if best_ri < len(driven_curvatures) else 0
        dev = deviations[i] if i < len(deviations) else 0

        for thresh in curv_bins:
            if local_curv >= thresh:
                curv_bins[thresh].append(dev)

    corr_data = []
    for thresh in sorted(curv_bins.keys()):
        devs = curv_bins[thresh]
        if devs:
            corr_data.append({
                'threshold': thresh,
                'mean_dev': float(np.mean(devs)),
                'rms_dev': float(np.sqrt(np.mean(np.array(devs)**2))),
                'count': len(devs),
            })
    analysis['curvature_dev_corr'] = corr_data

    # --- Segment analysis: compare first half vs second half ---
    n_dev = len(deviations)
    if n_dev > 4:
        first_half = deviations[:n_dev//2]
        second_half = deviations[n_dev//2:]
        analysis['first_half_mean'] = float(np.mean(first_half))
        analysis['first_half_rms'] = float(np.sqrt(np.mean(np.array(first_half)**2)))
        analysis['second_half_mean'] = float(np.mean(second_half))
        analysis['second_half_rms'] = float(np.sqrt(np.mean(np.array(second_half)**2)))
    else:
        analysis['first_half_mean'] = 0
        analysis['first_half_rms'] = 0
        analysis['second_half_mean'] = 0
        analysis['second_half_rms'] = 0

    return analysis


def print_analysis_report(analysis):
    """Print analysis report"""
    print()
    print("=" * 60)
    print("  Corner Cutting Analysis Report")
    print("=" * 60)
    ref_lo, ref_hi = analysis.get('driven_ref_range', (0, analysis['total_ref_pts']-1))
    driven = analysis.get('driven_ref_pts', analysis['total_ref_pts'])
    print("  Ref path: %d pts (car drove idx %d-%d, %d pts)" % (
        analysis['total_ref_pts'], ref_lo, ref_hi, driven))
    print("  Car position points: %d" % analysis['total_car_pts'])
    print("  Corners detected: %d" % analysis['n_corners'])
    print("-" * 60)
    print("  Max lateral deviation: %.3f m" % analysis['max_deviation'])
    print("  Mean lateral deviation: %+.3f m" % analysis['mean_deviation'])
    print("  RMS lateral deviation: %.3f m" % analysis['rms_deviation'])
    print("  Outside ratio: %.0f%%" % analysis['outside_pct'])
    print("-" * 60)
    print("  Straight segment mean dev: %+.3f m" % analysis['straight_mean_dev'])
    print("  Straight segment RMS dev: %.3f m" % analysis['straight_rms_dev'])
    print("-" * 60)

    for i, cs in enumerate(analysis['corner_stats']):
        print("  Corner %d: %s turn, peak=%.1f deg/m, mean_dev=%+.3f m, max_dev=%.3f m (%d pts)" % (
            i+1, cs['direction'], cs['peak_curvature'],
            cs['mean_dev'], cs['max_dev'], cs['n_points']))

    # Summary
    if analysis['mean_deviation'] > 0.05:
        print("-" * 60)
        print("  Vehicle tends to be OUTSIDE (corner cutting):")
        print("    1. Increase feedforward steering angle")
        print("    2. Slow down before curves")
        print("    3. Increase Pure Pursuit lookahead distance")
    elif analysis['mean_deviation'] < -0.05:
        print("-" * 60)
        print("  Vehicle tends to be INSIDE (understeering):")
        print("    1. Decrease feedforward steering angle")
        print("    2. Increase turn radius")
    else:
        print("-" * 60)
        print("  Tracking accuracy is good")

    # Segment analysis
    if 'first_half_mean' in analysis:
        print("-" * 60)
        print("  Segment Analysis (first half = start of follow):")
        print("    First half:  mean=%+.3f m, RMS=%.3f m" % (
            analysis['first_half_mean'], analysis['first_half_rms']))
        print("    Second half: mean=%+.3f m, RMS=%.3f m" % (
            analysis['second_half_mean'], analysis['second_half_rms']))
        if abs(analysis['second_half_rms'] - analysis['first_half_rms']) > 0.5:
            if analysis['second_half_rms'] > analysis['first_half_rms']:
                print("    WARNING: Tracking degrades over time (drift accumulation)")
            else:
                print("    NOTE: Tracking improves over time")

    # Curvature-deviation correlation
    if 'curvature_dev_corr' in analysis:
        corr = analysis['curvature_dev_corr']
        if corr:
            print("-" * 60)
            print("  Curvature vs Deviation Correlation:")
            for row in corr:
                print("    curv>%5.0f deg/m: mean_dev=%+.3f m, RMS=%.3f m (%d pts)" % (
                    row['threshold'], row['mean_dev'], row['rms_dev'], row['count']))

    print("=" * 60)


# =====================================================================
# Plotting
# =====================================================================

def plot_mag_yaw_lut(lut, bin_size_deg=1, save_path=None):
    """Plot magnetometer LUT correction curve."""
    if not HAS_MPL:
        print("[WARN] matplotlib not available, skipping plot")
        return

    n_bins = int(360 / bin_size_deg)
    angles = [i * bin_size_deg for i in range(n_bins)]
    mean_errs = [lut.get(i, (0,0,0,0))[0] for i in range(n_bins)]
    median_errs = [lut.get(i, (0,0,0,0))[1] for i in range(n_bins)]
    std_errs = [lut.get(i, (0,0,0,0))[2] for i in range(n_bins)]
    counts = [lut.get(i, (0,0,0,0))[3] for i in range(n_bins)]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 1. Correction curve
    ax1 = axes[0]
    ax1.fill_between(angles,
                     [m - s for m, s in zip(median_errs, std_errs)],
                     [m + s for m, s in zip(median_errs, std_errs)],
                     alpha=0.3, color='blue', label='+/-1std')
    ax1.plot(angles, median_errs, 'b-', linewidth=1.5, label='Median correction')
    ax1.plot(angles, mean_errs, 'r--', linewidth=1.0, alpha=0.7, label='Mean correction')
    ax1.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax1.set_xlabel('Magnetometer Yaw (deg)')
    ax1.set_ylabel('Correction (deg)')
    ax1.set_title('Magnetometer Yaw LUT Correction Curve')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Correction polar plot
    ax2 = fig.add_subplot(3, 1, 2, projection='polar')
    theta = np.radians(angles)
    ax2.plot(theta, median_errs, 'b-', linewidth=1.5, label='Correction')
    ax2.set_title('LUT Correction (Polar)', pad=20)
    ax2.legend(loc='upper right')

    # 3. Sample distribution
    ax3 = axes[2]
    ax3.bar(angles, counts, width=bin_size_deg, color='green', alpha=0.6)
    ax3.set_xlabel('Magnetometer Yaw (deg)')
    ax3.set_ylabel('Sample Count')
    ax3.set_title('Sample Distribution per Bin')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[LUT] Plot saved to: {save_path}")
    else:
        plt.show()


def plot_corner_cutting(analysis, save_path=None):
    """Plot corner cutting analysis results."""
    if not HAS_MPL:
        print("[WARN] matplotlib not available, skipping plot")
        return

    car_pts = analysis['car_pts']
    ref_pts = analysis['ref_pts']
    deviations = analysis['deviations']
    corners = analysis['corners']

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # 1. Trajectory comparison
    ax1 = axes[0, 0]
    rx = [p[0] for p in ref_pts]
    ry = [p[1] for p in ref_pts]
    cx = [p[0] for p in car_pts]
    cy = [p[1] for p in car_pts]

    # Plot full reference path as thin gray background
    full_ref = analysis.get('full_ref_pts', ref_pts)
    if full_ref is not ref_pts:
        frx = [p[0] for p in full_ref]
        fry = [p[1] for p in full_ref]
        ax1.plot(frx, fry, '-', color='lightgray', linewidth=1.0, alpha=0.5, label='Full path')
    ax1.plot(rx, ry, 'b-', linewidth=2, label='Driven ref (save)', alpha=0.7)
    ax1.plot(cx, cy, 'r-', linewidth=1.5, label='Car actual', alpha=0.7)

    # Mark corners
    ref_lo = analysis.get('driven_ref_range', (0, len(ref_pts)-1))[0]
    for ci, (s, e, pk, d) in enumerate(corners):
        s_local = s - ref_lo
        e_local = e - ref_lo
        if 0 <= s_local < len(ref_pts) and 0 <= e_local < len(ref_pts):
            corner_pts_x = [ref_pts[i][0] for i in range(max(0, s_local), min(len(ref_pts), e_local+1))]
            corner_pts_y = [ref_pts[i][1] for i in range(max(0, s_local), min(len(ref_pts), e_local+1))]
            ax1.plot(corner_pts_x, corner_pts_y, 'g-', linewidth=4, alpha=0.5)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory Comparison')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # 2. Lateral deviation along path
    ax2 = axes[0, 1]
    ax2.plot(range(len(deviations)), deviations, 'b-', linewidth=0.8, alpha=0.7)
    ax2.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax2.axhline(0.05, color='orange', linewidth=0.5, linestyle=':', label='Outside threshold')
    ax2.axhline(-0.05, color='orange', linewidth=0.5, linestyle=':')
    ax2.set_xlabel('Car point index')
    ax2.set_ylabel('Signed lateral deviation (m)')
    ax2.set_title('Lateral Deviation Along Path')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # 3. Deviation histogram
    ax3 = axes[1, 0]
    ax3.hist(deviations, bins=50, color='steelblue', alpha=0.7, edgecolor='white')
    ax3.axvline(0, color='gray', linewidth=0.5, linestyle='--')
    ax3.axvline(np.mean(deviations), color='red', linewidth=1.0, linestyle='-', label='Mean')
    ax3.set_xlabel('Signed lateral deviation (m)')
    ax3.set_ylabel('Count')
    ax3.set_title('Deviation Distribution')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 4. Per-corner deviation
    ax4 = axes[1, 1]
    if analysis['corner_stats']:
        labels = [f"C{i+1}\n({cs['direction']})" for i, cs in enumerate(analysis['corner_stats'])]
        means = [cs['mean_dev'] for cs in analysis['corner_stats']]
        maxs = [cs['max_dev'] for cs in analysis['corner_stats']]
        x_pos = range(len(labels))
        ax4.bar(x_pos, means, color='steelblue', alpha=0.7, label='Mean dev')
        ax4.bar(x_pos, maxs, color='salmon', alpha=0.5, label='Max dev')
        ax4.set_xticks(x_pos)
        ax4.set_xticklabels(labels, fontsize=8)
        ax4.axhline(0, color='gray', linewidth=0.5, linestyle='--')
        ax4.set_ylabel('Deviation (m)')
        ax4.set_title('Per-Corner Deviation')
        ax4.legend(fontsize=8)
        ax4.grid(True, alpha=0.3)
    else:
        ax4.text(0.5, 0.5, 'No corners detected', ha='center', va='center', transform=ax4.transAxes)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[Corner] Plot saved to: {save_path}")
    else:
        plt.show()


def plot_three_yaw_display(ins_data, rtk_data, lut=None, bin_size_deg=1, save_path=None):
    """Plot three yaw comparison: raw mag, calibrated mag, LUT-corrected mag.

    Args:
        ins_data: [(x, y, yaw_gyro_deg, yaw_mag_deg, yaw_ekf_deg), ...]
        rtk_data: [(x_ins, y_ins, yaw_ins, vf, phase, x_enu, y_enu, yaw_tn), ...]
        lut: lookup table (optional)
    """
    if not HAS_MPL:
        print("[WARN] matplotlib not available, skipping plot")
        return

    if not ins_data:
        print("[WARN] No INS data")
        return

    n = len(ins_data)
    t = np.arange(n)
    yaw_gyro = [p[2] for p in ins_data]
    yaw_mag_raw = [p[3] for p in ins_data]
    yaw_ekf = [p[4] for p in ins_data]

    # LUT correction
    if lut:
        yaw_mag_lut = [apply_lut_correction(p[3], lut, bin_size_deg) for p in ins_data]
    else:
        yaw_mag_lut = yaw_mag_raw

    # RTK yaw (if available)
    rtk_yaw_deg = None
    if rtk_data:
        # p[7] = yaw_tn (radians)
        valid_rtk = [math.degrees(p[7]) for p in rtk_data if (p[3] & 3) == 3]
        if valid_rtk:
            rtk_yaw_deg = valid_rtk

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # 1. Three yaw time series
    ax1 = axes[0]
    ax1.plot(t, yaw_gyro, 'g-', linewidth=0.8, alpha=0.7, label='Gyro yaw')
    ax1.plot(t, yaw_mag_raw, 'b-', linewidth=0.8, alpha=0.7, label='Mag yaw (raw)')
    ax1.plot(t, yaw_mag_lut, 'c-', linewidth=0.8, alpha=0.7, label='Mag yaw (LUT corrected)')
    ax1.plot(t, yaw_ekf, 'r-', linewidth=1.0, alpha=0.9, label='EKF yaw')
    if rtk_yaw_deg:
        rtk_t = np.linspace(0, n-1, len(rtk_yaw_deg))
        ax1.plot(rtk_t, rtk_yaw_deg, 'k-', linewidth=1.5, alpha=0.8, label='RTK yaw (truth)')
    ax1.set_ylabel('Yaw (deg)')
    ax1.set_title('Yaw Comparison: Gyro / Mag Raw / Mag LUT / EKF / RTK')
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(True, alpha=0.3)

    # 2. Magnetometer error (vs EKF)
    ax2 = axes[1]
    mag_err_raw = [(yaw_mag_raw[i] - yaw_ekf[i]) for i in range(n)]
    mag_err_lut = [(yaw_mag_lut[i] - yaw_ekf[i]) for i in range(n)]
    # Normalize to [-180, 180]
    for i in range(n):
        while mag_err_raw[i] > 180: mag_err_raw[i] -= 360
        while mag_err_raw[i] < -180: mag_err_raw[i] += 360
        while mag_err_lut[i] > 180: mag_err_lut[i] -= 360
        while mag_err_lut[i] < -180: mag_err_lut[i] += 360
    ax2.plot(t, mag_err_raw, 'b-', linewidth=0.8, alpha=0.7, label='Mag raw error')
    ax2.plot(t, mag_err_lut, 'c-', linewidth=0.8, alpha=0.7, label='Mag LUT error')
    ax2.axhline(0, color='gray', linewidth=0.5, linestyle='--')
    ax2.set_ylabel('Error vs EKF (deg)')
    ax2.set_title('Magnetometer Error (vs EKF)')
    ax2.legend(fontsize=8, loc='upper right')
    ax2.grid(True, alpha=0.3)

    # 3. Error histogram
    ax3 = axes[2]
    ax3.hist(mag_err_raw, bins=50, color='blue', alpha=0.5, label='Raw error')
    ax3.hist(mag_err_lut, bins=50, color='cyan', alpha=0.5, label='LUT error')
    ax3.set_xlabel('Error (deg)')
    ax3.set_ylabel('Count')
    ax3.set_title('Error Distribution')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"[Yaw] Plot saved to: {save_path}")
    else:
        plt.show()


# =====================================================================
# Main
# =====================================================================

def main():
    parser = argparse.ArgumentParser(description='MagYaw LUT Builder & Corner Cutting Analyzer')
    parser.add_argument('--dir', type=str, default='track_data', help='Data directory')
    parser.add_argument('--lut-out', type=str, default=None, help='LUT C code output file')
    parser.add_argument('--bin-size', type=int, default=1, help='LUT angular bin size (deg)')
    parser.add_argument('--build-lut', action='store_true', help='Build magnetometer LUT')
    parser.add_argument('--analyze-corner', action='store_true', help='Analyze corner cutting')
    parser.add_argument('--show-yaw', action='store_true', help='Show three yaw comparison')
    parser.add_argument('--all', action='store_true', help='Run all analyses')
    parser.add_argument('--curvature-thresh', type=float, default=15.0, help='Curvature threshold (deg/m)')
    parser.add_argument('--no-plot', action='store_true', help='Do not display plots')

    args = parser.parse_args()

    if args.all:
        args.build_lut = True
        args.analyze_corner = True
        args.show_yaw = True

    data_dir = args.dir

    # ---- Load data ----
    print(f"Loading data from: {data_dir}")
    datasets = find_data_sets(data_dir)
    if not datasets:
        print("[WARN] No data sets found")
        return

    # Use the latest dataset
    ds = datasets[-1]
    print(f"  Follow: {ds.get('follow', 'N/A')}")
    print(f"  Drive:  {ds.get('drive', 'N/A')}")

    # Load follow data
    follow_data = load_follow_data(ds['follow']) if ds.get('follow') else []
    drive_data = load_drive_data(ds['drive']) if ds.get('drive') else []

    # Load RTK data (try follow_rtk first, then drive_rtk)
    rtk_data = []
    if ds.get('follow_rtk'):
        rtk_data = load_rtk_data(ds['follow_rtk'])
    if not rtk_data and ds.get('drive_rtk'):
        rtk_data = load_rtk_data(ds['drive_rtk'])

    # Load INS data (try follow_ins first, then drive_ins)
    ins_data = []
    if ds.get('follow_ins'):
        ins_data = load_ins_data(ds['follow_ins'])
    if not ins_data and ds.get('drive_ins'):
        ins_data = load_ins_data(ds['drive_ins'])

    # Load yawrtk data (preferred for LUT building)
    yawrtk_data = []
    if ds.get('follow_yawrtk'):
        yawrtk_data = load_yawrtk_data(ds['follow_yawrtk'])

    print(f"  Follow pts: {len(follow_data)}, Drive pts: {len(drive_data)}")
    print(f"  RTK pts: {len(rtk_data)}, INS pts: {len(ins_data)}, yawrtk pts: {len(yawrtk_data)}")

    # ---- Build LUT ----
    if args.build_lut:
        print("\n--- Building Magnetometer Yaw LUT ---")
        lut = None

        # Prefer yawrtk data (has direct mag-RTK comparison, no time alignment needed)
        if yawrtk_data:
            print("  Using yawrtk data (preferred method)")
            lut = build_mag_yaw_lut_from_yawrtk(yawrtk_data, bin_size_deg=args.bin_size)
        elif ins_data and rtk_data:
            print("  Using INS + RTK data (fallback method)")
            lut = build_mag_yaw_lut(ins_data, rtk_data, bin_size_deg=args.bin_size)
        else:
            print("  [WARN] Missing data, cannot build LUT")
            print("  Need either: yawrtk data, or INS + RTK data")

        if lut:
            # Stats
            valid_bins = sum(1 for v in lut.values() if v[3] > 0)
            total_samples = sum(v[3] for v in lut.values())
            max_correction = max(abs(v[1]) for v in lut.values() if v[3] > 0)
            print(f"  Valid bins: {valid_bins}/{len(lut)}")
            print(f"  Total samples: {total_samples}")
            print(f"  Max correction: {max_correction:.2f}deg")

            # Generate C code
            c_code = generate_c_lut_code(lut, bin_size_deg=args.bin_size)
            if args.lut_out:
                with open(args.lut_out, 'w', encoding='utf-8') as f:
                    f.write(c_code)
                print(f"  C code saved to: {args.lut_out}")
            else:
                lut_out = os.path.join(data_dir, 'mag_yaw_lut.c')
                with open(lut_out, 'w', encoding='utf-8') as f:
                    f.write(c_code)
                print(f"  C code saved to: {lut_out}")

            # Generate Python LUT
            py_lut = generate_python_lut(lut, bin_size_deg=args.bin_size)
            py_lut_path = os.path.join(data_dir, 'mag_yaw_lut.py')
            with open(py_lut_path, 'w', encoding='utf-8') as f:
                f.write(f'#!/usr/bin/env python\n')
                f.write(f'# -*- coding: utf-8 -*-\n')
                f.write(f'# Magnetometer yaw LUT (auto-generated by mag_yaw_lut.py)\n')
                f.write(f'# Date: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write(f'# Resolution: {args.bin_size}deg, {len(py_lut)} entries\n')
                f.write(f'# Usage: corrected = mag_yaw + MAG_YAW_LUT[bin_idx]\n')
                f.write(f'#        bin_idx = int(mag_yaw % 360 / {args.bin_size})\n\n')
                f.write(f'MAG_YAW_LUT_BIN_SIZE = {args.bin_size}\n\n')
                f.write(f'MAG_YAW_LUT = {{\n')
                for k in sorted(py_lut.keys()):
                    f.write(f'    {k}: {py_lut[k]:+.4f},\n')
                f.write(f'}}\n')
            print(f"  Python LUT saved to: {py_lut_path}")

            # Plot
            if not args.no_plot:
                plot_path = os.path.join(data_dir, 'mag_yaw_lut.png')
                plot_mag_yaw_lut(lut, bin_size_deg=args.bin_size, save_path=plot_path)
        else:
            print("  [WARN] LUT build returned empty result")

    # ---- Corner cutting analysis ----
    if args.analyze_corner:
        print("\n--- Corner Cutting Analysis ---")
        if drive_data and follow_data:
            analysis = analyze_corner_cutting(drive_data, follow_data,
                                             curvature_threshold=args.curvature_thresh)
            if 'error' in analysis:
                print(f"  [ERROR] {analysis['error']}")
            else:
                print_analysis_report(analysis)
                if not args.no_plot:
                    plot_path = os.path.join(data_dir, 'corner_cutting_analysis.png')
                    plot_corner_cutting(analysis, save_path=plot_path)
        else:
            print("  [WARN] Missing drive or follow data")

    # ---- Three yaw comparison ----
    if args.show_yaw:
        print("\n--- Three Yaw Comparison ---")
        if ins_data:
            lut = None
            # Try loading existing LUT
            lut_path = os.path.join(data_dir, 'mag_yaw_lut.py')
            if os.path.exists(lut_path):
                import importlib.util
                spec = importlib.util.spec_from_file_location("mag_yaw_lut_mod", lut_path)
                lut_mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(lut_mod)
                py_lut_dict = lut_mod.MAG_YAW_LUT
                # Convert to standard format
                lut = {k: (0, v, 0, 1) for k, v in py_lut_dict.items()}
                print(f"  Loaded existing LUT: {lut_path}")

            if not args.no_plot:
                plot_path = os.path.join(data_dir, 'yaw_comparison.png')
                plot_three_yaw_display(ins_data, rtk_data, lut=lut,
                                      bin_size_deg=args.bin_size, save_path=plot_path)
        else:
            print("  [WARN] No INS data")

    print("\nDone!")


if __name__ == "__main__":
    main()
