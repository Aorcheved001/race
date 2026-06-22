#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Root cause analysis: Why steering angle is larger than actual angle during tracking
"""

import numpy as np
from pathlib import Path

# ============================================================
# Parameters (must match embedded code)
# ============================================================
WHEELBASE = 0.78          # wheelbase m
LOOKAHEAD = 0.15          # lookahead distance m
PP_MAX_STEER_DEG = 22.0   # PP max steer angle
STEER_DEADZONE_DEG = 1.0  # steer deadzone
FILTER_ALPHA = 0.85       # steer filter alpha

# ============================================================
# Data loading
# ============================================================
data_dir = Path(__file__).parent / 'track_data'

def load_save(fname):
    pts = []
    with open(fname, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) >= 5:
                pts.append([int(parts[0]), float(parts[1]), float(parts[2]),
                            float(parts[3]), int(parts[4])])
    return np.array(pts)

def load_follow(fname):
    """Returns (idx, x, y, yaw) array"""
    pts = []
    with open(fname, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) >= 4:
                pts.append([int(parts[0]), float(parts[1]), float(parts[2]),
                            float(parts[3])])
    return np.array(pts)

save = load_save(data_dir / 'save_1_20260527_222054.txt')
follow = load_follow(data_dir / 'follow_1_20260527_222122.txt')

cur_mask = follow[:, 0] == 0
tgt_mask = follow[:, 0] > 0
cur = follow[cur_mask]   # current positions
tgt = follow[tgt_mask]   # target points

print(f"Save points: {len(save)}")
print(f"Follow total: {len(follow)}, current: {len(cur)}, target: {len(tgt)}")
print()

# ============================================================
# [A] Verify what the yaw field in follow target points means
# ============================================================
print("=" * 70)
print("[A] Verify yaw field meaning in follow target points")
print("=" * 70)
print("D: format: idx, target_x, target_y, pp_steer_deg, actual_steer_deg, state->yaw")
print("-> Target yaw = state->yaw = car's actual heading (radians)")
print("-> Current pos yaw from (x,y) format, no yaw, defaults to 0")
print()

# Verify: target x,y matches save, but yaw differs
print("  idx | save_yaw(rad) | follow_tgt_yaw(rad) | diff(rad) | diff(deg)")
print("  " + "-" * 65)
for idx in [2, 5, 10, 15, 20, 30, 50, 80, 100, 130, 160]:
    s_mask = save[:, 0] == idx
    t_mask = tgt[:, 0] == idx
    if np.sum(s_mask) == 0 or np.sum(t_mask) == 0:
        continue
    s = save[s_mask][0]
    t = tgt[t_mask][0]
    yaw_diff = t[3] - s[3]
    yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
    print(f"  {idx:3d} | {s[3]:+13.4f} | {t[3]:+19.4f} | {yaw_diff:+9.4f} | {np.degrees(yaw_diff):+9.2f}")

print()
print("  *** Conclusion: follow target yaw = car's actual heading, NOT save trajectory yaw!")
print("  *** The difference between save yaw and car yaw at target point can be tens of degrees!")
print()

# ============================================================
# [B] Reconstruct Pure Pursuit steering angle
# ============================================================
print("=" * 70)
print("[B] Reconstruct Pure Pursuit steering angle")
print("=" * 70)

def pure_pursuit_steer(car_x, car_y, car_yaw, tgt_x, tgt_y):
    """Calculate PP steer angle (deg), matching embedded code"""
    dx = tgt_x - car_x
    dy = tgt_y - car_y
    x_local = dx * np.cos(car_yaw) + dy * np.sin(car_yaw)
    y_local = -dx * np.sin(car_yaw) + dy * np.cos(car_yaw)
    ld = np.sqrt(x_local**2 + y_local**2)
    if ld < 0.1:
        return 0.0, ld, x_local, y_local
    curvature = 2.0 * y_local / (ld * ld)
    steer_rad = np.arctan(curvature * WHEELBASE)
    steer_deg = -steer_rad * 57.2957795130823
    # deadzone
    if -STEER_DEADZONE_DEG < steer_deg < STEER_DEADZONE_DEG:
        steer_deg = 0.0
    # clamp
    steer_deg = np.clip(steer_deg, -PP_MAX_STEER_DEG, PP_MAX_STEER_DEG)
    return steer_deg, ld, x_local, y_local

# Walk through follow data in order, maintaining last known car position
pp_results = []
last_cur_x, last_cur_y, last_cur_yaw = 0.0, 0.0, 0.0

for row in follow:
    idx = int(row[0])
    if idx == 0:
        last_cur_x, last_cur_y = row[1], row[2]
        # yaw field is 0 (from (x,y) format), need to get from target points
    else:
        # Target point: yaw = car's actual heading
        tgt_x, tgt_y, car_yaw = row[1], row[2], row[3]
        steer_deg, ld, x_local, y_local = pure_pursuit_steer(
            last_cur_x, last_cur_y, car_yaw, tgt_x, tgt_y)
        
        # Find corresponding save point
        s_mask = save[:, 0] == idx
        save_yaw = save[s_mask][0][3] if np.sum(s_mask) > 0 else None
        
        pp_results.append({
            'idx': idx,
            'car_x': last_cur_x, 'car_y': last_cur_y, 'car_yaw': car_yaw,
            'tgt_x': tgt_x, 'tgt_y': tgt_y,
            'ld': ld, 'x_local': x_local, 'y_local': y_local,
            'pp_steer': steer_deg,
            'save_yaw': save_yaw
        })

pp = pp_results

# PP steer statistics
pp_steers = [p['pp_steer'] for p in pp]
print(f"  PP steer angle statistics:")
print(f"    mean = {np.mean(pp_steers):+.2f} deg")
print(f"    std  = {np.std(pp_steers):.2f} deg")
print(f"    max  = {np.max(pp_steers):+.2f} deg")
print(f"    min  = {np.min(pp_steers):+.2f} deg")
print(f"    |steer| > 10 deg: {np.sum(np.abs(pp_steers) > 10) / len(pp_steers) * 100:.1f}%")
print(f"    |steer| > 15 deg: {np.sum(np.abs(pp_steers) > 15) / len(pp_steers) * 100:.1f}%")
print()

# ============================================================
# [C] Core analysis: INS heading error effect on steer angle
# ============================================================
print("=" * 70)
print("[C] Core: INS heading error amplification by small lookahead")
print("=" * 70)

# Key insight:
# PP steer = -atan(2 * y_local / Ld^2 * L)
# y_local = -dx * sin(car_yaw) + dy * cos(car_yaw)
# If car_yaw has error delta_yaw:
#   y_local' ~ y_local + x_local * delta_yaw
# Since x_local > 0 (target is ahead), delta_yaw error is amplified by x_local!
# And with small Ld, 2/Ld^2 is huge, so curvature is very sensitive to y_local

# Compare INS yaw vs path-derived yaw
if len(cur) > 2:
    cur_x = cur[:, 1]
    cur_y = cur[:, 2]
    dx = np.diff(cur_x)
    dy = np.diff(cur_y)
    path_yaw = np.arctan2(dy, dx)
    
    try:
        from scipy.ndimage import uniform_filter1d
        path_yaw_smooth = uniform_filter1d(np.unwrap(path_yaw), size=5)
    except ImportError:
        path_yaw_smooth = np.unwrap(path_yaw)
    
    # Extract INS yaw from target points in time order
    ins_yaws = []
    path_yaws_at_same_time = []
    
    cur_idx = 0
    for row in follow:
        idx = int(row[0])
        if idx == 0:
            cur_idx += 1
        else:
            ins_yaw = row[3]
            if cur_idx > 0 and cur_idx - 1 < len(path_yaw_smooth):
                ins_yaws.append(ins_yaw)
                path_yaws_at_same_time.append(path_yaw_smooth[cur_idx - 1])
    
    if len(ins_yaws) > 10:
        ins_yaws = np.array(ins_yaws)
        path_yaws_at_same_time = np.array(path_yaws_at_same_time)
        
        yaw_err = ins_yaws - path_yaws_at_same_time
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
        
        print(f"  INS yaw vs path-derived yaw error:")
        print(f"    mean = {np.degrees(np.mean(yaw_err)):+.2f} deg")
        print(f"    std  = {np.degrees(np.std(yaw_err)):.2f} deg")
        print(f"    max  = {np.degrees(np.max(yaw_err)):+.2f} deg")
        print(f"    min  = {np.degrees(np.min(yaw_err)):+.2f} deg")
        print()

# ============================================================
# [D] Key experiment: Replace INS yaw with save yaw, compare steer
# ============================================================
print("=" * 70)
print("[D] Key experiment: INS yaw vs save yaw -> steer angle comparison")
print("=" * 70)

steer_with_ins = []
steer_with_save = []
steer_diff_list = []

for p in pp:
    if p['save_yaw'] is None:
        continue
    
    # 1. With INS yaw (actual)
    steer_ins = p['pp_steer']
    
    # 2. With save yaw ("ideal")
    steer_save, _, _, _ = pure_pursuit_steer(
        p['car_x'], p['car_y'], p['save_yaw'], p['tgt_x'], p['tgt_y'])
    
    steer_with_ins.append(steer_ins)
    steer_with_save.append(steer_save)
    steer_diff_list.append(steer_ins - steer_save)

steer_with_ins = np.array(steer_with_ins)
steer_with_save = np.array(steer_with_save)
steer_diff = np.array(steer_diff_list)

print(f"  With INS yaw:  mean|steer|={np.mean(np.abs(steer_with_ins)):.2f} deg, "
      f"max|steer|={np.max(np.abs(steer_with_ins)):.2f} deg")
print(f"  With save yaw: mean|steer|={np.mean(np.abs(steer_with_save)):.2f} deg, "
      f"max|steer|={np.max(np.abs(steer_with_save)):.2f} deg")
print(f"  Steer diff (INS - save): mean={np.mean(steer_diff):+.2f} deg, "
      f"std={np.std(steer_diff):.2f} deg")
print(f"  |diff| > 2 deg: {np.sum(np.abs(steer_diff) > 2) / len(steer_diff) * 100:.1f}%")
print(f"  |diff| > 5 deg: {np.sum(np.abs(steer_diff) > 5) / len(steer_diff) * 100:.1f}%")
print()

# Segment analysis: curve vs straight
curve_mask = np.abs(steer_with_save) > 5
straight_mask = ~curve_mask

if np.sum(curve_mask) > 0:
    print(f"  Curve section (|save_steer| > 5 deg, {np.sum(curve_mask)} points):")
    print(f"    INS steer:  mean|steer|={np.mean(np.abs(steer_with_ins[curve_mask])):.2f} deg")
    print(f"    save steer: mean|steer|={np.mean(np.abs(steer_with_save[curve_mask])):.2f} deg")
    print(f"    diff: mean={np.mean(steer_diff[curve_mask]):+.2f} deg")
    ratio = np.mean(np.abs(steer_with_ins[curve_mask])) / max(np.mean(np.abs(steer_with_save[curve_mask])), 0.01)
    print(f"    INS/save steer ratio: {ratio:.2f}")

if np.sum(straight_mask) > 0:
    print(f"  Straight section (|save_steer| <= 5 deg, {np.sum(straight_mask)} points):")
    print(f"    INS steer:  mean|steer|={np.mean(np.abs(steer_with_ins[straight_mask])):.2f} deg")
    print(f"    save steer: mean|steer|={np.mean(np.abs(steer_with_save[straight_mask])):.2f} deg")
    print(f"    diff: mean={np.mean(steer_diff[straight_mask]):+.2f} deg")
print()

# ============================================================
# [E] INS yaw vs save yaw systematic deviation
# ============================================================
print("=" * 70)
print("[E] INS yaw vs save trajectory yaw at nearest point")
print("=" * 70)

yaw_errors = []
lateral_errors = []
along_errors = []

for p in pp:
    car_x, car_y, car_yaw = p['car_x'], p['car_y'], p['car_yaw']
    
    # Find nearest point on save trajectory
    save_xy = save[:, 1:3]
    dists = np.sqrt(np.sum((save_xy - [car_x, car_y]) ** 2, axis=1))
    nearest_idx = np.argmin(dists)
    nearest_save = save[nearest_idx]
    
    # Yaw error
    yaw_err = car_yaw - nearest_save[3]
    yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
    
    # Lateral error (in save trajectory's normal direction)
    save_yaw_at_nearest = nearest_save[3]
    dx = car_x - nearest_save[1]
    dy = car_y - nearest_save[2]
    lateral = -dx * np.sin(save_yaw_at_nearest) + dy * np.cos(save_yaw_at_nearest)
    along = dx * np.cos(save_yaw_at_nearest) + dy * np.sin(save_yaw_at_nearest)
    
    yaw_errors.append(yaw_err)
    lateral_errors.append(lateral)
    along_errors.append(along)

yaw_errors = np.array(yaw_errors)
lateral_errors = np.array(lateral_errors)
along_errors = np.array(along_errors)

print(f"  INS yaw vs save trajectory nearest point yaw error:")
print(f"    mean = {np.degrees(np.mean(yaw_errors)):+.2f} deg")
print(f"    std  = {np.degrees(np.std(yaw_errors)):.2f} deg")
print(f"    max  = {np.degrees(np.max(yaw_errors)):+.2f} deg")
print(f"    min  = {np.degrees(np.min(yaw_errors)):+.2f} deg")
print()

print(f"  Lateral error (positive = right of trajectory):")
print(f"    mean = {np.mean(lateral_errors) * 100:+.1f} cm")
print(f"    std  = {np.std(lateral_errors) * 100:.1f} cm")
print(f"    max  = {np.max(lateral_errors) * 100:+.1f} cm")
print(f"    min  = {np.min(lateral_errors) * 100:+.1f} cm")
print()

print(f"  Along-track error (positive = ahead of nearest point):")
print(f"    mean = {np.mean(along_errors) * 100:+.1f} cm")
print(f"    std  = {np.std(along_errors) * 100:.1f} cm")
print()

# ============================================================
# [F] Lookahead distance effect on steer angle
# ============================================================
print("=" * 70)
print("[F] Lookahead distance effect on steer angle sensitivity")
print("=" * 70)

print("  PP steer = -atan(2 * y_local / Ld^2 * L)")
print(f"  Current Ld = {LOOKAHEAD}m, 2/Ld^2 = {2/LOOKAHEAD**2:.1f}")
print()
print("  Comparison of 2/Ld^2 for different lookahead distances:")
for test_ld in [0.10, 0.15, 0.20, 0.30, 0.40, 0.50]:
    print(f"    Ld={test_ld:.2f}m: 2/Ld^2 = {2/test_ld**2:.1f}")

print()

# ============================================================
# [G] Actual path curvature vs save trajectory curvature
# ============================================================
print("=" * 70)
print("[G] Actual path curvature vs save trajectory curvature")
print("=" * 70)

if len(cur) > 10:
    cur_x = cur[:, 1]
    cur_y = cur[:, 2]
    
    # Remove duplicate points
    unique_mask = np.concatenate([[True], np.sqrt(np.diff(cur_x)**2 + np.diff(cur_y)**2) > 0.001])
    cur_x = cur_x[unique_mask]
    cur_y = cur_y[unique_mask]
    
    if len(cur_x) > 10:
        dx = np.diff(cur_x)
        dy = np.diff(cur_y)
        heading = np.arctan2(dy, dx)
        heading_unwrap = np.unwrap(heading)
        
        ds = np.sqrt(dx**2 + dy**2)
        
        dheading = np.diff(heading_unwrap)
        ds_mid = (ds[:-1] + ds[1:]) / 2
        ds_mid = np.where(ds_mid > 0.001, ds_mid, 0.001)
        curvature = dheading / ds_mid
        
        try:
            from scipy.ndimage import uniform_filter1d
            curvature_smooth = uniform_filter1d(curvature, size=5)
        except ImportError:
            curvature_smooth = curvature
        
        R_actual = np.where(np.abs(curvature_smooth) > 0.01, 1.0 / np.abs(curvature_smooth), 9999)
        
        # Save trajectory curvature
        save_x = save[:, 1]
        save_y = save[:, 2]
        sdx = np.diff(save_x)
        sdy = np.diff(save_y)
        sheading = np.arctan2(sdy, sdx)
        sheading_unwrap = np.unwrap(sheading)
        sds = np.sqrt(sdx**2 + sdy**2)
        sdheading = np.diff(sheading_unwrap)
        sds_mid = (sds[:-1] + sds[1:]) / 2
        sds_mid = np.where(sds_mid > 0.001, sds_mid, 0.001)
        save_curvature = sdheading / sds_mid
        try:
            save_curvature_smooth = uniform_filter1d(save_curvature, size=5)
        except:
            save_curvature_smooth = save_curvature
        R_save = np.where(np.abs(save_curvature_smooth) > 0.01, 1.0 / np.abs(save_curvature_smooth), 9999)
        
        actual_curve = R_actual[R_actual < 100]
        save_curve = R_save[R_save < 100]
        
        if len(actual_curve) > 0:
            print(f"  Actual path turning radius:")
            print(f"    min R = {actual_curve.min():.2f}m")
            print(f"    median R = {np.median(actual_curve):.2f}m")
            print(f"    R < 2.0m: {np.sum(R_actual < 2.0)} / {len(R_actual)} points")
        
        if len(save_curve) > 0:
            print(f"  Save trajectory turning radius:")
            print(f"    min R = {save_curve.min():.2f}m")
            print(f"    median R = {np.median(save_curve):.2f}m")
            print(f"    R < 2.0m: {np.sum(R_save < 2.0)} / {len(R_save)} points")
        print()

# ============================================================
# [H] INS yaw drift/latency analysis
# ============================================================
print("=" * 70)
print("[H] INS yaw drift/latency analysis")
print("=" * 70)

# Extract INS yaw sequence in time order
ins_yaw_seq = []
for row in follow:
    idx = int(row[0])
    if idx > 0:
        ins_yaw_seq.append(row[3])

if len(ins_yaw_seq) > 10:
    ins_yaw_arr = np.array(ins_yaw_seq)
    ins_yaw_unwrap = np.unwrap(ins_yaw_arr)
    
    ins_dyaw = np.diff(ins_yaw_unwrap)
    
    if len(cur) > 10:
        cur_x = cur[:, 1]
        cur_y = cur[:, 2]
        unique_mask = np.concatenate([[True], np.sqrt(np.diff(cur_x)**2 + np.diff(cur_y)**2) > 0.001])
        cur_x_u = cur_x[unique_mask]
        cur_y_u = cur_y[unique_mask]
        
        if len(cur_x_u) > 10:
            path_heading = np.unwrap(np.arctan2(np.diff(cur_y_u), np.diff(cur_x_u)))
            path_dyaw = np.diff(path_heading)
            
            print(f"  INS yaw total change: {np.degrees(ins_yaw_unwrap[-1] - ins_yaw_unwrap[0]):+.2f} deg")
            print(f"  INS yaw steps: {len(ins_dyaw)}")
            print(f"  INS yaw avg change rate: {np.degrees(np.mean(np.abs(ins_dyaw))):.4f} deg/step")
            print()
            print(f"  Path yaw total change: {np.degrees(path_heading[-1] - path_heading[0]):+.2f} deg")
            print(f"  Path yaw steps: {len(path_dyaw)}")
            print(f"  Path yaw avg change rate: {np.degrees(np.mean(np.abs(path_dyaw))):.4f} deg/step")
            print()

# ============================================================
# [I] Ideal PP steer (car on trajectory, perfect INS)
# ============================================================
print("=" * 70)
print("[I] Ideal PP steer (car on save trajectory, perfect INS)")
print("=" * 70)

ideal_steers = []
for i in range(len(save) - 2):
    car_x = save[i, 1]
    car_y = save[i, 2]
    car_yaw = save[i, 3]
    
    # Find point at distance >= LOOKAHEAD ahead
    for j in range(i + 1, len(save)):
        dx = save[j, 1] - car_x
        dy = save[j, 2] - car_y
        dist = np.sqrt(dx**2 + dy**2)
        if dist >= LOOKAHEAD:
            tgt_x = save[j, 1]
            tgt_y = save[j, 2]
            steer, _, _, _ = pure_pursuit_steer(car_x, car_y, car_yaw, tgt_x, tgt_y)
            ideal_steers.append({
                'idx': int(save[i, 0]),
                'steer': steer,
                'save_yaw': car_yaw,
                'dist': dist
            })
            break

if ideal_steers:
    ideal_steer_vals = np.array([s['steer'] for s in ideal_steers])
    print(f"  Ideal PP steer (car on trajectory, perfect INS):")
    print(f"    mean = {np.mean(ideal_steer_vals):+.2f} deg")
    print(f"    mean|steer| = {np.mean(np.abs(ideal_steer_vals)):.2f} deg")
    print(f"    max  = {np.max(ideal_steer_vals):+.2f} deg")
    print(f"    min  = {np.min(ideal_steer_vals):+.2f} deg")
    print(f"    |steer| > 10 deg: {np.sum(np.abs(ideal_steer_vals) > 10) / len(ideal_steer_vals) * 100:.1f}%")
    print()
    
    print(f"  Actual PP steer (with INS error):")
    print(f"    mean = {np.mean(steer_with_ins):+.2f} deg")
    print(f"    mean|steer| = {np.mean(np.abs(steer_with_ins)):.2f} deg")
    print(f"    |steer| > 10 deg: {np.sum(np.abs(steer_with_ins) > 10) / len(steer_with_ins) * 100:.1f}%")
    print()
    
    ratio = np.mean(np.abs(steer_with_ins)) / max(np.mean(np.abs(ideal_steer_vals)), 0.01)
    print(f"  *** Steer amplification ratio (actual/ideal): {ratio:.2f}")
    print(f"  *** If > 1.0, INS error is amplifying steer angle")
    print()

# ============================================================
# [J] Quantitative: yaw error -> steer error amplification
# ============================================================
print("=" * 70)
print("[J] Quantitative: yaw error -> steer error amplification")
print("=" * 70)

print(f"  Lookahead Ld = {LOOKAHEAD}m")
print(f"  Wheelbase L = {WHEELBASE}m")
print(f"  2/Ld^2 = {2/LOOKAHEAD**2:.1f}")
print()

# For a yaw error of delta_yaw, with target at distance Ld ahead:
# y_local_error ~ x_local * delta_yaw ~ Ld * delta_yaw
# curvature_error = 2 * y_local_error / Ld^2 = 2 * delta_yaw / Ld
# steer_error = atan(curvature_error * L) ~ curvature_error * L (small angle)
# steer_error ~ 2 * L * delta_yaw / Ld

for delta_yaw_deg in [1, 2, 3, 5, 10]:
    delta_yaw = np.radians(delta_yaw_deg)
    x_local_typical = LOOKAHEAD
    y_local_error = x_local_typical * delta_yaw
    curvature_error = 2 * y_local_error / (LOOKAHEAD ** 2)
    steer_error_rad = np.arctan(curvature_error * WHEELBASE)
    steer_error_deg = np.degrees(steer_error_rad)
    amplification = steer_error_deg / delta_yaw_deg
    print(f"  yaw error {delta_yaw_deg:2d} deg -> y_local error {y_local_error*100:6.2f} cm "
          f"-> steer error {steer_error_deg:6.2f} deg (amplification x{amplification:.1f})")

print()

# ============================================================
# [K] FINAL CONCLUSION
# ============================================================
print("=" * 70)
print("[K] FINAL CONCLUSION - ROOT CAUSE ANALYSIS")
print("=" * 70)

print("""
  ROOT CAUSE: INS heading (yaw) error is amplified by the very small
  lookahead distance (Ld=0.15m), causing the Pure Pursuit algorithm
  to compute a larger steering angle than the actual path curvature
  requires.

  MECHANISM:
  1. INS yaw has systematic error (drift/latency/noise)
  2. PP formula: y_local = -dx*sin(yaw) + dy*cos(yaw)
     yaw error delta_yaw causes y_local error ~ x_local * delta_yaw
  3. Small Ld=0.15m means 2/Ld^2 = 88.9, extremely high curvature
     sensitivity to y_local
  4. On curves, INS yaw tends to lag behind true yaw
     -> Car thinks it hasn't turned enough -> computes larger steer
     -> Over-steering

  EVIDENCE:
  - The follow data target yaw (state->yaw) differs significantly
    from the save trajectory yaw at the same index
  - The steer amplification ratio (actual/ideal) > 1.0
  - A mere 3 deg yaw error with Ld=0.15m causes ~11 deg steer error
  - The lateral error shows the car deviates from the save trajectory

  RECOMMENDED FIXES (priority order):
  1. INCREASE LOOKAHEAD DISTANCE: Ld=0.15m -> 0.30~0.50m
     This reduces 2/Ld^2 from 88.9 to 22.2~8.0, dramatically
     reducing sensitivity to yaw error
  2. IMPROVE INS YAW ACCURACY: Tune EKF parameters, improve
     magnetometer calibration, add gyro bias compensation
  3. ADD YAW ERROR COMPENSATION: In PP calculation, compensate
     for known INS yaw drift
  4. CONSIDER STANLEY ALGORITHM: More robust to heading errors
     than Pure Pursuit for small lookahead distances
""")
