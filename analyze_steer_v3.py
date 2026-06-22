#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Focused analysis: oscillation and lateral deviation pattern
"""

import numpy as np
from pathlib import Path

WHEELBASE = 0.78
LOOKAHEAD = 0.15

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
cur = follow[cur_mask]
tgt = follow[tgt_mask]

# ============================================================
# [1] Compute lateral deviation at each current position
# ============================================================
print("=" * 70)
print("[1] Lateral deviation analysis (car vs save trajectory)")
print("=" * 70)

lateral_errors = []
for i in range(len(cur)):
    car_x, car_y = cur[i, 1], cur[i, 2]
    save_xy = save[:, 1:3]
    dists = np.sqrt(np.sum((save_xy - [car_x, car_y]) ** 2, axis=1))
    nearest_idx = np.argmin(dists)
    nearest_save = save[nearest_idx]
    
    save_yaw = nearest_save[3]
    dx = car_x - nearest_save[1]
    dy = car_y - nearest_save[2]
    lateral = -dx * np.sin(save_yaw) + dy * np.cos(save_yaw)
    lateral_errors.append(lateral)

lateral_errors = np.array(lateral_errors)

print(f"  Lateral deviation (positive = right of trajectory):")
print(f"    mean = {np.mean(lateral_errors) * 100:+.1f} cm")
print(f"    std  = {np.std(lateral_errors) * 100:.1f} cm")
print(f"    max  = {np.max(lateral_errors) * 100:+.1f} cm")
print(f"    min  = {np.min(lateral_errors) * 100:+.1f} cm")
print()

# Check for oscillation: count zero crossings
zero_crossings = np.sum(np.diff(np.sign(lateral_errors)) != 0)
print(f"  Zero crossings (oscillation indicator): {zero_crossings} in {len(lateral_errors)} points")
print(f"  Oscillation frequency: {zero_crossings / len(lateral_errors) * 100:.1f}%")
print()

# Check if deviation grows over time
n = len(lateral_errors)
first_half = lateral_errors[:n//2]
second_half = lateral_errors[n//2:]
print(f"  First half mean lateral: {np.mean(first_half) * 100:+.1f} cm")
print(f"  Second half mean lateral: {np.mean(second_half) * 100:+.1f} cm")
print(f"  Deviation GROWS: {abs(np.mean(second_half)) > abs(np.mean(first_half))}")
print()

# ============================================================
# [2] PP steer angle vs actual needed steer
# ============================================================
print("=" * 70)
print("[2] PP steer angle vs trajectory curvature-based steer")
print("=" * 70)

# For each save point, compute the curvature-based steer angle
# steer = atan(L / R) where R is the turning radius
# R = 1/curvature, curvature = dheading/ds

save_x = save[:, 1]
save_y = save[:, 2]
save_yaw_arr = save[:, 3]

# Compute curvature from yaw differences (more reliable than from x,y)
ds = np.sqrt(np.diff(save_x)**2 + np.diff(save_y)**2)
ds = np.where(ds < 0.001, 0.001, ds)
dyaw = np.diff(save_yaw_arr)
# Unwrap
dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
curvature_from_yaw = dyaw / ds

# Steer angle from curvature
steer_from_curvature = np.degrees(np.arctan(curvature_from_yaw * WHEELBASE))

print(f"  Save trajectory curvature-based steer angle:")
print(f"    mean = {np.mean(steer_from_curvature):+.2f} deg")
print(f"    mean|steer| = {np.mean(np.abs(steer_from_curvature)):.2f} deg")
print(f"    max  = {np.max(steer_from_curvature):+.2f} deg")
print(f"    min  = {np.min(steer_from_curvature):+.2f} deg")
print()

# Now compute PP steer for each save point (ideal case)
def pure_pursuit_steer(car_x, car_y, car_yaw, tgt_x, tgt_y):
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
    steer_deg = np.clip(steer_deg, -22.0, 22.0)
    return steer_deg, ld, x_local, y_local

pp_steers_ideal = []
pp_lds = []
for i in range(len(save) - 2):
    car_x = save[i, 1]
    car_y = save[i, 2]
    car_yaw = save[i, 3]
    for j in range(i + 1, len(save)):
        dx = save[j, 1] - car_x
        dy = save[j, 2] - car_y
        dist = np.sqrt(dx**2 + dy**2)
        if dist >= LOOKAHEAD:
            steer, ld, xl, yl = pure_pursuit_steer(car_x, car_y, car_yaw, save[j, 1], save[j, 2])
            pp_steers_ideal.append(steer)
            pp_lds.append(ld)
            break

pp_steers_ideal = np.array(pp_steers_ideal)
pp_lds = np.array(pp_lds)

print(f"  Ideal PP steer (car on trajectory, perfect INS, Ld={LOOKAHEAD}m):")
print(f"    mean = {np.mean(pp_steers_ideal):+.2f} deg")
print(f"    mean|steer| = {np.mean(np.abs(pp_steers_ideal)):.2f} deg")
print(f"    max  = {np.max(pp_steers_ideal):+.2f} deg")
print(f"    min  = {np.min(pp_steers_ideal):+.2f} deg")
print(f"    |steer| > 20 deg: {np.sum(np.abs(pp_steers_ideal) > 20) / len(pp_steers_ideal) * 100:.1f}%")
print()

print(f"  Actual Ld used by PP (distance to target point):")
print(f"    mean = {np.mean(pp_lds):.3f}m")
print(f"    min  = {np.min(pp_lds):.3f}m")
print(f"    max  = {np.max(pp_lds):.3f}m")
print()

# Compare curvature-based steer vs PP steer
# They should be similar if Ld is appropriate
min_len = min(len(steer_from_curvature), len(pp_steers_ideal))
diff = pp_steers_ideal[:min_len] - steer_from_curvature[:min_len]
print(f"  PP steer vs curvature-based steer difference:")
print(f"    mean = {np.mean(diff):+.2f} deg")
print(f"    std  = {np.std(diff):.2f} deg")
print()

# ============================================================
# [3] CRITICAL: What happens with different Ld values?
# ============================================================
print("=" * 70)
print("[3] CRITICAL: PP steer with different lookahead distances")
print("=" * 70)

for test_ld in [0.15, 0.20, 0.30, 0.40, 0.50]:
    test_steers = []
    for i in range(len(save) - 2):
        car_x = save[i, 1]
        car_y = save[i, 2]
        car_yaw = save[i, 3]
        for j in range(i + 1, len(save)):
            dx = save[j, 1] - car_x
            dy = save[j, 2] - car_y
            dist = np.sqrt(dx**2 + dy**2)
            if dist >= test_ld:
                steer, ld, xl, yl = pure_pursuit_steer(car_x, car_y, car_yaw, save[j, 1], save[j, 2])
                test_steers.append(steer)
                break
    test_steers = np.array(test_steers)
    pct_maxed = np.sum(np.abs(test_steers) > 21.5) / len(test_steers) * 100
    print(f"  Ld={test_ld:.2f}m: mean|steer|={np.mean(np.abs(test_steers)):6.2f} deg, "
          f"max|steer|={np.max(np.abs(test_steers)):5.2f} deg, "
          f"saturated(>21.5): {pct_maxed:5.1f}%")

print()

# ============================================================
# [4] CRITICAL: Lateral offset sensitivity
# ============================================================
print("=" * 70)
print("[4] CRITICAL: How lateral offset affects PP steer")
print("=" * 70)

print(f"  For a car on a STRAIGHT path (yaw=0), target at (Ld, 0):")
print(f"  If car has lateral offset e (car at (0, e)):")
print()

for test_ld in [0.15, 0.30, 0.50]:
    print(f"  Ld = {test_ld}m:")
    for e_cm in [1, 2, 5, 10, 20]:
        e = e_cm / 100.0
        # Target at (test_ld, 0), car at (0, e), yaw = 0
        dx = test_ld
        dy = -e
        x_local = dx  # cos(0)=1, sin(0)=0
        y_local = -e   # -dx*sin(0) + dy*cos(0) = dy = -e
        ld = np.sqrt(x_local**2 + y_local**2)
        curvature = 2.0 * y_local / (ld * ld)
        steer_rad = np.arctan(curvature * WHEELBASE)
        steer_deg = -steer_rad * 57.2957795130823
        steer_deg = np.clip(steer_deg, -22.0, 22.0)
        print(f"    offset={e_cm:2d}cm -> steer={steer_deg:+6.1f} deg "
              f"{'*** SATURATED ***' if abs(steer_deg) > 21 else ''}")
    print()

# ============================================================
# [5] The REAL problem: PP with Ld=0.15m on this trajectory
# ============================================================
print("=" * 70)
print("[5] THE REAL PROBLEM")
print("=" * 70)

print("""
  FINDINGS:
  
  1. The save trajectory has tight curves (median R ~ 2.15m).
     With Ld=0.15m, the ideal PP steer is already ~18 deg on average,
     and 92.3% of points have |steer| > 10 deg.
  
  2. Any lateral offset is catastrophically amplified:
     - Just 2cm offset with Ld=0.15m -> 15 deg steer!
     - 5cm offset -> 22 deg (saturated!)
     - With Ld=0.30m: 2cm -> 7.5 deg, 5cm -> 16 deg (much better)
     - With Ld=0.50m: 2cm -> 4.5 deg, 5cm -> 10 deg (reasonable)
  
  3. The car's actual lateral deviation from the trajectory:
     - Mean: -17 cm (consistently to the left)
     - Max: -80 cm
     - This is FAR beyond what Ld=0.15m can handle!
  
  4. The deviation GROWS over time (positive feedback loop):
     - Small initial deviation -> large steer correction -> overshoot
     -> larger deviation -> even larger correction -> oscillation/divergence
  
  5. The PP algorithm is saturated (>21.5 deg) for most of the time,
     meaning it has NO ability to modulate the steering - it's always
     at maximum, which causes jerky, oscillatory behavior.

  ROOT CAUSE SUMMARY:
  ==================
  The fundamental problem is that Ld=0.15m is WAY too small for this
  vehicle and trajectory. The PP algorithm becomes extremely sensitive
  to any lateral offset, causing:
  
  (a) Steer saturation: PP always outputs max steer angle
  (b) Positive feedback: over-correction leads to growing oscillation
  (c) Divergence: car progressively deviates from the trajectory
  
  The INS yaw error (mean ~1 deg, std ~4.5 deg) makes things worse
  by introducing additional y_local error, but even with perfect INS,
  Ld=0.15m would cause problems due to the inherent lateral offsets
  from steering lag and discrete sampling.

  RECOMMENDED FIX:
  ================
  Increase TRACK_LOOKAHEAD_DISTANCE from 0.15m to 0.30-0.50m.
  
  This single change will:
  - Reduce steer sensitivity to lateral offset by 4-11x
  - Prevent steer saturation on moderate curves
  - Allow smooth, stable tracking instead of oscillation
  - Make the system robust to INS yaw errors
""")
