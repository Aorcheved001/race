#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analyze latest track data - auto-detect latest files, discard reverse section in follow analysis."""

import numpy as np
import os
import glob
import re

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'track_data')

# === Auto-detect latest files ===
def find_latest_files():
    """Find the latest save and follow files, with smart matching."""
    save_files = sorted(glob.glob(os.path.join(DATA_DIR, 'save_*.txt')))
    follow_files = sorted(glob.glob(os.path.join(DATA_DIR, 'follow_*.txt')))
    
    if not save_files:
        print("ERROR: No save files found!")
        return None, None
    if not follow_files:
        print("ERROR: No follow files found!")
        return None, None
    
    follow_file = follow_files[-1]
    
    # Try to find matching save by timestamp
    follow_ts = re.search(r'(\d{8}_\d{6})', os.path.basename(follow_file))
    save_file = None
    if follow_ts:
        ts = follow_ts.group(1)
        matching_save = os.path.join(DATA_DIR, f'save_1_{ts}.txt')
        if os.path.exists(matching_save):
            save_file = matching_save
    
    # If no timestamp match, use latest save
    if save_file is None:
        save_file = save_files[-1]
    
    # Quick check: does the save file have enough points for the follow data?
    # Read max target index from follow file (first pass - just check the max idx)
    max_tgt_idx = 0
    with open(follow_file, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            parts = line.split(',')
            if len(parts) >= 1:
                try:
                    idx = int(parts[0])
                    if idx > max_tgt_idx:
                        max_tgt_idx = idx
                except ValueError:
                    pass
    
    # Read save point count
    save_count = 0
    with open(save_file, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            parts = line.split(',')
            if len(parts) >= 5:
                try:
                    save_count = max(save_count, int(parts[0]))
                except ValueError:
                    pass
    
    # If save doesn't have enough points, try to find a better match
    if max_tgt_idx > save_count:
        print(f'  WARNING: Follow max target idx={max_tgt_idx} > save points={save_count}')
        print(f'  Looking for a save file with more points...')
        for sf in reversed(save_files):
            sc = 0
            with open(sf, encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'): continue
                    parts = line.split(',')
                    if len(parts) >= 5:
                        try:
                            sc = max(sc, int(parts[0]))
                        except ValueError:
                            pass
            if sc >= max_tgt_idx:
                save_file = sf
                save_count = sc
                print(f'  Found better save: {os.path.basename(sf)} ({sc} points)')
                break
        
        if save_count < max_tgt_idx:
            print(f'  WARNING: No save file found with enough points (need {max_tgt_idx})')
            print(f'  Using {os.path.basename(save_file)} ({save_count} points) - analysis may be incomplete')
    
    return save_file, follow_file

save_file, follow_file = find_latest_files()
if save_file is None:
    exit(1)

print(f'Save file: {os.path.basename(save_file)}')
print(f'Follow file: {os.path.basename(follow_file)}')

# === Load data ===
save = []
with open(save_file, encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'): continue
        parts = line.split(',')
        if len(parts) >= 5:
            save.append([int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), int(parts[4])])
save = np.array(save)

follow = []
with open(follow_file, encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'): continue
        parts = line.split(',')
        if len(parts) >= 4:
            follow.append([int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])])
follow = np.array(follow)

cur = follow[follow[:,0]==0]
tgt = follow[follow[:,0]>0]

save_xy = save[:, 1:3]
save_yaw = save[:, 3]
save_dir = save[:, 4]

save_ds = np.sqrt(np.diff(save_xy[:,0])**2 + np.diff(save_xy[:,1])**2)
save_dist = np.concatenate([[0], np.cumsum(save_ds)])

WHEELBASE = 0.78
LD = 0.50
MAX_STEER_DEG = 22.0

# === Find direction change point ===
fwd_count = int(np.sum(save_dir == 1))
rev_count = int(np.sum(save_dir == 0))
dir_change_idx = fwd_count  # index where direction changes (1-based)

print('='*70)
print('[Basic Info]')
print('='*70)
print(f'Save: {len(save)} pts, path {save_dist[-1]:.2f}m')
print(f'  X=[{save[:,1].min():.3f}, {save[:,1].max():.3f}], Y=[{save[:,2].min():.3f}, {save[:,2].max():.3f}]')
print(f'  Fwd={fwd_count}, Rev={rev_count}')
if rev_count > 0:
    print(f'  Direction change at idx {dir_change_idx} -> {dir_change_idx+1}')
    print(f'  Last fwd point: idx={dir_change_idx}, ({save[dir_change_idx-1,1]:.3f}, {save[dir_change_idx-1,2]:.3f})')
    print(f'  First rev point: idx={dir_change_idx+1}, ({save[dir_change_idx,1]:.3f}, {save[dir_change_idx,2]:.3f})')
print(f'Follow: {len(follow)} pts (pos={len(cur)}, tgt={len(tgt)})')
if len(cur)>1:
    cur_ds = np.sqrt(np.diff(cur[:,1])**2 + np.diff(cur[:,2])**2)
    print(f'  Driven path: {np.sum(cur_ds):.2f}m')
print(f'  Target idx: [{int(tgt[:,0].min())}, {int(tgt[:,0].max())}] / {len(save)}')

# === Discard reverse section in follow analysis ===
# Only analyze car positions where the target index is in the forward section
tgt_idx_arr_all = tgt[:, 0].astype(int)
fwd_only_mask = tgt_idx_arr_all <= fwd_count

if rev_count > 0 and not np.all(fwd_only_mask):
    first_rev_target = np.where(~fwd_only_mask)[0][0]
    print(f'\n  *** REVERSE SECTION DETECTED ***')
    print(f'  Target jumps to reverse section at follow target line {first_rev_target}')
    print(f'  Target idx at jump: {tgt_idx_arr_all[first_rev_target-1]} -> {tgt_idx_arr_all[first_rev_target]}')
    print(f'  Discarding reverse section for analysis (keeping {np.sum(fwd_only_mask)}/{len(tgt)} target points)')
    
    # Build forward-only follow array
    # Keep: (1) all target lines with idx <= fwd_count, (2) car position lines whose nearest preceding target is forward
    fwd_follow_indices = []
    for i in range(len(follow)):
        if follow[i, 0] > 0:
            # Target line - keep if in forward section
            if follow[i, 0] <= fwd_count:
                fwd_follow_indices.append(i)
        else:
            # Car position line - keep if nearest preceding target is forward
            for j in range(i-1, -1, -1):
                if follow[j, 0] > 0:
                    if follow[j, 0] <= fwd_count:
                        fwd_follow_indices.append(i)
                    break
    
    fwd_follow_indices = sorted(set(fwd_follow_indices))
    follow_fwd = follow[fwd_follow_indices]
    cur_fwd = follow_fwd[follow_fwd[:,0]==0]
    tgt_fwd = follow_fwd[follow_fwd[:,0]>0]
    
    print(f'  Forward-only follow: {len(follow_fwd)} pts (pos={len(cur_fwd)}, tgt={len(tgt_fwd)})')
    
    # Use forward-only data for analysis
    cur_analysis = cur_fwd
    tgt_analysis = tgt_fwd
    follow_analysis = follow_fwd
else:
    cur_analysis = cur
    tgt_analysis = tgt
    follow_analysis = follow
    print(f'  No reverse section detected, analyzing all data')

# === 1. Yaw-decomposed lateral error (forward section only) ===
print()
print('='*70)
print('[1] Cross-track + Along-track error (yaw-decomposed, forward section only)')
print('='*70)

errors_ct = []
errors_at = []
distances = []
seg_idx_arr = []
seg_dir_arr = []

for p in cur_analysis[:, 1:3]:
    dists = np.sqrt(np.sum((save_xy - p)**2, axis=1))
    nearest_idx = np.argmin(dists)
    yaw_ref = save_yaw[nearest_idx]
    dx = p[0] - save_xy[nearest_idx, 0]
    dy = p[1] - save_xy[nearest_idx, 1]
    e_along = dx * np.cos(yaw_ref) + dy * np.sin(yaw_ref)
    e_cross = -dx * np.sin(yaw_ref) + dy * np.cos(yaw_ref)
    errors_ct.append(e_cross)
    errors_at.append(e_along)
    distances.append(dists[nearest_idx])
    seg_idx_arr.append(nearest_idx)
    seg_dir_arr.append(save_dir[nearest_idx])

errors_ct = np.array(errors_ct)
errors_at = np.array(errors_at)
distances = np.array(distances)
seg_idx_arr = np.array(seg_idx_arr)
seg_dir_arr = np.array(seg_dir_arr)

mean_ct = np.mean(np.abs(errors_ct))
mean_ct_signed = np.mean(errors_ct)
direction = "LEFT" if mean_ct_signed > 0 else "RIGHT"
print(f'  Cross-track (ct):')
print(f'    mean|ct| = {mean_ct*100:.2f} cm')
print(f'    RMS ct   = {np.sqrt(np.mean(errors_ct**2))*100:.2f} cm')
print(f'    max|ct|  = {np.max(np.abs(errors_ct))*100:.2f} cm')
print(f'    median|ct| = {np.median(np.abs(errors_ct))*100:.2f} cm')
print(f'    std = {np.std(errors_ct)*100:.2f} cm')
print(f'    bias direction: {direction} ({mean_ct_signed*100:.2f}cm)')
print(f'    <5cm:  {np.sum(np.abs(errors_ct)<0.05)/len(errors_ct)*100:.1f}%')
print(f'    <10cm: {np.sum(np.abs(errors_ct)<0.10)/len(errors_ct)*100:.1f}%')
print(f'    <20cm: {np.sum(np.abs(errors_ct)<0.20)/len(errors_ct)*100:.1f}%')

print()
print(f'  Along-track (at):')
print(f'    mean at  = {np.mean(errors_at)*100:.2f} cm (+ = ahead)')
print(f'    max ahead  = {np.max(errors_at)*100:.2f} cm')
print(f'    max behind = {np.min(errors_at)*100:.2f} cm')
print(f'    std = {np.std(errors_at)*100:.2f} cm')

# === 2. Segment error by path distance (forward section only) ===
print()
print('='*70)
print('[2] Segment error by path distance (forward section only)')
print('='*70)
n = len(errors_ct)
third = n // 3

for label, sl in [('First 1/3', slice(0, third)), ('Mid 1/3', slice(third, 2*third)), ('Last 1/3', slice(2*third, n))]:
    ct = errors_ct[sl]
    at = errors_at[sl]
    idx_range = seg_idx_arr[sl]
    path_d = save_dist[idx_range]
    ct_dir = "L" if np.mean(ct) > 0 else "R"
    print(f'  {label} (path {path_d.min():.1f}~{path_d.max():.1f}m):')
    print(f'    ct: mean|ct|={np.mean(np.abs(ct))*100:.1f}cm, max={np.max(np.abs(ct))*100:.1f}cm, dir={ct_dir}')
    print(f'    at: mean={np.mean(at)*100:.1f}cm, ahead={np.max(at)*100:.1f}cm, behind={np.min(at)*100:.1f}cm')

# === 3. Yaw drift from D: lines (forward section) ===
print()
print('='*70)
print('[3] Yaw drift: car yaw (from D: lines) vs save reference yaw')
print('='*70)
# D: lines have idx>0, x=target_x, y=target_y, yaw=car_yaw
tgt_indices = tgt_analysis[:, 0].astype(int)
car_yaws = tgt_analysis[:, 3]  # car's actual yaw from D: line

valid = (tgt_indices >= 1) & (tgt_indices <= len(save))
save_yaws_at_tgt = np.zeros(len(tgt_analysis))
for i in range(len(tgt_analysis)):
    if valid[i]:
        save_yaws_at_tgt[i] = save_yaw[tgt_indices[i]-1]

yaw_errors = car_yaws[valid] - save_yaws_at_tgt[valid]
yaw_errors = (yaw_errors + np.pi) % (2*np.pi) - np.pi

n_valid = len(yaw_errors)
if n_valid > 0:
    q1 = n_valid // 4
    q2 = n_valid // 2
    q3 = 3 * n_valid // 4

    print(f'  Yaw error (car_yaw - save_yaw_at_target):')
    print(f'    mean  = {np.degrees(np.mean(yaw_errors)):+.2f} deg')
    print(f'    std   = {np.degrees(np.std(yaw_errors)):.2f} deg')
    print(f'    max|err| = {np.degrees(np.max(np.abs(yaw_errors))):.2f} deg')
    print(f'  By quarter:')
    print(f'    Q1: mean={np.degrees(np.mean(yaw_errors[:q1])):+.2f}deg, max|err|={np.degrees(np.max(np.abs(yaw_errors[:q1]))):.2f}deg')
    print(f'    Q2: mean={np.degrees(np.mean(yaw_errors[q1:q2])):+.2f}deg, max|err|={np.degrees(np.max(np.abs(yaw_errors[q1:q2]))):.2f}deg')
    print(f'    Q3: mean={np.degrees(np.mean(yaw_errors[q2:q3])):+.2f}deg, max|err|={np.degrees(np.max(np.abs(yaw_errors[q2:q3]))):.2f}deg')
    print(f'    Q4: mean={np.degrees(np.mean(yaw_errors[q3:])):+.2f}deg, max|err|={np.degrees(np.max(np.abs(yaw_errors[q3:]))):.2f}deg')

    if len(tgt_analysis) > 10:
        start_yaw_err = np.degrees(np.mean(yaw_errors[:n_valid//10]))
        end_yaw_err = np.degrees(np.mean(yaw_errors[-n_valid//10:]))
        print(f'    trend: start={start_yaw_err:+.2f}deg -> end={end_yaw_err:+.2f}deg')
        drift = end_yaw_err - start_yaw_err
        print(f'    total drift: {drift:+.2f}deg')
else:
    print(f'  No valid yaw data')

# === 4. Target index progression ===
print()
print('='*70)
print('[4] Target index progression')
print('='*70)
tgt_idx_arr = tgt_analysis[:, 0].astype(int)
unique_tgt_idx = np.unique(tgt_idx_arr)
print(f'  Unique target indices: {len(unique_tgt_idx)} / {len(save)} save points')
print(f'  Target index range: [{unique_tgt_idx.min()}, {unique_tgt_idx.max()}]')

idx_diffs = np.diff(tgt_idx_arr)
backward_count = np.sum(idx_diffs < 0)
same_count = np.sum(idx_diffs == 0)
forward_count = np.sum(idx_diffs > 0)
print(f'  Index transitions: forward={forward_count}, same={same_count}, backward={backward_count}')

max_tgt = int(tgt_idx_arr.max())
print(f'  Max target index: {max_tgt} / {len(save)} (fwd section ends at {fwd_count})')
if max_tgt > fwd_count:
    print(f'  ? Car entered reverse section (target jumped to {max_tgt})')
    # Check for the big jump bug (e.g. 163->199)
    big_jumps = np.where(idx_diffs > 5)[0]
    if len(big_jumps) > 0:
        for j in big_jumps:
            print(f'  ? BIG JUMP: target {tgt_idx_arr[j]} -> {tgt_idx_arr[j+1]} (diff={idx_diffs[j]})')
            print(f'     This is the "skip reverse section" bug in find_lookahead_point()')
            print(f'     Fix: return direction change point immediately instead of skipping')
else:
    print(f'  ? Car did NOT reach reverse section')

# Also check ALL target data (including reverse) for the jump bug
if rev_count > 0:
    all_tgt_idx = tgt[:, 0].astype(int)
    all_idx_diffs = np.diff(all_tgt_idx)
    big_jumps_all = np.where(all_idx_diffs > 5)[0]
    if len(big_jumps_all) > 0:
        print(f'\n  *** DIRECTION SWITCH BUG DETECTED (in raw data) ***')
        for j in big_jumps_all:
            print(f'  ? Target index jumps {all_tgt_idx[j]} -> {all_tgt_idx[j+1]} (diff={all_idx_diffs[j]})')
            print(f'     Root cause: find_lookahead_point() skips all reverse points')
            print(f'     and falls through to follow_abs_index = stored_points')

# === 5. Car-to-target distance (effective lookahead, forward section) ===
print()
print('='*70)
print('[5] Car-to-target distance (effective Ld, forward section)')
print('='*70)

car_target_dists = []
for i in range(len(follow_analysis)):
    if follow_analysis[i, 0] == 0:
        for j in range(i-1, -1, -1):
            if follow_analysis[j, 0] > 0:
                dx = follow_analysis[i, 1] - follow_analysis[j, 1]
                dy = follow_analysis[i, 2] - follow_analysis[j, 2]
                car_target_dists.append(np.sqrt(dx*dx + dy*dy))
                break

car_target_dists = np.array(car_target_dists)
if len(car_target_dists) > 0:
    print(f'  Car-to-target distance:')
    print(f'    mean  = {np.mean(car_target_dists)*100:.1f}cm')
    print(f'    std   = {np.std(car_target_dists)*100:.1f}cm')
    print(f'    min   = {np.min(car_target_dists)*100:.1f}cm')
    print(f'    max   = {np.max(car_target_dists)*100:.1f}cm')
    print(f'    < 20cm: {np.sum(car_target_dists < 0.20)}/{len(car_target_dists)} ({np.sum(car_target_dists<0.20)/len(car_target_dists)*100:.1f}%)')
    print(f'    < 35cm (Ld): {np.sum(car_target_dists < 0.35)}/{len(car_target_dists)} ({np.sum(car_target_dists<0.35)/len(car_target_dists)*100:.1f}%)')
    print(f'    > 50cm: {np.sum(car_target_dists > 0.50)}/{len(car_target_dists)} ({np.sum(car_target_dists>0.50)/len(car_target_dists)*100:.1f}%)')
    print(f'    > 100cm: {np.sum(car_target_dists > 1.00)}/{len(car_target_dists)} ({np.sum(car_target_dists>1.00)/len(car_target_dists)*100:.1f}%)')

# === 6. Reconstructed PP steer (forward section) ===
print()
print('='*70)
print('[6] Reconstructed PP steer analysis (forward section)')
print('='*70)

paired_data = []
for i in range(len(follow_analysis)):
    if follow_analysis[i, 0] > 0:
        for j in range(i-1, -1, -1):
            if follow_analysis[j, 0] == 0:
                cx, cy = follow_analysis[j, 1], follow_analysis[j, 2]
                car_yaw = follow_analysis[i, 3]
                tx, ty = follow_analysis[i, 1], follow_analysis[i, 2]
                tidx = int(follow_analysis[i, 0])
                paired_data.append((tidx, cx, cy, car_yaw, tx, ty))
                break

if paired_data:
    paired = np.array(paired_data)
    tidx = paired[:, 0].astype(int)
    cx = paired[:, 1]
    cy = paired[:, 2]
    cyaw = paired[:, 3]
    tx = paired[:, 4]
    ty = paired[:, 5]
    
    dx = tx - cx
    dy = ty - cy
    x_local = dx * np.cos(cyaw) + dy * np.sin(cyaw)
    y_local = -dx * np.sin(cyaw) + dy * np.cos(cyaw)
    ld = np.sqrt(x_local**2 + y_local**2)
    ld = np.where(ld < 0.1, 0.1, ld)
    curvature = 2.0 * y_local / (ld * ld)
    steer_rad = np.arctan(curvature * WHEELBASE)
    steer_deg = -np.degrees(steer_rad)
    steer_deg = np.clip(steer_deg, -MAX_STEER_DEG, MAX_STEER_DEG)
    
    total = len(steer_deg)
    print(f'  Reconstructed PP steer ({total} samples):')
    print(f'    mean  = {np.mean(steer_deg):+.2f} deg')
    print(f'    std   = {np.std(steer_deg):.2f} deg')
    print(f'    min   = {np.min(steer_deg):+.2f} deg')
    print(f'    max   = {np.max(steer_deg):+.2f} deg')
    
    sat_20 = np.sum(np.abs(steer_deg) > 20)
    sat_15 = np.sum(np.abs(steer_deg) > 15)
    sat_10 = np.sum(np.abs(steer_deg) > 10)
    print(f'    |steer| > 20 deg: {sat_20}/{total} ({sat_20/total*100:.1f}%)')
    print(f'    |steer| > 15 deg: {sat_15}/{total} ({sat_15/total*100:.1f}%)')
    print(f'    |steer| > 10 deg: {sat_10}/{total} ({sat_10/total*100:.1f}%)')
    
    n_p = len(steer_deg)
    pq1 = n_p // 4
    pq2 = n_p // 2
    pq3 = 3 * n_p // 4
    print(f'  Steer by quarter:')
    print(f'    Q1: mean={np.mean(steer_deg[:pq1]):+.2f}, sat>20={np.sum(np.abs(steer_deg[:pq1])>20)}/{pq1}')
    print(f'    Q2: mean={np.mean(steer_deg[pq1:pq2]):+.2f}, sat>20={np.sum(np.abs(steer_deg[pq1:pq2])>20)}/{pq2-pq1}')
    print(f'    Q3: mean={np.mean(steer_deg[pq2:pq3]):+.2f}, sat>20={np.sum(np.abs(steer_deg[pq2:pq3])>20)}/{pq3-pq2}')
    print(f'    Q4: mean={np.mean(steer_deg[pq3:]):+.2f}, sat>20={np.sum(np.abs(steer_deg[pq3:])>20)}/{n_p-pq3}')

# === 7. Path curvature ===
print()
print('='*70)
print('[7] Path curvature analysis')
print('='*70)

if len(save) > 2:
    sdx = np.diff(save[:,1])
    sdy = np.diff(save[:,2])
    shead = np.arctan2(sdy, sdx)
    sdh = np.diff(np.unwrap(shead))
    sds = np.sqrt(sdx[:-1]**2 + sdy[:-1]**2) * 0.5 + np.sqrt(sdx[1:]**2 + sdy[1:]**2) * 0.5
    sds = np.where(sds > 0.001, sds, 0.001)
    save_curv = np.abs(sdh / sds)
    save_R = np.where(save_curv > 0.001, 1.0 / save_curv, 9999)
    print(f'  Save path curvature radius:')
    valid_R = save_R[save_R < 9999]
    if len(valid_R) > 0:
        print(f'    min R = {valid_R.min():.2f}m')
        print(f'    median R = {np.median(valid_R):.2f}m')
    print(f'    R < 1.0m: {np.sum(save_R < 1.0)}')
    print(f'    R < 1.5m: {np.sum(save_R < 1.5)}')
    print(f'    R < 2.0m: {np.sum(save_R < 2.0)}')
    
    min_R_steer = WHEELBASE / np.tan(np.radians(MAX_STEER_DEG))
    print(f'  Min turning radius at max steer ({MAX_STEER_DEG}deg): {min_R_steer:.2f}m')
    print(f'  Points with R < min_R_steer: {np.sum(save_R < min_R_steer)}')

# === 8. Divergence analysis (forward section) ===
print()
print('='*70)
print('[8] Divergence analysis (forward section)')
print('='*70)

window = 20
if len(errors_ct) > window:
    running_max_ct = np.array([np.max(np.abs(errors_ct[max(0,i-window):i+1])) for i in range(len(errors_ct))])
    diverge_idx = np.where(running_max_ct > 0.30)[0]
    if len(diverge_idx) > 0:
        first_diverge = diverge_idx[0]
        diverge_path_dist = save_dist[seg_idx_arr[first_diverge]]
        print(f'  First significant divergence (>30cm):')
        print(f'    At car point #{first_diverge}/{len(cur_analysis)}')
        print(f'    At save path distance: {diverge_path_dist:.2f}m')
        print(f'    Car position: ({cur_analysis[first_diverge,1]:.3f}, {cur_analysis[first_diverge,2]:.3f})')
        print(f'    Nearest save point: #{seg_idx_arr[first_diverge]}')
        print(f'    Lateral error: {errors_ct[first_diverge]*100:.1f}cm')
    else:
        print(f'  No significant divergence (>30cm) detected')
    
    diverge_20 = np.where(running_max_ct > 0.20)[0]
    if len(diverge_20) > 0:
        first_20 = diverge_20[0]
        print(f'  First 20cm divergence at car point #{first_20}, path dist={save_dist[seg_idx_arr[first_20]]:.2f}m')

# === 9. Summary ===
print()
print('='*70)
print('[9] SUMMARY & RECOMMENDATIONS')
print('='*70)

rms_ct = np.sqrt(np.mean(errors_ct**2)) * 100
max_ct = np.max(np.abs(errors_ct)) * 100
if n_valid > 0:
    mean_yaw_err_deg = np.degrees(np.mean(np.abs(yaw_errors)))
else:
    mean_yaw_err_deg = 0
sat_pct = sat_20/total*100 if paired_data else 0

print(f'  Lateral RMS: {rms_ct:.1f}cm (target < 10cm)')
print(f'  Lateral MAX: {max_ct:.1f}cm')
print(f'  Yaw error mean: {mean_yaw_err_deg:.1f}deg')
print(f'  Steer saturation (>20deg): {sat_pct:.1f}%')
print()

if rms_ct > 15:
    print('  ? Lateral deviation too large!')
    print('  Possible causes:')
    print('    1. Ld too small -> steer saturation -> oscillation')
    print('    2. INS yaw drift -> wrong coordinate transform')
    print('    3. Speed too high for path curvature')
elif rms_ct > 10:
    print('  ? Lateral deviation marginal')
    print('  Consider: increase Ld or reduce speed at curves')
else:
    print('  ? Lateral deviation acceptable')

if sat_pct > 30:
    print('  ? Steer saturation too high!')
    print('  -> Increase Ld to reduce sensitivity to lateral error')
elif sat_pct > 15:
    print('  ? Steer saturation marginal')
else:
    print('  ? Steer saturation acceptable')

if n_valid > 0:
    yaw_drift_deg = np.degrees(np.max(np.abs(yaw_errors)))
    if yaw_drift_deg > 15:
        print(f'  ? Yaw drift too large ({yaw_drift_deg:.1f}deg max)')
        print('  -> Check INS yaw estimation, consider mag fusion')
    elif yaw_drift_deg > 8:
        print(f'  ? Yaw drift marginal ({yaw_drift_deg:.1f}deg max)')
    else:
        print(f'  ? Yaw drift acceptable ({yaw_drift_deg:.1f}deg max)')

# Direction switch analysis
if rev_count > 0:
    print()
    print('  *** DIRECTION SWITCH ANALYSIS ***')
    print(f'  Save has {fwd_count} fwd + {rev_count} rev points')
    print(f'  Direction change at save idx {dir_change_idx} -> {dir_change_idx+1}')
    
    # Check if follow data shows the jump bug
    all_tgt_idx = tgt[:, 0].astype(int)
    big_jumps = np.where(np.diff(all_tgt_idx) > 5)[0]
    if len(big_jumps) > 0:
        for j in big_jumps:
            print(f'  ? BUG DETECTED: Target index jumps {all_tgt_idx[j]} -> {all_tgt_idx[j+1]}')
            print(f'     This is the "skip reverse section" bug in find_lookahead_point()')
            print(f'     Fix: return direction change point immediately instead of skipping')
    else:
        print(f'  ? No target index jumps detected')

print()
print('  Key parameters:')
print(f'    Ld = {LD}m, wheelbase = {WHEELBASE}m, max_steer = {MAX_STEER_DEG}deg')
print(f'    min_R at max_steer = {WHEELBASE/np.tan(np.radians(MAX_STEER_DEG)):.2f}m')

print('='*70)
