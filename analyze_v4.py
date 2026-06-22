#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import os, glob

# Auto-find latest data files
data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'track_data')
save_files = sorted(glob.glob(os.path.join(data_dir, 'save_*.txt')))
follow_files = sorted(glob.glob(os.path.join(data_dir, 'follow_*.txt')))

save_path = save_files[-1]
follow_path = follow_files[-1]
print(f'Save: {os.path.basename(save_path)}')
print(f'Follow: {os.path.basename(follow_path)}')

# Load
save = []
with open(save_path, encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'): continue
        parts = line.split(',')
        if len(parts) >= 5:
            save.append([int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), int(parts[4])])
save = np.array(save)

follow = []
with open(follow_path, encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'): continue
        parts = line.split(',')
        if len(parts) >= 4:
            follow.append([int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])])
follow = np.array(follow)

cur = follow[follow[:,0]==0]
tgt = follow[follow[:,0]>0]

# === A: Overview ===
print('\n' + '='*70)
print('[A] DATA OVERVIEW')
print('='*70)
save_ds = np.sqrt(np.diff(save[:,1])**2 + np.diff(save[:,2])**2)
cur_ds = np.sqrt(np.diff(cur[:,1])**2 + np.diff(cur[:,2])**2)
print(f'  Save: {len(save)} pts, path={np.sum(save_ds):.2f}m')
print(f'  Follow: {len(follow)} pts (cur={len(cur)}, tgt={len(tgt)})')
print(f'  Follow path: {np.sum(cur_ds):.2f}m')
print(f'  Target idx: [{int(tgt[:,0].min())}, {int(tgt[:,0].max())}]')

# === B: Save path shape ===
print('\n' + '='*70)
print('[B] SAVE PATH SHAPE')
print('='*70)
shead = np.arctan2(np.diff(save[:,2]), np.diff(save[:,1]))
shead_uw = np.unwrap(shead)
total_turn = np.degrees(shead_uw[-1] - shead_uw[0])
print(f'  Total heading change: {total_turn:.1f} deg')
print(f'  Start: ({save[0,1]:.2f},{save[0,2]:.2f}) yaw={np.degrees(save[0,3]):.1f}')
print(f'  End:   ({save[-1,1]:.2f},{save[-1,2]:.2f}) yaw={np.degrees(save[-1,3]):.1f}')

# Curvature radius
sdh = np.diff(shead_uw)
sds = np.sqrt(np.diff(save[:,1])[:-1]**2 + np.diff(save[:,2])[:-1]**2)*0.5 + np.sqrt(np.diff(save[:,1])[1:]**2 + np.diff(save[:,2])[1:]**2)*0.5
sds = np.where(sds > 0.001, sds, 0.001)
save_curv = np.abs(sdh / sds)
save_R = np.where(save_curv > 0.001, 1.0/save_curv, 9999)
print(f'  Min curvature radius: {save_R.min():.2f}m')
print(f'  Median R: {np.median(save_R):.2f}m')
print(f'  R<1.5m: {np.sum(save_R<1.5)}/{len(save_R)}')
print(f'  R<2.0m: {np.sum(save_R<2.0)}/{len(save_R)}')

# Tightest turns
tight_idx = np.argsort(save_R)[:5]
print(f'  Tightest turns:')
for idx in tight_idx:
    si = idx + 1
    print(f'    idx={save[si,0]} pos=({save[si,1]:.2f},{save[si,2]:.2f}) R={save_R[idx]:.2f}m')

# === C: Lateral deviation ===
print('\n' + '='*70)
print('[C] LATERAL DEVIATION (car vs save)')
print('='*70)
lat_errors = []
for p in cur[:, 1:3]:
    dists = np.sqrt(np.sum((save[:,1:3] - p)**2, axis=1))
    ni = np.argmin(dists)
    dx = p[0] - save[ni,1]
    dy = p[1] - save[ni,2]
    yaw = save[ni,3]
    lat = -dx*np.sin(yaw) + dy*np.cos(yaw)
    lat_errors.append(lat)
lat_errors = np.array(lat_errors)
print(f'  RMS: {np.sqrt(np.mean(lat_errors**2))*100:.2f}cm')
print(f'  Mean: {np.mean(lat_errors)*100:+.2f}cm')
print(f'  Max|err|: {np.max(np.abs(lat_errors))*100:.2f}cm')

# Progression
n_segs = 10
seg_size = len(lat_errors) // n_segs
print(f'  Progression:')
for i in range(n_segs):
    s = i*seg_size
    e = (i+1)*seg_size if i < n_segs-1 else len(lat_errors)
    seg = lat_errors[s:e]
    print(f'    Seg{i+1:2d} [{s:4d}-{e:4d}]: mean={np.mean(seg)*100:+7.2f}cm  max|err|={np.max(np.abs(seg))*100:7.2f}cm')

# === D: INS yaw consistency ===
print('\n' + '='*70)
print('[D] INS YAW CONSISTENCY (INS yaw vs path-derived heading)')
print('='*70)
if len(cur) > 2:
    path_h = np.arctan2(np.diff(cur[:,2]), np.diff(cur[:,1]))
    ins_h_mid = (cur[:-1,3] + cur[1:,3]) / 2
    hdiff = path_h - ins_h_mid
    hdiff = (hdiff + np.pi) % (2*np.pi) - np.pi
    step_d = np.sqrt(np.diff(cur[:,1])**2 + np.diff(cur[:,2])**2)
    valid = step_d > 0.02
    if np.sum(valid) > 10:
        hd = hdiff[valid]
        print(f'  Valid steps: {np.sum(valid)}/{len(valid)}')
        print(f'  Mean diff: {np.degrees(np.mean(hd)):+.2f}deg')
        print(f'  Std diff:  {np.degrees(np.std(hd)):.2f}deg')
        print(f'  Max|diff|: {np.degrees(np.max(np.abs(hd))):.2f}deg')
        p90 = np.sum(np.abs(hd) > np.radians(90))/np.sum(valid)*100
        p30 = np.sum(np.abs(hd) > np.radians(30))/np.sum(valid)*100
        print(f'  |diff|>30deg: {p30:.0f}%')
        print(f'  |diff|>90deg: {p90:.0f}%')
        
        # Progression (use valid indices)
        valid_indices = np.where(valid)[0]
        v_seg_size = len(valid_indices) // n_segs
        print(f'  Progression (valid steps only):')
        for i in range(n_segs):
            s2 = i * v_seg_size
            e2 = (i+1) * v_seg_size if i < n_segs-1 else len(valid_indices)
            seg2 = hd[s2:e2]
            if len(seg2) == 0: continue
            print(f'    Seg{i+1:2d}: mean={np.degrees(np.mean(seg2)):+7.2f}deg  std={np.degrees(np.std(seg2)):6.2f}deg  max|err|={np.degrees(np.max(np.abs(seg2))):7.2f}deg')

# === E: Direction check at matched distances ===
print('\n' + '='*70)
print('[E] DIRECTION CHECK (save yaw vs follow yaw at same distance)')
print('='*70)
save_s = np.concatenate([[0], np.cumsum(save_ds)])
cur_s = np.concatenate([[0], np.cumsum(cur_ds)])
min_len = min(save_s[-1], cur_s[-1])
for i in range(10):
    d = (i+0.5)/10 * min_len
    si = np.argmin(np.abs(save_s - d))
    ci = np.argmin(np.abs(cur_s - d))
    sy = save[si,3]
    fy = cur[ci,3]
    diff = (fy - sy + np.pi) % (2*np.pi) - np.pi
    dd = np.degrees(diff)
    if abs(dd) < 10: st = 'OK'
    elif abs(dd) < 45: st = 'DRIFT'
    elif abs(dd) < 90: st = 'WRONG'
    else: st = 'REVERSE!'
    print(f'  d={d:5.2f}m: save_yaw={np.degrees(sy):+7.1f} follow_yaw={np.degrees(fy):+7.1f} diff={dd:+7.1f} {st}')

# === F: Target idx progression ===
print('\n' + '='*70)
print('[F] TARGET IDX PROGRESSION')
print('='*70)
tgt_idx = tgt[:,0].astype(int)
unique_tgt = []
prev = -1
for t in tgt_idx:
    if t != prev:
        unique_tgt.append(t)
        prev = t
print(f'  Unique targets: {len(unique_tgt)}')
print(f'  First 20: {unique_tgt[:20]}')
print(f'  Last 20: {unique_tgt[-20:]}')
jumps = [(unique_tgt[i-1], unique_tgt[i], unique_tgt[i]-unique_tgt[i-1]) for i in range(1, len(unique_tgt)) if unique_tgt[i]-unique_tgt[i-1] != 1]
if jumps:
    print(f'  Jumps: {len(jumps)}')
    for p,c,d in jumps[:10]:
        print(f'    {p}->{c} (jump={d})')
else:
    print(f'  All sequential')

# === G: PP simulation ===
print('\n' + '='*70)
print('[G] PP SIMULATION (Ld=0.35m)')
print('='*70)
WHEELBASE = 0.78
Ld = 0.35
n_sim = min(500, len(cur))
sim_steer = []
for i in range(n_sim):
    cx, cy, cyaw = cur[i,1], cur[i,2], cur[i,3]
    dists = np.sqrt((save[:,1]-cx)**2 + (save[:,2]-cy)**2)
    ni = np.argmin(dists)
    # Find Ld ahead
    accum = 0
    ti = ni
    for j in range(ni, len(save)-1):
        seg = np.sqrt((save[j+1,1]-save[j,1])**2 + (save[j+1,2]-save[j,2])**2)
        accum += seg
        if accum >= Ld:
            ti = j+1
            break
    else:
        ti = len(save)-1
    tx, ty = save[ti,1], save[ti,2]
    dx = tx - cx
    dy = ty - cy
    x_l = dx*np.cos(cyaw) + dy*np.sin(cyaw)
    y_l = -dx*np.sin(cyaw) + dy*np.cos(cyaw)
    ld_a = np.sqrt(dx**2 + dy**2)
    if ld_a > 0.01:
        curv = 2*y_l/(ld_a**2)
        steer = np.degrees(np.arctan(curv*WHEELBASE))
    else:
        steer = 0
    sim_steer.append(steer)
sim_steer = np.array(sim_steer)
print(f'  Mean|steer|: {np.mean(np.abs(sim_steer)):.2f}deg')
print(f'  Max|steer|:  {np.max(np.abs(sim_steer)):.2f}deg')
print(f'  >20deg: {np.sum(np.abs(sim_steer)>20)}/{n_sim}')
print(f'  >15deg: {np.sum(np.abs(sim_steer)>15)}/{n_sim}')

# === H: Required Ld analysis ===
print('\n' + '='*70)
print('[H] REQUIRED Ld FOR STABILITY')
print('='*70)
for max_s in [15, 18, 20]:
    tan_s = np.tan(np.radians(max_s))
    req_lds = []
    for i in range(min(500, len(cur))):
        cx, cy, cyaw = cur[i,1], cur[i,2], cur[i,3]
        dists = np.sqrt((save[:,1]-cx)**2 + (save[:,2]-cy)**2)
        ni = np.argmin(dists)
        dx = save[ni,1] - cx
        dy = save[ni,2] - cy
        y_l = -dx*np.sin(cyaw) + dy*np.cos(cyaw)
        if abs(y_l) > 0.001:
            ld_r = np.sqrt(2*abs(y_l)*WHEELBASE/tan_s)
            req_lds.append(ld_r)
    req_lds = np.array(req_lds)
    print(f'  Max_steer<{max_s}deg: Ld mean={np.mean(req_lds):.3f}m p50={np.median(req_lds):.3f}m p90={np.percentile(req_lds,90):.3f}m max={np.max(req_lds):.3f}m')

print('\n' + '='*70)
print('[I] KEY FINDINGS SUMMARY')
print('='*70)
print(f'  1. Lateral RMS = {np.sqrt(np.mean(lat_errors**2))*100:.1f}cm (target <10cm)')
print(f'  2. Max lateral = {np.max(np.abs(lat_errors))*100:.1f}cm')
if len(cur) > 2 and np.sum(valid) > 10:
    print(f'  3. INS yaw consistency std = {np.degrees(np.std(hd)):.1f}deg')
    print(f'  4. INS yaw |diff|>90deg = {p90:.0f}%')
print(f'  5. Save min R = {save_R.min():.2f}m')
print(f'  6. PP steer >20deg = {np.sum(np.abs(sim_steer)>20)}/{n_sim}')
print('='*70)
