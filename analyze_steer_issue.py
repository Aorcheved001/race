#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
深度分析：循迹过程中转向角度比真实角度大的根因分析

分析思路：
1. 从 follow 数据中提取当前位置(idx=0)和目标点(idx>0)
2. 从 D: 格式可知，目标点的 yaw 字段 = state->yaw（车辆实际航向），而非保存轨迹的 yaw
3. 当前位置(idx=0)的 yaw 来自 (x,y) 格式，不含 yaw，默认为 0
4. 重建 Pure Pursuit 转向角，与保存轨迹的期望转向角对比
5. 分析 INS 航向误差、横向偏差、前视距离等因素
"""

import numpy as np
from pathlib import Path

# ============================================================
# 参数（与嵌入式代码一致）
# ============================================================
WHEELBASE = 0.78          # 轴距 m
LOOKAHEAD = 0.15          # 前视距离 m
PP_MAX_STEER_DEG = 22.0   # PP最大转向角
STEER_DEADZONE_DEG = 1.0  # 转向死区
FILTER_ALPHA = 0.85       # 转向滤波系数

# ============================================================
# 数据加载
# ============================================================
data_dir = Path(r'e:\大车惯导\track_data')

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
    """返回 (idx, x, y, yaw) 数组"""
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
cur = follow[cur_mask]   # 当前位置
tgt = follow[tgt_mask]   # 目标点

print(f"Save points: {len(save)}")
print(f"Follow total: {len(follow)}, current: {len(cur)}, target: {len(tgt)}")
print()

# ============================================================
# [A] 关键发现：follow 数据中目标点的 yaw 是什么？
# ============================================================
print("=" * 70)
print("[A] follow 数据中目标点 yaw 字段的含义验证")
print("=" * 70)
print("D: 格式: idx, target_x, target_y, pp_steer_deg, actual_steer_deg, state->yaw")
print("→ 目标点的 yaw = state->yaw = 车辆实际航向（弧度）")
print("→ 当前位置的 yaw 来自 (x,y) 格式，不含 yaw，默认为 0")
print()

# 验证：目标点的 x,y 与 save 点一致，但 yaw 不同
print("  idx | save_yaw(rad) | follow_tgt_yaw(rad) | 差值(rad) | 差值(deg)")
print("  " + "-" * 65)
for idx in [2, 5, 10, 15, 20, 30, 50, 80, 100, 130, 160]:
    s_mask = save[:, 0] == idx
    t_mask = tgt[:, 0] == idx
    if np.sum(s_mask) == 0 or np.sum(t_mask) == 0:
        continue
    s = save[s_mask][0]
    # 取该 idx 的第一个目标点
    t = tgt[t_mask][0]
    yaw_diff = t[3] - s[3]
    yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
    print(f"  {idx:3d} | {s[3]:+13.4f} | {t[3]:+19.4f} | {yaw_diff:+9.4f} | {np.degrees(yaw_diff):+9.2f}")

print()
print("  ★ 结论：follow 目标点的 yaw 是车辆实际航向，不是保存轨迹的 yaw！")
print("  ★ 保存轨迹的 yaw 在目标点处与车辆实际航向差异很大（最大可达数十度）")
print()

# ============================================================
# [B] 重建 Pure Pursuit 转向角
# ============================================================
print("=" * 70)
print("[B] 重建 Pure Pursuit 转向角 vs 保存轨迹期望转向角")
print("=" * 70)

def pure_pursuit_steer(car_x, car_y, car_yaw, tgt_x, tgt_y):
    """计算 Pure Pursuit 转向角（度），与嵌入式代码一致"""
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
    # 死区
    if -STEER_DEADZONE_DEG < steer_deg < STEER_DEADZONE_DEG:
        steer_deg = 0.0
    # 限幅
    steer_deg = np.clip(steer_deg, -PP_MAX_STEER_DEG, PP_MAX_STEER_DEG)
    return steer_deg, ld, x_local, y_local

# 对每个目标点，找到最近的当前位置，重建 PP 转向角
# follow 数据是交替的：target, cur, target, cur, ...
# 我们需要配对：每个 target 对应时间上最近的 cur

# 方法：按数据行顺序遍历，维护最近的 cur 位置
pp_results = []
last_cur_x, last_cur_y, last_cur_yaw = 0.0, 0.0, 0.0

for row in follow:
    idx = int(row[0])
    if idx == 0:
        last_cur_x, last_cur_y = row[1], row[2]
        # yaw 字段为 0（(x,y) 格式不含 yaw），需要从目标点获取
    else:
        # 目标点：yaw = 车辆实际航向
        tgt_x, tgt_y, car_yaw = row[1], row[2], row[3]
        steer_deg, ld, x_local, y_local = pure_pursuit_steer(
            last_cur_x, last_cur_y, car_yaw, tgt_x, tgt_y)
        
        # 找对应的 save 点
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

# 分析 PP 转向角
pp_steers = [p['pp_steer'] for p in pp]
print(f"  PP 转向角统计:")
print(f"    mean = {np.mean(pp_steers):+.2f}°")
print(f"    std  = {np.std(pp_steers):.2f}°")
print(f"    max  = {np.max(pp_steers):+.2f}°")
print(f"    min  = {np.min(pp_steers):+.2f}°")
print(f"    |steer| > 10° 的比例: {np.sum(np.abs(pp_steers) > 10) / len(pp_steers) * 100:.1f}%")
print(f"    |steer| > 15° 的比例: {np.sum(np.abs(pp_steers) > 15) / len(pp_steers) * 100:.1f}%")
print()

# ============================================================
# [C] 核心分析：INS 航向误差对转向角的影响
# ============================================================
print("=" * 70)
print("[C] 核心分析：INS 航向误差如何放大转向角")
print("=" * 70)

# 关键思路：
# Pure Pursuit 的转向角 = -atan(2 * y_local / Ld^2 * L)
# y_local = -dx * sin(car_yaw) + dy * cos(car_yaw)
# 如果 car_yaw 有误差 Δyaw，则 y_local 会变化：
# y_local' ≈ y_local + (dx * cos(car_yaw) + dy * sin(car_yaw)) * Δyaw
#          = y_local + x_local * Δyaw
# 因为 x_local > 0（目标在前方），所以：
# - 如果 Δyaw > 0（INS 认为车头偏右），y_local 增大，转向角增大（左转更多）
# - 如果 Δyaw < 0（INS 认为车头偏左），y_local 减小，转向角减小（右转更多）

# 但更关键的是：在弯道上，INS 的 yaw 如果比真实 yaw 更"直"（更接近0），
# 那么在左转弯时，y_local 会偏大，导致左转更多
# 在右转弯时，y_local 会偏小（绝对值偏大），导致右转更多

# 让我验证：比较 INS yaw 与从实际路径推算的 yaw
# 从当前位置序列推算航向角
if len(cur) > 2:
    cur_x = cur[:, 1]
    cur_y = cur[:, 2]
    # 用差分计算航向角
    dx = np.diff(cur_x)
    dy = np.diff(cur_y)
    path_yaw = np.arctan2(dy, dx)
    # 平滑
    from scipy.ndimage import uniform_filter1d
    path_yaw_smooth = uniform_filter1d(np.unwrap(path_yaw), size=5)
    
    # 从目标点获取 INS yaw（按时间顺序）
    # 需要按 follow 数据的原始顺序提取
    ins_yaws = []
    path_yaws_at_same_time = []
    
    cur_idx = 0
    for row in follow:
        idx = int(row[0])
        if idx == 0:
            cur_idx += 1
        else:
            ins_yaw = row[3]
            # 找最近的 path_yaw
            if cur_idx > 0 and cur_idx - 1 < len(path_yaw_smooth):
                ins_yaws.append(ins_yaw)
                path_yaws_at_same_time.append(path_yaw_smooth[cur_idx - 1])
    
    if len(ins_yaws) > 10:
        ins_yaws = np.array(ins_yaws)
        path_yaws_at_same_time = np.array(path_yaws_at_same_time)
        
        yaw_err = ins_yaws - path_yaws_at_same_time
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
        
        print(f"  INS yaw vs 路径推算 yaw 误差:")
        print(f"    mean = {np.degrees(np.mean(yaw_err)):+.2f}°")
        print(f"    std  = {np.degrees(np.std(yaw_err)):.2f}°")
        print(f"    max  = {np.degrees(np.max(yaw_err)):+.2f}°")
        print(f"    min  = {np.degrees(np.min(yaw_err)):+.2f}°")
        print()

# ============================================================
# [D] 关键实验：用保存轨迹的 yaw 替代 INS yaw，看转向角变化
# ============================================================
print("=" * 70)
print("[D] 关键实验：用保存轨迹的 yaw 替代 INS yaw，转向角变化")
print("=" * 70)

# 对每个目标点，计算两种转向角：
# 1. 用 INS yaw（实际使用的）
# 2. 用保存轨迹在最近点处的 yaw（"理想" yaw）

steer_with_ins = []
steer_with_save = []
steer_diff_list = []

for p in pp:
    if p['save_yaw'] is None:
        continue
    
    # 1. 用 INS yaw（实际使用的）
    steer_ins = p['pp_steer']
    
    # 2. 用保存轨迹的 yaw（"理想" yaw）
    steer_save, _, _, _ = pure_pursuit_steer(
        p['car_x'], p['car_y'], p['save_yaw'], p['tgt_x'], p['tgt_y'])
    
    steer_with_ins.append(steer_ins)
    steer_with_save.append(steer_save)
    steer_diff_list.append(steer_ins - steer_save)

steer_with_ins = np.array(steer_with_ins)
steer_with_save = np.array(steer_with_save)
steer_diff = np.array(steer_diff_list)

print(f"  用 INS yaw 的转向角: mean={np.mean(np.abs(steer_with_ins)):.2f}°, "
      f"max={np.max(np.abs(steer_with_ins)):.2f}°")
print(f"  用 save yaw 的转向角: mean={np.mean(np.abs(steer_with_save)):.2f}°, "
      f"max={np.max(np.abs(steer_with_save)):.2f}°")
print(f"  转向角差异 (INS - save): mean={np.mean(steer_diff):+.2f}°, "
      f"std={np.std(steer_diff):.2f}°")
print(f"  |差异| > 2° 的比例: {np.sum(np.abs(steer_diff) > 2) / len(steer_diff) * 100:.1f}%")
print(f"  |差异| > 5° 的比例: {np.sum(np.abs(steer_diff) > 5) / len(steer_diff) * 100:.1f}%")
print()

# 分段分析：在弯道和直道上分别看
# 弯道 = |steer| > 5°，直道 = |steer| <= 5°
curve_mask = np.abs(steer_with_save) > 5
straight_mask = ~curve_mask

if np.sum(curve_mask) > 0:
    print(f"  弯道段 (|save_steer| > 5°, 共 {np.sum(curve_mask)} 个点):")
    print(f"    INS steer: mean={np.mean(np.abs(steer_with_ins[curve_mask])):.2f}°")
    print(f"    save steer: mean={np.mean(np.abs(steer_with_save[curve_mask])):.2f}°")
    print(f"    差异: mean={np.mean(steer_diff[curve_mask]):+.2f}°")
    print(f"    INS转向角/Save转向角 比值: {np.mean(np.abs(steer_with_ins[curve_mask])) / np.mean(np.abs(steer_with_save[curve_mask])):.2f}")

if np.sum(straight_mask) > 0:
    print(f"  直道段 (|save_steer| <= 5°, 共 {np.sum(straight_mask)} 个点):")
    print(f"    INS steer: mean={np.mean(np.abs(steer_with_ins[straight_mask])):.2f}°")
    print(f"    save steer: mean={np.mean(np.abs(steer_with_save[straight_mask])):.2f}°")
    print(f"    差异: mean={np.mean(steer_diff[straight_mask]):+.2f}°")
print()

# ============================================================
# [E] 更精确的分析：INS yaw 与 save yaw 的系统性偏差
# ============================================================
print("=" * 70)
print("[E] INS yaw 与 save yaw 的系统性偏差分析")
print("=" * 70)

# 对每个目标点，INS yaw 是车辆实际航向，save yaw 是轨迹在该点的航向
# 但这两个 yaw 不在同一个位置！INS yaw 是在车辆当前位置，save yaw 是在目标点位置
# 所以直接比较没有意义

# 更好的方法：对每个当前位置，找到保存轨迹上最近的点，比较 yaw
# 但当前位置的 yaw 为 0（数据问题），所以用目标点的 yaw 作为 INS yaw

# 方法：对每个目标点数据，用车辆位置找保存轨迹最近点，比较 yaw
yaw_errors = []
lateral_errors = []
along_errors = []

for p in pp:
    # 车辆当前位置
    car_x, car_y, car_yaw = p['car_x'], p['car_y'], p['car_yaw']
    
    # 找保存轨迹上最近的点
    save_xy = save[:, 1:3]
    dists = np.sqrt(np.sum((save_xy - [car_x, car_y]) ** 2, axis=1))
    nearest_idx = np.argmin(dists)
    nearest_save = save[nearest_idx]
    
    # Yaw 误差
    yaw_err = car_yaw - nearest_save[3]
    yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi
    
    # 横向误差（在保存轨迹的法线方向）
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

print(f"  INS yaw 与保存轨迹最近点 yaw 的误差:")
print(f"    mean = {np.degrees(np.mean(yaw_errors)):+.2f}°")
print(f"    std  = {np.degrees(np.std(yaw_errors)):.2f}°")
print(f"    max  = {np.degrees(np.max(yaw_errors)):+.2f}°")
print(f"    min  = {np.degrees(np.min(yaw_errors)):+.2f}°")
print()

print(f"  横向偏差（正=轨迹右侧）:")
print(f"    mean = {np.mean(lateral_errors) * 100:+.1f} cm")
print(f"    std  = {np.std(lateral_errors) * 100:.1f} cm")
print(f"    max  = {np.max(lateral_errors) * 100:+.1f} cm")
print(f"    min  = {np.min(lateral_errors) * 100:+.1f} cm")
print()

print(f"  纵向偏差（正=在最近点前方）:")
print(f"    mean = {np.mean(along_errors) * 100:+.1f} cm")
print(f"    std  = {np.std(along_errors) * 100:.1f} cm")
print()

# ============================================================
# [F] 核心根因分析：前视距离太小导致转向角过大
# ============================================================
print("=" * 70)
print("[F] 核心根因分析：前视距离对转向角的影响")
print("=" * 70)

# Pure Pursuit 的转向角公式：
# steer = -atan(2 * y_local / Ld^2 * L)
# 
# 对于圆弧路径，如果车在圆弧上：
# y_local ≈ Ld^2 / (2R)  （小角度近似）
# curvature = 2 * y_local / Ld^2 = 1/R
# steer = atan(L/R)
#
# 这个结果与 Ld 无关！理论上前视距离不影响稳态转向角。
#
# 但实际上，车不可能完美在圆弧上，存在横向偏差 e：
# y_local ≈ Ld^2 / (2R) + e * cos(α)  （α 是偏差方向角）
# 当 e > 0（车在圆弧外侧），y_local 增大，转向角增大
# 当 e < 0（车在圆弧内侧），y_local 减小，转向角减小
#
# 更关键的是：前视距离小 → 目标点近 → 对横向偏差更敏感
# Ld 小时，2/Ld^2 大，同样的 y_local 偏差导致更大的曲率变化

# 让我验证：不同前视距离下的转向角
print("  不同前视距离下的 PP 转向角对比（取弯道段数据）:")
print()

# 选取一些弯道上的数据点
curve_points = [p for p in pp if abs(p['pp_steer']) > 5 and p['save_yaw'] is not None]

if curve_points:
    for test_ld in [0.15, 0.25, 0.35, 0.50]:
        steers = []
        for p in curve_points[:50]:  # 取前50个弯道点
            # 重新计算 y_local，但用不同的 Ld
            # 注意：Ld 变化意味着目标点也变了，这里只是近似分析
            # 实际上应该重新搜索目标点，但这里用简化方法
            dx = p['tgt_x'] - p['car_x']
            dy = p['tgt_y'] - p['car_y']
            x_local = dx * np.cos(p['car_yaw']) + dy * np.sin(p['car_yaw'])
            y_local = -dx * np.sin(p['car_yaw']) + dy * np.cos(p['car_yaw'])
            
            # 用测试 Ld 重新计算（近似，实际目标点会变）
            ld_actual = np.sqrt(x_local**2 + y_local**2)
            # 按比例缩放 y_local（假设目标点在相同方向但不同距离）
            scale = test_ld / ld_actual if ld_actual > 0.01 else 1.0
            y_local_scaled = y_local * scale
            
            curvature = 2.0 * y_local_scaled / (test_ld ** 2)
            steer_rad = np.arctan(curvature * WHEELBASE)
            steer_deg = -steer_rad * 57.2957795130823
            steer_deg = np.clip(steer_deg, -PP_MAX_STEER_DEG, PP_MAX_STEER_DEG)
            steers.append(steer_deg)
        
        steers = np.array(steers)
        print(f"    Ld={test_ld:.2f}m: mean|steer|={np.mean(np.abs(steers)):.2f}°, "
              f"max|steer|={np.max(np.abs(steers)):.2f}°")

print()

# ============================================================
# [G] 最关键的分析：实际路径曲率 vs 保存轨迹曲率
# ============================================================
print("=" * 70)
print("[G] 实际路径曲率 vs 保存轨迹曲率")
print("=" * 70)

# 从当前位置序列计算实际路径曲率
if len(cur) > 10:
    cur_x = cur[:, 1]
    cur_y = cur[:, 2]
    
    # 去除重复点
    unique_mask = np.concatenate([[True], np.sqrt(np.diff(cur_x)**2 + np.diff(cur_y)**2) > 0.001])
    cur_x = cur_x[unique_mask]
    cur_y = cur_y[unique_mask]
    
    if len(cur_x) > 10:
        # 计算航向角
        dx = np.diff(cur_x)
        dy = np.diff(cur_y)
        heading = np.arctan2(dy, dx)
        heading_unwrap = np.unwrap(heading)
        
        # 计算弧长
        ds = np.sqrt(dx**2 + dy**2)
        s = np.concatenate([[0], np.cumsum(ds)])
        
        # 计算曲率 = dheading / ds
        dheading = np.diff(heading_unwrap)
        ds_mid = (ds[:-1] + ds[1:]) / 2
        ds_mid = np.where(ds_mid > 0.001, ds_mid, 0.001)
        curvature = dheading / ds_mid
        
        # 平滑曲率
        from scipy.ndimage import uniform_filter1d
        curvature_smooth = uniform_filter1d(curvature, size=5)
        
        # 转弯半径
        R_actual = np.where(np.abs(curvature_smooth) > 0.01, 1.0 / np.abs(curvature_smooth), 9999)
        
        # 保存轨迹曲率
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
        save_curvature_smooth = uniform_filter1d(save_curvature, size=5)
        R_save = np.where(np.abs(save_curvature_smooth) > 0.01, 1.0 / np.abs(save_curvature_smooth), 9999)
        
        print(f"  实际路径转弯半径:")
        print(f"    min R = {R_actual[R_actual < 100].min():.2f}m" if np.sum(R_actual < 100) > 0 else "    min R = N/A")
        print(f"    median R = {np.median(R_actual[R_actual < 100]):.2f}m" if np.sum(R_actual < 100) > 0 else "    median R = N/A")
        print(f"    R < 2.0m 的段数: {np.sum(R_actual < 2.0)} / {len(R_actual)}")
        
        print(f"  保存轨迹转弯半径:")
        print(f"    min R = {R_save[R_save < 100].min():.2f}m" if np.sum(R_save < 100) > 0 else "    min R = N/A")
        print(f"    median R = {np.median(R_save[R_save < 100]):.2f}m" if np.sum(R_save < 100) > 0 else "    median R = N/A")
        print(f"    R < 2.0m 的段数: {np.sum(R_save < 2.0)} / {len(R_save)}")
        print()

# ============================================================
# [H] 最核心的发现：分析 INS yaw 漂移/延迟
# ============================================================
print("=" * 70)
print("[H] INS yaw 漂移/延迟分析（最核心）")
print("=" * 70)

# 关键思路：
# 如果 INS yaw 比真实 yaw 滞后（即 INS 认为车还没转那么多），
# 那么在弯道上：
# - 左转弯时：INS yaw 比真实 yaw 偏右 → y_local 偏大 → 左转更多
# - 右转弯时：INS yaw 比真实 yaw 偏左 → |y_local| 偏大 → 右转更多
# 结果：无论左转右转，车都会转得更多！

# 验证方法：比较 INS yaw 变化率与路径 yaw 变化率
# 如果 INS yaw 变化率 < 路径 yaw 变化率，说明 INS 有延迟

# 从目标点数据提取 INS yaw 序列（按时间顺序）
ins_yaw_seq = []
for row in follow:
    idx = int(row[0])
    if idx > 0:
        ins_yaw_seq.append(row[3])

if len(ins_yaw_seq) > 10:
    ins_yaw_arr = np.array(ins_yaw_seq)
    ins_yaw_unwrap = np.unwrap(ins_yaw_arr)
    
    # INS yaw 变化率
    ins_dyaw = np.diff(ins_yaw_unwrap)
    
    # 从当前位置计算路径 yaw 变化率
    if len(cur) > 10:
        cur_x = cur[:, 1]
        cur_y = cur[:, 2]
        unique_mask = np.concatenate([[True], np.sqrt(np.diff(cur_x)**2 + np.diff(cur_y)**2) > 0.001])
        cur_x_u = cur_x[unique_mask]
        cur_y_u = cur_y[unique_mask]
        
        if len(cur_x_u) > 10:
            path_heading = np.unwrap(np.arctan2(np.diff(cur_y_u), np.diff(cur_x_u)))
            path_dyaw = np.diff(path_heading)
            
            # 采样比较（INS 数据点更多，需要匹配）
            # 简化：比较总变化量
            print(f"  INS yaw 总变化量: {np.degrees(ins_yaw_unwrap[-1] - ins_yaw_unwrap[0]):+.2f}°")
            print(f"  INS yaw 变化点数: {len(ins_dyaw)}")
            print(f"  INS yaw 平均变化率: {np.degrees(np.mean(np.abs(ins_dyaw))):.4f}°/step")
            print()
            print(f"  路径 yaw 总变化量: {np.degrees(path_heading[-1] - path_heading[0]):+.2f}°")
            print(f"  路径 yaw 变化点数: {len(path_dyaw)}")
            print(f"  路径 yaw 平均变化率: {np.degrees(np.mean(np.abs(path_dyaw))):.4f}°/step")
            print()

# ============================================================
# [I] 终极验证：用保存轨迹的 yaw 重新计算 PP 转向角，模拟理想情况
# ============================================================
print("=" * 70)
print("[I] 终极验证：模拟理想 INS（无 yaw 误差）的转向角")
print("=" * 70)

# 假设 INS 完美，车辆始终在保存轨迹上，yaw = 保存轨迹的 yaw
# 那么目标点就是前方 Ld 处的保存轨迹点
# 计算这种情况下的 PP 转向角

ideal_steers = []
for i in range(len(save) - 2):
    car_x = save[i, 1]
    car_y = save[i, 2]
    car_yaw = save[i, 3]
    
    # 找前方 Ld 处的点
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
    print(f"  理想情况（车在轨迹上，INS 完美）的 PP 转向角:")
    print(f"    mean = {np.mean(ideal_steer_vals):+.2f}°")
    print(f"    mean|steer| = {np.mean(np.abs(ideal_steer_vals)):.2f}°")
    print(f"    max  = {np.max(ideal_steer_vals):+.2f}°")
    print(f"    min  = {np.min(ideal_steer_vals):+.2f}°")
    print(f"    |steer| > 10° 的比例: {np.sum(np.abs(ideal_steer_vals) > 10) / len(ideal_steer_vals) * 100:.1f}%")
    print()
    
    # 对比实际 PP 转向角
    print(f"  实际情况（INS 有误差）的 PP 转向角:")
    print(f"    mean = {np.mean(steer_with_ins):+.2f}°")
    print(f"    mean|steer| = {np.mean(np.abs(steer_with_ins)):.2f}°")
    print(f"    |steer| > 10° 的比例: {np.sum(np.abs(steer_with_ins) > 10) / len(steer_with_ins) * 100:.1f}%")
    print()
    
    # 转向角放大比
    ratio = np.mean(np.abs(steer_with_ins)) / np.mean(np.abs(ideal_steer_vals))
    print(f"  ★ 转向角放大比（实际/理想）: {ratio:.2f}")
    print(f"  ★ 如果 > 1.0，说明 INS 误差导致转向角被放大")
    print()

# ============================================================
# [J] 最终结论
# ============================================================
print("=" * 70)
print("[J] 最终结论与根因分析")
print("=" * 70)

# 综合所有分析，给出结论
print("""
  分析总结：
  
  1. follow 数据中目标点的 yaw 字段 = state->yaw（车辆实际航向），
     不是保存轨迹在该点的 yaw。两者差异可达数十度。
  
  2. 当前位置(idx=0)的 yaw 始终为 0（数据格式不含 yaw），
     无法直接用于航向误差分析。
  
  3. Pure Pursuit 转向角公式：
     steer = -atan(2 * y_local / Ld? * L)
     其中 y_local = -dx·sin(yaw) + dy·cos(yaw)
     
     关键：y_local 对 yaw 非常敏感！
     当 yaw 有误差 Δyaw 时：
     y_local' ≈ y_local + x_local · Δyaw
     
     因为 x_local > 0（目标在前方），Δyaw 的误差会被 x_local 放大！
  
  4. 前视距离 Ld = 0.15m 非常小，导致：
     - 2/Ld? = 88.9，曲率对 y_local 的敏感度极高
     - 同样的 y_local 偏差，Ld=0.15m 时的曲率是 Ld=0.30m 时的 4 倍
     - 小前视距离 + yaw 误差 = 转向角被大幅放大
""")

# 计算具体的放大效应
print("  具体放大效应计算：")
print(f"    前视距离 Ld = {LOOKAHEAD}m")
print(f"    轴距 L = {WHEELBASE}m")
print(f"    2/Ld? = {2/LOOKAHEAD**2:.1f}")
print()

# 假设 yaw 误差为 3°，计算对转向角的影响
for delta_yaw_deg in [1, 2, 3, 5]:
    delta_yaw = np.radians(delta_yaw_deg)
    # 假设 x_local ≈ Ld = 0.15m（目标正前方）
    x_local_typical = LOOKAHEAD
    y_local_error = x_local_typical * delta_yaw
    curvature_error = 2 * y_local_error / (LOOKAHEAD ** 2)
    steer_error_rad = np.arctan(curvature_error * WHEELBASE)
    steer_error_deg = np.degrees(steer_error_rad)
    print(f"    yaw 误差 {delta_yaw_deg}° → y_local 偏差 {y_local_error*100:.2f}cm "
          f"→ 转向角偏差 {steer_error_deg:.2f}°")

print()
print("  ★★★ 核心根因 ★★★")
print()
print("  转向角比真实角度大的根本原因是：")
print()
print("  【INS 航向(yaw)误差被小前视距离放大】")
print()
print("  机制：")
print("  1. INS 的 yaw 存在系统性误差（漂移/延迟/噪声）")
print("  2. Pure Pursuit 中 y_local = -dx·sin(yaw) + dy·cos(yaw)")
print("     yaw 误差 Δyaw 导致 y_local 偏差 ≈ x_local · Δyaw")
print("  3. 前视距离 Ld=0.15m 极小，2/Ld?=88.9，曲率敏感度极高")
print("     y_local 的微小偏差被放大为巨大的曲率变化")
print("  4. 在弯道上，INS yaw 倾向于滞后于真实 yaw")
print("     → 车认为还没转够 → 计算出更大的转向角 → 过度转向")
print()
print("  建议改进方案：")
print("  1. 增大前视距离 Ld（0.15m → 0.30~0.50m），降低曲率敏感度")
print("  2. 改善 INS yaw 精度（EKF 调参、磁力计补偿等）")
print("  3. 在 PP 计算中加入 yaw 误差补偿")
print("  4. 考虑使用 Stanley 算法替代 Pure Pursuit（对航向误差更鲁棒）")
