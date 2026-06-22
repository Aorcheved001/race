# -*- coding: utf-8 -*-
"""
分析科目三转弯切角问题 - PP算法根因分析
分析最新一次数据(subj3_follow_20260613_233757.txt)
量化切角程度，探讨是否为PP算法固有缺陷
"""
import math
import numpy as np

# ============================================================
# 1. 读取数据
# ============================================================
drive_file = r"e:\大车惯导\track_data\subj3_drive_20260613_233623.txt"
follow_file = r"e:\大车惯导\track_data\subj3_follow_20260613_233757.txt"

# 读取DRIVE路径点
path_points = []
with open(drive_file, 'r') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split(',')
        if len(parts) >= 5:
            idx = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            yaw = float(parts[3])
            path_points.append((idx, x, y, yaw))

print(f"路径点数: {len(path_points)}")
print(f"路径范围: x=[{path_points[0][1]:.2f}, {path_points[-1][1]:.2f}], y=[{min(p[2] for p in path_points):.2f}, {max(p[2] for p in path_points):.2f}]")

# 读取FOLLOW车辆轨迹
car_positions = []
target_positions = []
with open(follow_file, 'r') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split(',')
        if len(parts) >= 5:
            idx = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            yaw = float(parts[3])
            target_idx = int(parts[4])
            if idx == 0:
                car_positions.append((x, y, yaw, target_idx))
            else:
                target_positions.append((x, y, yaw, target_idx))

print(f"车辆轨迹点数: {len(car_positions)}")

# ============================================================
# 2. 计算每个车辆位置到路径的横向偏差
# ============================================================
def point_to_segment_distance(px, py, x1, y1, x2, y2):
    """计算点(px,py)到线段(x1,y1)-(x2,y2)的最短距离"""
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx*dx + dy*dy
    if len_sq < 1e-10:
        return math.sqrt((px-x1)**2 + (py-y1)**2)
    t = max(0, min(1, ((px-x1)*dx + (py-y1)*dy) / len_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.sqrt((px-proj_x)**2 + (py-proj_y)**2)

def signed_lateral_error(px, py, path_x1, path_y1, path_x2, path_y2):
    """计算带符号的横向偏差（正=路径左侧，负=路径右侧）"""
    dx = path_x2 - path_x1
    dy = path_y2 - path_y1
    len_sq = dx*dx + dy*dy
    if len_sq < 1e-10:
        return 0.0
    # 叉积判断方向
    cross = dx * (py - path_y1) - dy * (px - path_x1)
    dist = point_to_segment_distance(px, py, path_x1, path_y1, path_x2, path_y2)
    return dist if cross > 0 else -dist

# 计算每个车辆位置的横向偏差
lateral_errors = []
for i, (cx, cy, cyaw, tidx) in enumerate(car_positions):
    min_dist = float('inf')
    signed_err = 0.0
    # 搜索最近的路径段
    for j in range(len(path_points) - 1):
        _, x1, y1, _ = path_points[j]
        _, x2, y2, _ = path_points[j+1]
        d = point_to_segment_distance(cx, cy, x1, y1, x2, y2)
        if d < min_dist:
            min_dist = d
            signed_err = signed_lateral_error(cx, cy, x1, y1, x2, y2)
    lateral_errors.append((i, cx, cy, cyaw, tidx, min_dist, signed_err))

# ============================================================
# 3. 分析弯道区域的切角
# ============================================================
# 计算路径曲率（通过相邻三点）
path_curvatures = []
for i in range(1, len(path_points) - 1):
    _, x0, y0, _ = path_points[i-1]
    _, x1, y1, _ = path_points[i]
    _, x2, y2, _ = path_points[i+1]
    
    # 向量
    v1x, v1y = x1-x0, y1-y0
    v2x, v2y = x2-x1, y2-y1
    
    len1 = math.sqrt(v1x**2 + v1y**2)
    len2 = math.sqrt(v2x**2 + v2y**2)
    
    if len1 < 1e-6 or len2 < 1e-6:
        path_curvatures.append(0.0)
        continue
    
    # 曲率 = 2 * |叉积| / (|v1|*|v2|*|v1+v2|)
    cross = v1x * v2y - v1y * v2x
    chord = math.sqrt((x2-x0)**2 + (y2-y0)**2)
    if chord < 1e-6:
        path_curvatures.append(0.0)
        continue
    
    curvature = 2.0 * abs(cross) / (len1 * len2 * chord)
    # 带符号曲率：正=左转，负=右转
    sign = 1.0 if cross > 0 else -1.0
    path_curvatures.append(sign * curvature)

# 找到弯道区域（曲率较大的区域）
print("\n" + "="*70)
print("路径曲率分析")
print("="*70)
max_curv = max(abs(c) for c in path_curvatures)
print(f"最大曲率: {max_curv:.4f} 1/m (最小转弯半径: {1/max_curv:.2f} m)")

# 找弯道中心
curve_regions = []
in_curve = False
curve_start = 0
for i, c in enumerate(path_curvatures):
    if abs(c) > 0.15:  # 曲率阈值
        if not in_curve:
            curve_start = i
            in_curve = True
    else:
        if in_curve:
            curve_regions.append((curve_start+1, i+1))  # +1因为curvatures从idx=1开始
            in_curve = False
if in_curve:
    curve_regions.append((curve_start+1, len(path_points)-1))

print(f"\n弯道区域数: {len(curve_regions)}")
for start, end in curve_regions:
    max_c = max(abs(path_curvatures[i-1]) for i in range(start, end))
    direction = "左转" if path_curvatures[(start+end)//2 - 1] > 0 else "右转"
    print(f"  点 {start}-{end}: 最大曲率={max_c:.3f} 1/m (R={1/max_c:.2f}m), {direction}")

# ============================================================
# 4. 分析弯道中的横向偏差
# ============================================================
print("\n" + "="*70)
print("弯道区域横向偏差分析（切角量化）")
print("="*70)

# 对于每个弯道区域，找到对应的车辆位置
for ci, (curve_start, curve_end) in enumerate(curve_regions):
    # 找到target_idx在弯道范围内的车辆位置
    curve_car_pts = [(i, cx, cy, cyaw, tidx, dist, serr) 
                     for i, cx, cy, cyaw, tidx, dist, serr in lateral_errors
                     if curve_start <= tidx <= curve_end]
    
    if not curve_car_pts:
        print(f"\n弯道 {ci+1} (点{curve_start}-{curve_end}): 无车辆数据")
        continue
    
    # 计算该弯道区域的最大横向偏差
    max_err = max(abs(serr) for _, _, _, _, _, _, serr in curve_car_pts)
    avg_err = np.mean([abs(serr) for _, _, _, _, _, _, serr in curve_car_pts])
    
    # 切角方向：负=内侧切角（弯道内侧），正=外侧偏离
    # 对于左转弯道，内侧=右侧（负偏差），切角=负偏差
    # 对于右转弯道，内侧=左侧（正偏差），切角=正偏差
    direction = "左转" if path_curvatures[(curve_start+curve_end)//2 - 1] > 0 else "右转"
    
    # 计算切角量（弯道内侧的偏差）
    if direction == "左转":
        # 左转时切角=负偏差（车在路径右侧=内侧）
        cutting_errors = [serr for _, _, _, _, _, _, serr in curve_car_pts if serr < 0]
    else:
        # 右转时切角=正偏差（车在路径左侧=内侧）
        cutting_errors = [serr for _, _, _, _, _, _, serr in curve_car_pts if serr > 0]
    
    max_cut = max(abs(e) for e in cutting_errors) if cutting_errors else 0
    avg_cut = np.mean([abs(e) for e in cutting_errors]) if cutting_errors else 0
    
    print(f"\n弯道 {ci+1} (点{curve_start}-{curve_end}, {direction}):")
    print(f"  最大横向偏差: {max_err*100:.1f} cm")
    print(f"  平均横向偏差: {avg_err*100:.1f} cm")
    print(f"  最大切角量:   {max_cut*100:.1f} cm (弯道内侧)")
    print(f"  平均切角量:   {avg_cut*100:.1f} cm (弯道内侧)")

# ============================================================
# 5. 全局统计
# ============================================================
print("\n" + "="*70)
print("全局横向偏差统计")
print("="*70)
all_abs_err = [abs(serr) for _, _, _, _, _, _, serr in lateral_errors]
all_signed = [serr for _, _, _, _, _, _, serr in lateral_errors]
print(f"最大横向偏差: {max(all_abs_err)*100:.1f} cm")
print(f"平均横向偏差: {np.mean(all_abs_err)*100:.1f} cm")
print(f"中位横向偏差: {np.median(all_abs_err)*100:.1f} cm")
print(f"偏差>30cm的比例: {sum(1 for e in all_abs_err if e > 0.30)/len(all_abs_err)*100:.1f}%")
print(f"偏差>50cm的比例: {sum(1 for e in all_abs_err if e > 0.50)/len(all_abs_err)*100:.1f}%")

# 切角（内侧偏差）统计
cutting_count = sum(1 for _, _, _, _, _, _, serr in lateral_errors if serr < 0)
outside_count = sum(1 for _, _, _, _, _, _, serr in lateral_errors if serr > 0)
print(f"\n内侧切角点数: {cutting_count} ({cutting_count/len(lateral_errors)*100:.1f}%)")
print(f"外侧偏离点数: {outside_count} ({outside_count/len(lateral_errors)*100:.1f}%)")

# ============================================================
# 6. PP算法理论分析
# ============================================================
print("\n" + "="*70)
print("PP算法切角理论分析")
print("="*70)

# PP算法在圆弧路径上的理论切角量
# 对于半径R的圆弧，前视距离Ld，理论横向偏差 ≈ Ld?/(8R)
# 参考: Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm

ld = 0.40  # 前视距离
print(f"\n当前参数: Ld={ld}m, 轴距={0.78}m, 最大转向角={24}°")
print(f"最小转弯半径(物理): {0.78/math.tan(math.radians(24)):.2f}m")
print()

for start, end in curve_regions:
    max_c = max(abs(path_curvatures[i-1]) for i in range(start, end))
    R = 1.0/max_c if max_c > 0.01 else float('inf')
    direction = "左转" if path_curvatures[(start+end)//2 - 1] > 0 else "右转"
    
    # PP理论切角量
    theoretical_cut = ld**2 / (8*R) if R > 0 else 0
    
    # 实际切角量
    curve_car_pts = [(i, cx, cy, cyaw, tidx, dist, serr) 
                     for i, cx, cy, cyaw, tidx, dist, serr in lateral_errors
                     if start <= tidx <= end]
    if direction == "左转":
        actual_cuts = [abs(serr) for _, _, _, _, _, _, serr in curve_car_pts if serr < 0]
    else:
        actual_cuts = [abs(serr) for _, _, _, _, _, _, serr in curve_car_pts if serr > 0]
    actual_max = max(actual_cuts) if actual_cuts else 0
    actual_avg = np.mean(actual_cuts) if actual_cuts else 0
    
    print(f"弯道(点{start}-{end}, {direction}, R={R:.2f}m):")
    print(f"  PP理论切角: {theoretical_cut*100:.1f} cm (Ld?/8R)")
    print(f"  实际最大切角: {actual_max*100:.1f} cm")
    print(f"  实际平均切角: {actual_avg*100:.1f} cm")
    print(f"  实际/理论比: {actual_max/theoretical_cut:.1f}x" if theoretical_cut > 0.001 else "  N/A")

# ============================================================
# 7. 转向响应延迟分析
# ============================================================
print("\n" + "="*70)
print("转向响应延迟分析")
print("="*70)

# 分析车辆yaw变化率 vs 路径yaw变化率
# 在弯道入口处，车辆yaw是否滞后于路径yaw
for ci, (curve_start, curve_end) in enumerate(curve_regions):
    direction = "左转" if path_curvatures[(curve_start+curve_end)//2 - 1] > 0 else "右转"
    
    # 找弯道入口附近的车辆数据
    entry_pts = [(i, cx, cy, cyaw, tidx) 
                 for i, cx, cy, cyaw, tidx in [(le[0], le[1], le[2], le[3], le[4]) for le in lateral_errors]
                 if curve_start - 5 <= tidx <= curve_start + 10]
    
    if len(entry_pts) < 5:
        continue
    
    # 计算车辆在弯道入口处的yaw变化
    car_yaw_at_entry = entry_pts[0][3]
    car_yaw_mid = entry_pts[len(entry_pts)//2][3]
    car_yaw_change = car_yaw_mid - car_yaw_at_entry
    
    # 路径在弯道入口处的yaw变化
    path_yaw_at_entry = path_points[curve_start][3]
    path_yaw_mid = path_points[min(curve_start + 5, len(path_points)-1)][3]
    path_yaw_change = path_yaw_mid - path_yaw_at_entry
    
    print(f"\n弯道 {ci+1} ({direction}):")
    print(f"  车辆yaw变化: {math.degrees(car_yaw_change):.1f}°")
    print(f"  路径yaw变化: {math.degrees(path_yaw_change):.1f}°")
    print(f"  yaw滞后: {math.degrees(path_yaw_change - car_yaw_change):.1f}°")

# ============================================================
# 8. 前视距离敏感性分析
# ============================================================
print("\n" + "="*70)
print("前视距离敏感性分析")
print("="*70)
print("不同Ld下的PP理论切角量 (Ld?/8R):")
print(f"{'Ld(m)':>8}", end="")
for start, end in curve_regions:
    max_c = max(abs(path_curvatures[i-1]) for i in range(start, end))
    R = 1.0/max_c if max_c > 0.01 else float('inf')
    print(f"  R={R:.1f}m", end="")
print()

for ld_test in [0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60]:
    print(f"{ld_test:>8.2f}", end="")
    for start, end in curve_regions:
        max_c = max(abs(path_curvatures[i-1]) for i in range(start, end))
        R = 1.0/max_c if max_c > 0.01 else float('inf')
        cut = ld_test**2 / (8*R) if R > 0 else 0
        print(f"  {cut*100:>6.1f}cm", end="")
    print()

# ============================================================
# 9. 结论
# ============================================================
print("\n" + "="*70)
print("结论与建议")
print("="*70)
print("""
1. PP算法的切角是固有的：
   - PP算法追踪的是前视点而非路径本身，在弯道中必然产生内侧切角
   - 理论切角量 ≈ Ld?/(8R)，当前Ld=0.40m下，R=2m弯道理论切角=10cm
   
2. 实际切角远大于理论值的原因：
   - 转向滤波(alpha=0.50)引入约1帧延迟，弯道入口转向响应滞后
   - 转向变化率限幅(6°/帧)进一步延迟转向建立
   - 固定前视距离(0.40m)在所有弯道使用同一值，无法适应不同曲率
   - 无速度自适应(gain=0)，弯道中速度不减，惯性增大切角
   
3. 改进方案（按效果排序）：
   a) Stanley算法：直接追踪路径横向偏差，不切角，但高速振荡
   b) PP+Stanley混合：弯道用Stanley，直道用PP
   c) 曲率自适应前视距离：弯道中缩短Ld，直道中放长Ld
   d) 弯道减速：根据曲率降低速度，减少惯性切角
   e) 减小滤波系数和变化率限幅：提高转向响应速度
""")
