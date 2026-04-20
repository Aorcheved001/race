# -*- coding: utf-8 -*-
"""验证磁力计校准质量"""
import numpy as np
import re
import sys

# 读取数据
filepath = sys.argv[1] if len(sys.argv) > 1 else r'd:\race\save\4.10\new_ins\serial_log_COM14_20260416_211629.txt'
print(f"Reading: {filepath}")

data = []
with open(filepath, 'r', encoding='utf-8') as f:
    for line in f:
        m = re.search(r'\] ([-\d.]+),([-\d.]+),([-\d.]+)', line)
        if m:
            data.append([float(m.group(1)), float(m.group(2)), float(m.group(3))])

data = np.array(data)
n = len(data)
print(f"Data points: {n}")

if n == 0:
    print("No valid data!")
    sys.exit(1)

# 数据范围
print(f"\nData ranges:")
print(f"  X: {data[:,0].min():.0f} ~ {data[:,0].max():.0f} (range={data[:,0].max()-data[:,0].min():.0f})")
print(f"  Y: {data[:,1].min():.0f} ~ {data[:,1].max():.0f} (range={data[:,1].max()-data[:,1].min():.0f})")
print(f"  Z: {data[:,2].min():.0f} ~ {data[:,2].max():.0f} (range={data[:,2].max()-data[:,2].min():.0f})")

# 八分象限分布
octants = np.zeros(8)
for d in data:
    idx = 0
    if d[0] >= 0: idx += 1
    if d[1] >= 0: idx += 2
    if d[2] >= 0: idx += 4
    octants[idx] += 1

print(f"\nOctant distribution:")
for i in range(8):
    sx = '+' if (i & 1) else '-'
    sy = '+' if (i & 2) else '-'
    sz = '+' if (i & 4) else '-'
    print(f"  {sx}{sy}{sz}: {octants[i]:4.0f} ({octants[i]/n*100:.1f}%)")

# 当前校准参数
hard_iron = np.array([-16.742652, -841.443205, -756.814842])
soft_iron = np.array([
    [0.993887, -0.001127,  0.002785],
    [-0.001127, 0.984351, -0.019575],
    [0.002785, -0.019575,  1.023357]
])

print(f"\n=== Current calibration params ===")
print(f"Hard Iron: {hard_iron}")
print(f"Soft Iron eigenvalues: {np.linalg.eigvals(soft_iron)}")

# 应用校准
corrected = (data - hard_iron) @ soft_iron.T
norms = np.linalg.norm(corrected, axis=1)

print(f"\n=== After calibration ===")
print(f"Norm: mean={np.mean(norms):.1f}, std={np.std(norms):.1f}")
print(f"Norm range: {norms.min():.1f} ~ {norms.max():.1f}")
print(f"Residual (std/mean): {np.std(norms)/np.mean(norms)*100:.2f}%")
print(f"Max/Min ratio: {norms.max()/norms.min():.4f}")

# XY平面分析
xy_corrected = corrected[:, :2]
cov_xy = np.cov(xy_corrected.T)
eigvals_xy, eigvecs_xy = np.linalg.eig(cov_xy)
aspect_xy = np.sqrt(max(eigvals_xy)/min(eigvals_xy))

print(f"\n=== XY Plane Analysis ===")
print(f"Covariance eigenvalues: {eigvals_xy}")
print(f"Aspect ratio: {aspect_xy:.4f}")

# 角度误差估算
if aspect_xy > 1.01:
    # 椭圆角度误差公式: max_error ≈ arctan((a-b)/(a+b)) ≈ (a-b)/(a+b) rad
    angle_error_deg = np.degrees(np.arctan2(abs(np.sqrt(eigvals_xy[0]) - np.sqrt(eigvals_xy[1])),
                                             np.sqrt(eigvals_xy[0]) + np.sqrt(eigvals_xy[1])))
    print(f"Estimated max angle error: {angle_error_deg:.2f} deg")
else:
    print("XY plane is nearly circular - minimal angle error expected")

# 航向角覆盖
yaw = np.degrees(-np.arctan2(corrected[:,1], corrected[:,0]))
print(f"\nYaw coverage: {yaw.min():.1f} ~ {yaw.max():.1f} deg")

# 检查数据覆盖是否均匀
yaw_bins = np.linspace(-180, 180, 37)
yaw_hist, _ = np.histogram(yaw, bins=yaw_bins)
coverage = np.sum(yaw_hist > 0) / 36 * 100
print(f"Yaw coverage (bins occupied): {coverage:.1f}%")

# 重新拟合椭球
print(f"\n=== Refitting ellipsoid ===")

# 无约束最小二乘法 (PC端使用的方法)
D = np.column_stack([
    data[:,0]**2, data[:,1]**2, data[:,2]**2,
    2*data[:,0]*data[:,1], 2*data[:,0]*data[:,2], 2*data[:,1]*data[:,2],
    2*data[:,0], 2*data[:,1], 2*data[:,2]
])
rhs = np.ones(n)
v, _, _, _ = np.linalg.lstsq(D, rhs, rcond=None)

# 构建椭球矩阵
A = np.array([
    [v[0], v[3], v[4], v[6]],
    [v[3], v[1], v[5], v[7]],
    [v[4], v[5], v[2], v[8]],
    [v[6], v[7], v[8], -1.0]
])

A3 = A[:3, :3]
A34 = A[:3, 3]
center_new = -np.linalg.inv(A3) @ A34

T = np.eye(4)
T[:3, 3] = center_new
R = T.T @ A @ T
R_norm = -R[:3, :3] / R[3, 3]
R_norm = 0.5 * (R_norm + R_norm.T)  # 对称化

evals_new, evecs_new = np.linalg.eigh(R_norm)
radii_new = np.sqrt(1.0 / evals_new)

print(f"New center (hard iron): {center_new}")
print(f"New radii: {radii_new}")
print(f"Radii ratio X/Y: {radii_new[0]/radii_new[1]:.4f}")
print(f"Radii ratio X/Z: {radii_new[0]/radii_new[2]:.4f}")

# 计算新的软铁矩阵
avg_radius = np.mean(radii_new)
scale_matrix = np.diag(avg_radius / radii_new)
soft_iron_new = evecs_new @ scale_matrix @ evecs_new.T

print(f"\nNew soft iron matrix:")
print(soft_iron_new)

# 验证新参数
corrected_new = (data - center_new) @ soft_iron_new.T
norms_new = np.linalg.norm(corrected_new, axis=1)

print(f"\n=== After NEW calibration ===")
print(f"Norm: mean={np.mean(norms_new):.1f}, std={np.std(norms_new):.1f}")
print(f"Residual: {np.std(norms_new)/np.mean(norms_new)*100:.2f}%")
print(f"Max/Min ratio: {norms_new.max()/norms_new.min():.4f}")

# XY平面分析（新参数）
xy_new = corrected_new[:, :2]
cov_xy_new = np.cov(xy_new.T)
eigvals_xy_new, _ = np.linalg.eig(cov_xy_new)
aspect_new = np.sqrt(max(eigvals_xy_new)/min(eigvals_xy_new))
print(f"\nNew XY aspect ratio: {aspect_new:.4f}")

# 诊断结论
print(f"\n{'='*60}")
print("DIAGNOSIS")
print(f"{'='*60}")

if aspect_xy > 1.02:
    print(f"[WARNING] Current calibration has XY aspect ratio {aspect_xy:.4f}")
    print(f"  This causes angle errors up to ~{(aspect_xy-1)*30:.1f} degrees")
    print(f"  Recommend: re-calibrate with better 3D coverage")
else:
    print(f"[OK] XY aspect ratio {aspect_xy:.4f} is good")

if np.std(norms)/np.mean(norms) > 0.05:
    print(f"[WARNING] Residual {np.std(norms)/np.mean(norms)*100:.2f}% is high")
    print(f"  This indicates poor sphere fit after calibration")
else:
    print(f"[OK] Residual {np.std(norms)/np.mean(norms)*100:.2f}% is acceptable")

# 检查象限覆盖
min_octant = octants.min()
max_octant = octants.max()
if min_octant / max_octant < 0.3:
    print(f"[WARNING] Octant coverage is uneven")
    print(f"  Min: {min_octant:.0f}, Max: {max_octant:.0f}")
    print(f"  This may cause calibration inaccuracy")
else:
    print(f"[OK] Octant coverage is reasonably uniform")
