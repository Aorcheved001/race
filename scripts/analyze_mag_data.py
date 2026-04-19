# -*- coding: utf-8 -*-
import re
import sys
import os
import numpy as np

# 获取脚本所在目录，构建相对路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(PROJECT_ROOT, 'data')

if len(sys.argv) > 1:
    logfile = sys.argv[1]
else:
    logfile = os.path.join(DATA_DIR, 'serial_log_COM24_20260414_143752.txt')
lines = open(logfile, 'r', encoding='utf-8').readlines()
data = []
for line in lines:
    m = re.search(r'\] ([-\d.]+),([-\d.]+),([-\d.]+)', line)
    if m:
        data.append([float(m.group(1)), float(m.group(2)), float(m.group(3))])
data = np.array(data)
n = len(data)
print(f'Data points: {n}')

# Octant distribution
octants = np.zeros(8)
for d in data:
    idx = 0
    if d[0] >= 0: idx += 1
    if d[1] >= 0: idx += 2
    if d[2] >= 0: idx += 4
    octants[idx] += 1
print('\nOctant distribution:')
for i in range(8):
    sx = '+' if (i & 1) else '-'
    sy = '+' if (i & 2) else '-'
    sz = '+' if (i & 4) else '-'
    print(f'  {sx}{sy}{sz}: {octants[i]:4.0f} ({octants[i]/n*100:.1f}%)')

# Data center and radius
center = np.mean(data, axis=0)
print(f'\nData center (mean): X={center[0]:.1f}, Y={center[1]:.1f}, Z={center[2]:.1f}')
radii = np.linalg.norm(data - center, axis=1)
print(f'Radius: mean={np.mean(radii):.1f}, std={np.std(radii):.1f}, min={np.min(radii):.1f}, max={np.max(radii):.1f}')
print(f'Radius ratio max/min: {np.max(radii)/np.min(radii):.3f}')

# Spherical coverage
azimuth = np.arctan2(data[:,1]-center[1], data[:,0]-center[0])
elevation = np.arcsin((data[:,2]-center[2]) / (radii + 1e-10))
az_deg = np.degrees(azimuth)
el_deg = np.degrees(elevation)
print(f'\nAzimuth range: {np.min(az_deg):.1f} to {np.max(az_deg):.1f}')
print(f'Elevation range: {np.min(el_deg):.1f} to {np.max(el_deg):.1f}')

el_bins = np.linspace(-90, 90, 19)
el_hist, _ = np.histogram(el_deg, bins=el_bins)
print('\nElevation distribution:')
for i in range(len(el_hist)):
    bar = '#' * int(el_hist[i] / 20)
    print(f'  {el_bins[i]:6.1f}~{el_bins[i+1]:6.1f}: {el_hist[i]:4d} {bar}')

# ---- Ellipsoid fitting ----
D = np.column_stack([
    data[:,0]**2, data[:,1]**2, data[:,2]**2,
    2*data[:,0]*data[:,1], 2*data[:,0]*data[:,2], 2*data[:,1]*data[:,2],
    2*data[:,0], 2*data[:,1], 2*data[:,2]
])
S = D.T @ D
b_vec = D.T @ np.ones(n)

C = np.zeros((9,9))
C[0,2] = 2.0; C[2,0] = 2.0; C[1,1] = -1.0

L = np.linalg.cholesky(S)
Y = np.linalg.solve(L, C)
A = Y @ np.linalg.inv(L).T
A = 0.5 * (A + A.T)

eigvals, eigvecs = np.linalg.eigh(A)
v_raw = np.linalg.solve(L.T, eigvecs)

best_idx = -1
best_lambda = -1e30
for j in range(9):
    con = 4.0 * v_raw[0,j] * v_raw[2,j] - v_raw[3,j]**2
    if con > 0 and eigvals[j] > best_lambda:
        best_lambda = eigvals[j]
        best_idx = j

if best_idx >= 0:
    v = v_raw[:, best_idx]
else:
    v = np.linalg.solve(S, b_vec)
    print('\nFallback to unconstrained least squares')

Q = np.array([[v[0], v[3], v[4]],
              [v[3], v[1], v[5]],
              [v[4], v[5], v[2]]])
u = v[6:9]
bias = -np.linalg.solve(Q, u)

sf = 1.0 + bias @ u
Qn = Q / sf
eigvals3, eigvecs3 = np.linalg.eigh(Qn)

print(f'\n===== Calibration Result =====')
print(f'Hard Iron: X={bias[0]:.2f}, Y={bias[1]:.2f}, Z={bias[2]:.2f}')
print(f'Scale factor: {sf:.4f}')
print(f'Qn eigenvalues: {eigvals3}')

if np.all(eigvals3 > 0):
    sqrt_inv = 1.0 / np.sqrt(eigvals3)
    S_mat = eigvecs3 @ np.diag(sqrt_inv) @ eigvecs3.T
    centered = data - bias
    cal_radii = np.linalg.norm(centered @ S_mat.T, axis=1)
    mean_r = np.mean(cal_radii)
    std_r = np.std(cal_radii)
    rms_residual = std_r / mean_r * 100
    ratio = np.max(cal_radii) / np.min(cal_radii)
    score = max(0, 100 - rms_residual * 10 - abs(1 - ratio) * 30)
    print(f'\nSoft Iron Matrix:')
    for row in S_mat:
        print(f'  [{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}]')
    print(f'\nField Strength: {mean_r:.2f}')
    print(f'Calibrated Radii: min={np.min(cal_radii):.2f}, max={np.max(cal_radii):.2f}')
    print(f'Quality Score: {score:.1f}/100')
    print(f'Residual (RMS): {rms_residual:.2f}%')
    print(f'Max/Min Ratio: {ratio:.3f}')
    if score > 80:
        print('\n*** EXCELLENT ***')
    elif score > 60:
        print('\n*** GOOD ***')
    elif score > 40:
        print('\n*** FAIR - need more 3D coverage ***')
    else:
        print('\n*** POOR ***')
else:
    print('\nQn has negative eigenvalues - data does not form a proper ellipsoid')
    print('This means the data is not distributed on a sphere surface')
