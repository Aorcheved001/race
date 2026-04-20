import re
import numpy as np
from numpy.linalg import lstsq, inv, eig
from scipy.linalg import eig as scipy_eig

# Load data
data = open('serial_log_COM24_20260411_164927.txt', 'r', encoding='utf-8').read()
lines = [l.strip() for l in data.split('\n') if not l.startswith('#') and l.strip()]
xs, ys, zs = [], [], []
for l in lines:
    m = re.search(r'\] (.+)', l)
    if m:
        parts = m.group(1).split(',')
        if len(parts) == 3:
            xs.append(float(parts[0]))
            ys.append(float(parts[1]))
            zs.append(float(parts[2]))

x = np.array(xs)
y = np.array(ys)
z = np.array(zs)

print(f'=== Data Overview ===')
print(f'Points: {len(x)}')
print(f'X: {x.min():.0f} ~ {x.max():.0f} (range={x.max()-x.min():.0f})')
print(f'Y: {y.min():.0f} ~ {y.max():.0f} (range={y.max()-y.min():.0f})')
print(f'Z: {z.min():.0f} ~ {z.max():.0f} (range={z.max()-z.min():.0f})')

# Algebraic constrained ellipsoid fit
D = np.column_stack([
    x*x, y*y, z*z,
    2*x*y, 2*x*z, 2*y*z,
    2*x, 2*y, 2*z
])
d2 = np.ones(len(x))
S = D.T @ D

C = np.zeros((9, 9))
C[0, 2] = C[2, 0] = 2.0
C[1, 1] = -1.0

eigenvalues, eigenvectors = scipy_eig(S, C)

best_v = None
best_lambda = -np.inf

for i in range(len(eigenvalues)):
    lam = np.real(eigenvalues[i])
    v = np.real(eigenvectors[:, i])
    constraint_val = 4 * v[0] * v[2] - v[3]**2
    if constraint_val > 0 and lam > best_lambda:
        best_lambda = lam
        best_v = v

if best_v is None:
    best_v, _, _, _ = lstsq(D, d2, rcond=None)

v = best_v

A = np.array([
    [v[0], v[3], v[4], v[6]],
    [v[3], v[1], v[5], v[7]],
    [v[4], v[5], v[2], v[8]],
    [v[6], v[7], v[8], -1.0]
])

A3 = A[:3, :3]
A34 = A[:3, 3]
center = -inv(A3) @ A34

T = np.eye(4)
T[:3, 3] = center
R = T.T @ A @ T
R_norm = -R[:3, :3] / R[3, 3]

evals, evecs = eig(R_norm)
evals = np.real(evals)
evecs = np.real(evecs)

print(f'\n=== Ellipsoid Fit ===')
print(f'Constraint: {4*v[0]*v[2] - v[3]**2:.6f}')
print(f'Center (Hard Iron): X={center[0]:.2f}, Y={center[1]:.2f}, Z={center[2]:.2f}')

if np.all(evals > 0):
    radii = np.sqrt(1.0 / evals)
    print(f'Radii: X={radii[0]:.1f}, Y={radii[1]:.1f}, Z={radii[2]:.1f}')
    print(f'Valid ellipsoid: YES')
    
    avg_r = np.mean(radii)
    scale = np.diag(avg_r / radii)
    soft_iron = evecs @ scale @ evecs.T
    
    corrected = np.column_stack([x-center[0], y-center[1], z-center[2]]) @ soft_iron
    norms = np.linalg.norm(corrected, axis=1)
    
    ratio = np.max(norms) / np.min(norms)
    residual = np.std(norms) / np.mean(norms) * 100
    
    # Coverage estimate
    n_bins = 20
    theta = np.arctan2(y-center[1], x-center[0])
    phi = np.arccos(np.clip((z-center[2]) / np.sqrt((x-center[0])**2 + (y-center[1])**2 + (z-center[2])**2 + 1e-10), -1, 1))
    theta_bins = np.linspace(-np.pi, np.pi, n_bins + 1)
    phi_bins = np.linspace(0, np.pi, n_bins + 1)
    H, _, _ = np.histogram2d(theta, phi, bins=[theta_bins, phi_bins])
    coverage = np.sum(H > 0) / (n_bins * n_bins) * 100
    
    if ratio < 1.05:
        quality = 100
    elif ratio < 1.1:
        quality = 90
    elif ratio < 1.2:
        quality = 80
    elif ratio < 1.5:
        quality = 60
    elif ratio < 2.0:
        quality = 40
    else:
        quality = max(0, 100 - (ratio - 1) * 50)
    
    print(f'\n=== After Calibration ===')
    print(f'Radius mean: {np.mean(norms):.1f}')
    print(f'Radius std: {np.std(norms):.1f}')
    print(f'Residual: {residual:.1f}%')
    print(f'Max/Min ratio: {ratio:.3f}')
    print(f'Coverage: {coverage:.1f}%')
    print(f'Quality: {quality:.0f}/100')
    
    print(f'\n=== C Code ===')
    print(f'#define MAG_HARD_IRON_X  {center[0]:.2f}f')
    print(f'#define MAG_HARD_IRON_Y  {center[1]:.2f}f')
    print(f'#define MAG_HARD_IRON_Z  {center[2]:.2f}f')
    print()
    print('float soft_iron[3][3] = {')
    for row in soft_iron:
        print(f'    {{{row[0]:.6f}f, {row[1]:.6f}f, {row[2]:.6f}f}},')
    print('};')
else:
    print(f'Valid ellipsoid: NO')
    print(f'Eigenvalues: {evals}')
