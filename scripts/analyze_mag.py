import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import sys

def fit_ellipsoid(x, y, z):
    # D array
    D = np.array([
        x * x,
        y * y,
        z * z,
        2 * x * y,
        2 * x * z,
        2 * y * z,
        2 * x,
        2 * y,
        2 * z,
        np.ones_like(x)
    ]).T

    # S matrix
    S = D.T @ D

    # S * v = 0 -> S is 10x10, to avoid trivial solution v=0, we constraint v[9] or similar
    # Using unconstrained least squares: min ||D * v||^2 s.t. ||v|| = 1
    _, _, V = np.linalg.svd(D)
    v = V[-1, :]

    A = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8], v[9]]
    ])

    A3 = A[:3, :3]
    A34 = A[:3, 3]

    try:
        center = -np.linalg.inv(A3) @ A34
    except np.linalg.LinAlgError:
        return None

    T = np.eye(4)
    T[:3, 3] = center
    R = T.T @ A @ T

    if abs(R[3, 3]) < 1e-10:
        return None

    R_norm = -R[:3, :3] / R[3, 3]

    evals, evecs = np.linalg.eig(R_norm)
    evals = np.real(evals)
    evecs = np.real(evecs)

    if np.all(evals > 0):
        radii = np.sqrt(1.0 / evals)
    else:
        radii = np.array([np.std(x), np.std(y), np.std(z)]) * 2

    Q = evecs @ np.diag(1.0 / (radii ** 2)) @ evecs.T

    A_out = np.eye(4)
    A_out[:3, :3] = Q
    A_out[3, 3] = -1.0

    return {
        'center': center,
        'radii': radii,
        'evecs': evecs,
        'evals': evals,
        'Q': Q,
        'A': A_out
    }

def evaluate_calibration_quality(x, y, z, hard_iron, soft_iron, field_strength):
    score = 100.0
    corrected = np.column_stack([x - hard_iron[0], y - hard_iron[1], z - hard_iron[2]])
    corrected = corrected @ soft_iron

    norms = np.linalg.norm(corrected, axis=1)
    residual = np.std(norms) / field_strength * 100

    if residual < 5: score -= 0
    elif residual < 10: score -= 5
    elif residual < 15: score -= 15
    elif residual < 20: score -= 25
    else: score -= 40
    
    return max(0.0, score), residual

def main():
    # 获取脚本所在目录，构建相对路径
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
    DATA_DIR = os.path.join(PROJECT_ROOT, 'data')
    
    filepath = os.path.join(DATA_DIR, 'serial_log_COM10_20260415_221021.txt')
    
    xs, ys, zs = [], [], []
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip(): continue
            if ']' in line:
                data_part = line.split(']')[1].strip()
                parts = data_part.split(',')
                if len(parts) == 3:
                    try:
                        xs.append(float(parts[0]))
                        ys.append(float(parts[1]))
                        zs.append(float(parts[2]))
                    except ValueError:
                        pass
    
    x = np.array(xs)
    y = np.array(ys)
    z = np.array(zs)
    
    ellipsoid = fit_ellipsoid(x, y, z)
    if not ellipsoid:
        print("Failed to fit ellipsoid.")
        return
        
    center = ellipsoid['center']
    radii = ellipsoid['radii']
    evecs = ellipsoid['evecs']
    
    hard_iron = center
    avg_radius = np.mean(radii)
    scale_matrix = np.diag(avg_radius / radii)
    soft_iron = evecs @ scale_matrix @ evecs.T
    field_strength = avg_radius
    
    score, residual = evaluate_calibration_quality(x, y, z, hard_iron, soft_iron, field_strength)
    
    print(f"Data Points: {len(x)}")
    print(f"Hard Iron (Center): {hard_iron[0]:.2f}, {hard_iron[1]:.2f}, {hard_iron[2]:.2f}")
    print(f"Radii: {radii[0]:.2f}, {radii[1]:.2f}, {radii[2]:.2f}")
    print(f"Field Strength: {field_strength:.2f}")
    print(f"Quality Score: {score:.1f}/100 (Residual: {residual:.2f}%)")
    print("Soft Iron Matrix:")
    print(soft_iron)
    
    # Plotting
    corrected = np.column_stack([x - hard_iron[0], y - hard_iron[1], z - hard_iron[2]]) @ soft_iron
    
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Magnetometer Calibration Analysis', fontsize=16)
    
    # Raw data
    axs[0, 0].scatter(x, y, s=2, alpha=0.5, c='b')
    axs[0, 0].set_title('Raw XY')
    axs[0, 0].axis('equal')
    
    axs[0, 1].scatter(x, z, s=2, alpha=0.5, c='r')
    axs[0, 1].set_title('Raw XZ')
    axs[0, 1].axis('equal')
    
    axs[0, 2].scatter(y, z, s=2, alpha=0.5, c='g')
    axs[0, 2].set_title('Raw YZ')
    axs[0, 2].axis('equal')
    
    # Corrected data
    axs[1, 0].scatter(corrected[:, 0], corrected[:, 1], s=2, alpha=0.5, c='b')
    axs[1, 0].set_title('Corrected XY')
    axs[1, 0].axis('equal')
    
    axs[1, 1].scatter(corrected[:, 0], corrected[:, 2], s=2, alpha=0.5, c='r')
    axs[1, 1].set_title('Corrected XZ')
    axs[1, 1].axis('equal')
    
    axs[1, 2].scatter(corrected[:, 1], corrected[:, 2], s=2, alpha=0.5, c='g')
    axs[1, 2].set_title('Corrected YZ')
    axs[1, 2].axis('equal')
    
    plt.tight_layout()
    plot_path = os.path.join(DATA_DIR, 'calibration_analysis_20260415.png')
    plt.savefig(plot_path)
    print(f"Plot saved to {plot_path}")

if __name__ == '__main__':
    main()