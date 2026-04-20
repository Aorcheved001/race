# -*- coding: utf-8 -*-
import sys
import traceback
import numpy as np
from datetime import datetime

# Intercept uncaught exceptions to prevent silent exit
def excepthook(exc_type, exc_value, exc_traceback):
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return
    print("Uncaught exception:", file=sys.stderr)
    traceback.print_exception(exc_type, exc_value, exc_traceback, file=sys.stderr)

sys.excepthook = excepthook

try:
    import serial
    from PyQt5 import QtWidgets, QtCore
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
except ImportError as e:
    print(f"Import failed: {e}")
    print("Install: pip install pyserial numpy pyqt5 pyqtgraph pyopengl")
    raise RuntimeError(f"Cannot import required packages: {e}")

# =================================================================
# Serial port config
# =================================================================
serialPort = 'COM36'
baudRate = 115200
to = 0.1

try:
    ser = serial.Serial(serialPort, baudRate, timeout=to)
    print(f"Connected to {serialPort}")
except Exception as e:
    print(f"Serial open failed: {e}")
    print(f"Check device and port name: {serialPort}")
    raise RuntimeError(f"Cannot open serial port {serialPort}")

maxPoints = 20000
xRaw = np.empty(0)
yRaw = np.empty(0)
zRaw = np.empty(0)

# Calibration params (initial empty)
calibration = {
    'hard_iron': np.zeros(3),
    'soft_iron': np.eye(3),
    'field_strength': 0.0,
    'quality_score': 0.0,
    'is_calibrated': False
}

# =================================================================
# Ellipsoid fitting calibration algorithm
# =================================================================
def fit_ellipsoid(x, y, z):
    """
    Fit a 3D ellipsoid with an unconstrained least-squares model.

    The algebraic model is:
        ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz = 1

    This avoids the incorrect 2D ellipse constraint that was previously
    applied to the 3D magnetometer ellipsoid fit.
    """

    if len(x) < 9:
        return None

    # Solve D * v = 1 in the least-squares sense.
    D = np.column_stack([
        x * x, y * y, z * z,
        2 * x * y, 2 * x * z, 2 * y * z,
        2 * x, 2 * y, 2 * z
    ])

    try:
        rhs = np.ones(len(x))
        v, _, _, _ = np.linalg.lstsq(D, rhs, rcond=None)
    except np.linalg.LinAlgError:
        return None

    # Build 4x4 quadric matrix
    A = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8], -1.0]
    ])
    
    # Extract center
    A3 = A[:3, :3]
    A34 = A[:3, 3]
    
    try:
        center = -np.linalg.inv(A3) @ A34
    except np.linalg.LinAlgError:
        return None

    if not np.all(np.isfinite(center)):
        return None
    
    # Translate to center: T^T @ A @ T
    T = np.eye(4)
    T[:3, 3] = center
    R = T.T @ A @ T
    
    # Normalize by constant term
    if abs(R[3, 3]) < 1e-10:
        return None
    
    R_norm = -R[:3, :3] / R[3, 3]
    
    # Symmetrize before eigen decomposition to suppress numeric drift.
    R_norm = 0.5 * (R_norm + R_norm.T)

    try:
        evals, evecs = np.linalg.eigh(R_norm)
    except np.linalg.LinAlgError:
        return None

    if not np.all(evals > 0.0):
        return None

    radii = np.sqrt(1.0 / evals)
    
    # Build quadric form matrix
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


def compute_calibration_params(x, y, z, ellipsoid):
    """
    Compute hard iron and soft iron compensation from ellipsoid fit.
    Hard iron = ellipsoid center
    Soft iron = linear transform that maps ellipsoid to unit sphere
    """
    center = ellipsoid['center']
    radii = ellipsoid['radii']
    evecs = ellipsoid['evecs']

    hard_iron = center

    avg_radius = np.mean(radii)
    scale_matrix = np.diag(avg_radius / radii)
    soft_iron = evecs @ scale_matrix @ evecs.T

    field_strength = avg_radius

    return hard_iron, soft_iron, field_strength


def evaluate_calibration_quality(x, y, z, hard_iron, soft_iron, field_strength):
    """
    Evaluate calibration quality, return 0-100 score.

    Revised scoring system (more lenient):
    - Residual: primary indicator, but with relaxed thresholds
    - Radii ratio: secondary indicator
    - Coverage: informational only (no score penalty)
    """
    score = 100.0

    corrected = np.column_stack([x - hard_iron[0], y - hard_iron[1], z - hard_iron[2]])
    corrected = corrected @ soft_iron

    norms = np.linalg.norm(corrected, axis=1)
    residual = np.std(norms) / field_strength * 100

    # Relaxed residual scoring - main quality indicator
    if residual < 5:
        score -= 0      # Excellent
    elif residual < 10:
        score -= 5      # Very good
    elif residual < 15:
        score -= 15     # Good
    elif residual < 20:
        score -= 25     # Acceptable
    else:
        score -= 40     # Needs improvement

    # Relaxed radii ratio scoring
    radii_ratio = np.max(norms) / np.min(norms) if np.min(norms) > 0 else 999
    if radii_ratio < 1.15:
        score -= 0
    elif radii_ratio < 1.25:
        score -= 5
    elif radii_ratio < 1.40:
        score -= 15
    else:
        score -= 25

    # Coverage: informational only, no penalty
    # Low coverage doesn't necessarily mean bad calibration,
    # just means certain orientations weren't sampled
    coverage = estimate_sphere_coverage(x, y, z)

    score = max(0.0, min(100.0, score))
    return score, residual, radii_ratio, coverage


def estimate_sphere_coverage(x, y, z):
    """
    Estimate data coverage percentage on sphere surface.
    Divide sphere into 20x20 grid, count occupied cells.
    """
    n_bins = 20
    theta = np.arctan2(y, x)
    phi = np.arccos(np.clip(z / np.sqrt(x*x + y*y + z*z + 1e-10), -1, 1))

    theta_bins = np.linspace(-np.pi, np.pi, n_bins + 1)
    phi_bins = np.linspace(0, np.pi, n_bins + 1)

    H, _, _ = np.histogram2d(theta, phi, bins=[theta_bins, phi_bins])

    occupied = np.sum(H > 0)
    total = n_bins * n_bins
    coverage = (occupied / total) * 100

    return coverage


def run_calibration(x, y, z):
    """
    Execute full calibration pipeline.
    """
    if len(x) < 100:
        return False

    ellipsoid = fit_ellipsoid(x, y, z)
    if ellipsoid is None:
        return False

    hard_iron, soft_iron, field_strength = compute_calibration_params(x, y, z, ellipsoid)

    quality_score, residual, radii_ratio, coverage = evaluate_calibration_quality(
        x, y, z, hard_iron, soft_iron, field_strength
    )

    calibration['hard_iron'] = hard_iron
    calibration['soft_iron'] = soft_iron
    calibration['field_strength'] = field_strength
    calibration['quality_score'] = quality_score
    calibration['is_calibrated'] = True

    print(f"\n{'='*50}")
    print(f"  Magnetometer Calibration Result (points: {len(x)})")
    print(f"{'='*50}")
    print(f"  Hard Iron Offset:")
    print(f"    X: {hard_iron[0]:.2f}")
    print(f"    Y: {hard_iron[1]:.2f}")
    print(f"    Z: {hard_iron[2]:.2f}")
    print(f"\n  Soft Iron Matrix:")
    for i in range(3):
        print(f"    [{soft_iron[i, 0]:.4f}, {soft_iron[i, 1]:.4f}, {soft_iron[i, 2]:.4f}]")
    print(f"\n  Field Strength: {field_strength:.2f}")
    print(f"  Ellipsoid Radii: X={ellipsoid['radii'][0]:.2f}, Y={ellipsoid['radii'][1]:.2f}, Z={ellipsoid['radii'][2]:.2f}")
    print(f"\n  Quality Score: {quality_score:.1f}/100")
    print(f"  Residual (RMS): {residual:.2f}%")
    print(f"  Sphere Coverage: {coverage:.1f}%")
    print(f"  Max/Min Radius Ratio: {radii_ratio:.3f}")
    print(f"{'='*50}\n")

    return True


def export_calibration_to_c(hard_iron, soft_iron, field_strength):
    """
    Generate embedded C code format calibration params.
    """
    c_code = f"""
/* Magnetometer calibration params - auto-generated at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} */
/* Data points: {len(xRaw)}, Quality: {calibration['quality_score']:.1f}/100 */

#define MAG_HARD_IRON_X  {hard_iron[0]:.2f}f
#define MAG_HARD_IRON_Y  {hard_iron[1]:.2f}f
#define MAG_HARD_IRON_Z  {hard_iron[2]:.2f}f

#define MAG_SOFT_IRON_XX {soft_iron[0, 0]:.6f}f
#define MAG_SOFT_IRON_XY {soft_iron[0, 1]:.6f}f
#define MAG_SOFT_IRON_XZ {soft_iron[0, 2]:.6f}f
#define MAG_SOFT_IRON_YX {soft_iron[1, 0]:.6f}f
#define MAG_SOFT_IRON_YY {soft_iron[1, 1]:.6f}f
#define MAG_SOFT_IRON_YZ {soft_iron[1, 2]:.6f}f
#define MAG_SOFT_IRON_ZX {soft_iron[2, 0]:.6f}f
#define MAG_SOFT_IRON_ZY {soft_iron[2, 1]:.6f}f
#define MAG_SOFT_IRON_ZZ {soft_iron[2, 2]:.6f}f

#define MAG_FIELD_STRENGTH {field_strength:.2f}f
"""
    return c_code


# =================================================================
# Init PyQt5 App
# =================================================================
app = QtWidgets.QApplication([])
mainWindow = QtWidgets.QWidget()
mainWindow.setWindowTitle("Magnetometer Calibration 3D")
mainWindow.resize(1200, 800)

mainLayout = QtWidgets.QHBoxLayout()
mainWindow.setLayout(mainLayout)

# Left layout: three 2D plots
leftWidget = QtWidgets.QWidget()
plotLayout = QtWidgets.QGridLayout()
leftWidget.setLayout(plotLayout)
mainLayout.addWidget(leftWidget)

xyPlot = pg.PlotWidget(title="XY Plane")
yzPlot = pg.PlotWidget(title="YZ Plane")
xzPlot = pg.PlotWidget(title="XZ Plane")

for plot in [xyPlot, yzPlot, xzPlot]:
    plot.setAspectLocked(True)
    plot.showGrid(x=True, y=True)
    plot.setMinimumSize(300, 300)

plotLayout.addWidget(xyPlot, 0, 0)
plotLayout.addWidget(yzPlot, 0, 1)
plotLayout.addWidget(xzPlot, 1, 0)

xyScatter = pg.ScatterPlotItem(size=5)
yzScatter = pg.ScatterPlotItem(size=5)
xzScatter = pg.ScatterPlotItem(size=5)

xyPlot.addItem(xyScatter)
yzPlot.addItem(yzScatter)
xzPlot.addItem(xzScatter)

# Right layout: 3D view + calibration info panel
rightWidget = QtWidgets.QWidget()
rightLayout = QtWidgets.QVBoxLayout()
rightWidget.setLayout(rightLayout)

view3D = gl.GLViewWidget()
view3D.setMinimumSize(400, 400)
view3D.setCameraPosition(distance=10000)
rightLayout.addWidget(view3D)

grid3D = gl.GLGridItem()
grid3D.scale(500, 500, 500)
view3D.addItem(grid3D)

scatter3D = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=10, color=(1.0, 1.0, 1.0, 0.5))
view3D.addItem(scatter3D)

# Calibration info panel
infoPanel = QtWidgets.QTextEdit()
infoPanel.setReadOnly(True)
infoPanel.setMinimumHeight(200)
infoPanel.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
rightLayout.addWidget(infoPanel)

mainLayout.addWidget(rightWidget)


def update_info_panel():
    """Update calibration info panel display."""
    n_points = len(xRaw)

    if n_points == 0:
        infoPanel.setPlainText("Waiting for data...\n\nRotate device in 3D space to collect magnetometer data.\nRecommendation: slowly rotate in all directions.")
        return

    stats = (
        f"=== Data Collection ===\n"
        f"Points: {n_points}\n"
        f"X range: [{xRaw.min():.1f}, {xRaw.max():.1f}]\n"
        f"Y range: [{yRaw.min():.1f}, {yRaw.max():.1f}]\n"
        f"Z range: [{zRaw.min():.1f}, {zRaw.max():.1f}]\n"
    )

    if calibration['is_calibrated']:
        hi = calibration['hard_iron']
        si = calibration['soft_iron']
        fs = calibration['field_strength']
        score = calibration['quality_score']

        if score >= 80:
            grade = "Excellent"
        elif score >= 60:
            grade = "Good"
        elif score >= 40:
            grade = "Fair"
        else:
            grade = "Poor"

        stats += (
            f"\n=== Calibration Result ===\n"
            f"Quality Score: {score:.1f}/100 ({grade})\n"
            f"\nHard Iron:\n"
            f"  X: {hi[0]:.2f}\n"
            f"  Y: {hi[1]:.2f}\n"
            f"  Z: {hi[2]:.2f}\n"
            f"\nField Strength: {fs:.2f}\n"
            f"\nSoft Iron Matrix:\n"
            f"  [{si[0,0]:.4f}, {si[0,1]:.4f}, {si[0,2]:.4f}]\n"
            f"  [{si[1,0]:.4f}, {si[1,1]:.4f}, {si[1,2]:.4f}]\n"
            f"  [{si[2,0]:.4f}, {si[2,1]:.4f}, {si[2,2]:.4f}]\n"
        )
    else:
        stats += "\nCalibration: Not calibrated\n"
        if n_points >= 100:
            stats += "(Data sufficient, click 'Run Calibration' to start)"
        else:
            stats += f"(Need at least 100 points, current: {n_points})"

    infoPanel.setPlainText(stats)


def run_calibration_and_update():
    """Button callback: run calibration and update display."""
    if len(xRaw) < 100:
        infoPanel.setPlainText(f"Insufficient data! Current: {len(xRaw)} points, need at least 100.")
        return

    success = run_calibration(xRaw, yRaw, zRaw)

    if success:
        score = calibration['quality_score']

        # Determine quality grade
        if score >= 80:
            grade = "Excellent"
            suggestion = "Calibration quality is great!"
        elif score >= 60:
            grade = "Good"
            suggestion = "Calibration is acceptable. Consider recollecting data for better results."
        elif score >= 40:
            grade = "Fair"
            suggestion = "Calibration may work but accuracy could be improved. Try rotating in more directions."
        else:
            grade = "Poor"
            suggestion = "Calibration saved but quality is low. Recommend recollecting with more thorough rotation."

        # Always save calibration params
        c_code = export_calibration_to_c(
            calibration['hard_iron'],
            calibration['soft_iron'],
            calibration['field_strength']
        )

        import os
        output_file = os.path.join(os.path.dirname(__file__), "..", "code", "ins", "calibration_params.h")
        with open(output_file, 'w', encoding='gb2312') as f:
            f.write(c_code)

        info_text = infoPanel.toPlainText()
        info_text += f"\n\n{'='*50}\n"
        info_text += f"=== CALIBRATION SAVED ===\n"
        info_text += f"{'='*50}\n"
        info_text += f"Quality: {score:.1f}/100 ({grade})\n"
        info_text += f"Points: {len(xRaw)}\n"
        info_text += f"File: {output_file}\n\n"
        info_text += f"Suggestion: {suggestion}\n\n"
        info_text += "NOTE: Embedded firmware now uses calibration_params.h by default.\n"
        info_text += "  1. Collect about 60s of data on PC side\n"
        info_text += "  2. Run calibration to overwrite calibration_params.h\n"
        info_text += "  3. Rebuild and reboot, then the new params load automatically\n"
        info_text += "  4. On-chip calibration code is still retained for debugging use\n"
        infoPanel.setPlainText(info_text)

        print(f"\nCalibration params exported to: {output_file}")
        print(f"Quality Score: {score:.1f}/100 ({grade})")
    else:
        infoPanel.setPlainText("Calibration failed! Data may not fit ellipsoid distribution, please recollect.")

    update_info_panel()


# Add calibration button
calButton = QtWidgets.QPushButton("Run Calibration")
calButton.setStyleSheet("font-size: 14px; padding: 8px;")
calButton.clicked.connect(run_calibration_and_update)
rightLayout.addWidget(calButton)


def updatePlot():
    global xRaw, yRaw, zRaw

    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if not line:
                continue

            values = line.split(',')
            if len(values) == 3:
                try:
                    x = float(values[0])
                    y = float(values[1])
                    z = float(values[2])
                except ValueError:
                    continue

                xRaw = np.append(xRaw, x)[-maxPoints:]
                yRaw = np.append(yRaw, y)[-maxPoints:]
                zRaw = np.append(zRaw, z)[-maxPoints:]

                xyScatter.setData(x=xRaw, y=yRaw, brush=pg.mkBrush(0, 0, 255, 120))
                yzScatter.setData(x=yRaw, y=zRaw, brush=pg.mkBrush(0, 255, 0, 120))
                xzScatter.setData(x=xRaw, y=zRaw, brush=pg.mkBrush(255, 0, 0, 120))

                pos3d = np.column_stack((xRaw, yRaw, zRaw))
                color3d = np.ones((len(xRaw), 4))
                color3d[:, 0] = 0.2
                color3d[:, 1] = 0.8
                color3d[:, 2] = 1.0
                color3d[:, 3] = 0.6

                scatter3D.setData(pos=pos3d, color=color3d)

                update_info_panel()

        except Exception:
            pass


plotTimer = QtCore.QTimer()
plotTimer.timeout.connect(updatePlot)
plotTimer.start(50)

mainWindow.show()

try:
    app.exec_()
finally:
    ser.close()
