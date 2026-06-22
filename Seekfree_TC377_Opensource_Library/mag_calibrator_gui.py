# -*- coding: utf-8 -*-
"""Magnetometer 2D Offline Calibration GUI - PySide6 + pyqtgraph"""

import json, os, re, sys, time, math
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports
from PySide6.QtCore import QTimer, QThread, Signal
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QSplitter, QStatusBar, QTabWidget, QTextEdit,
    QVBoxLayout, QWidget,
)
from pyqtgraph import PlotWidget, mkPen, mkBrush

DEFAULT_HEADER = Path("code/ins/calibration_params.h")
DATA_DIR = Path("track_data")

MODE_INFO = {
    "raw": {"prefix": "mag_raw", "c_function": "imu_mag_send_raw_data_to_pc()",
            "header_comment": "raw magnetometer x,y,z", "data_field": "imu660.data_Raw.mag_x/y/z"},
    "calibrated": {"prefix": "mag_calibrated", "c_function": "imu_mag_send_calibrated_data_to_pc()",
                   "header_comment": "calibrated magnetometer x,y,z", "data_field": "imu660.data_Ripen.mag_x/y/z"},
}

RAW_ACCEPT = {"min_bins": 34, "max_gap_deg": 30.0, "min_span_deg": 330.0,
              "max_ratio": 1.15, "max_residual_pct": 3.0, "soft_iron_range": (0.75, 1.30)}
CAL_ACCEPT = {"max_center_offset": 50.0, "max_radius_ratio_good": 1.08,
              "max_radius_ratio_ok": 1.12, "max_cv_good_pct": 4.0,
              "max_cv_ok_pct": 6.0, "z_diff_warning": 500.0}

NUMBER_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
CSV_RE = re.compile(rf"^\s*(?:\[[^\]]*\]\s*)?({NUMBER_RE})\s*,\s*({NUMBER_RE})\s*,\s*({NUMBER_RE})\s*$")


def load_mag_data(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            m = CSV_RE.match(line)
            if m:
                rows.append([float(m.group(1)), float(m.group(2)), float(m.group(3))])
    if not rows:
        raise ValueError("No magnetometer samples found.")
    return np.asarray(rows, dtype=float)


def filter_valid(data, min_norm=50.0):
    return data[np.linalg.norm(data, axis=1) > min_norm]


def _read_define(text, name):
    m = re.search(rf"#define\s+{re.escape(name)}\s+([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)f?", text)
    if not m:
        raise ValueError(f"Missing define: {name}")
    return float(m.group(1))


def load_header(path):
    text = Path(path).read_text(encoding="utf-8")
    bias = np.array([
        _read_define(text, "MAG_HARD_IRON_X"),
        _read_define(text, "MAG_HARD_IRON_Y"),
        _read_define(text, "MAG_HARD_IRON_Z"),
    ], dtype=float)
    soft_iron = np.array([
        [_read_define(text, "MAG_SOFT_IRON_XX"), _read_define(text, "MAG_SOFT_IRON_XY"), _read_define(text, "MAG_SOFT_IRON_XZ")],
        [_read_define(text, "MAG_SOFT_IRON_YX"), _read_define(text, "MAG_SOFT_IRON_YY"), _read_define(text, "MAG_SOFT_IRON_YZ")],
        [_read_define(text, "MAG_SOFT_IRON_ZX"), _read_define(text, "MAG_SOFT_IRON_ZY"), _read_define(text, "MAG_SOFT_IRON_ZZ")],
    ], dtype=float)
    field = _read_define(text, "MAG_FIELD_STRENGTH")
    return bias, soft_iron, field


def fit_ellipse_2d(data):
    if len(data) < 6:
        raise ValueError(f"Insufficient data for 2D ellipse fit: {len(data)} samples (need >= 6)")
    x, y = data[:, 0], data[:, 1]
    design = np.column_stack([x * x, 2.0 * x * y, y * y, 2.0 * x, 2.0 * y])
    coeff, *_ = np.linalg.lstsq(design, np.ones(len(data)), rcond=None)
    q = np.array([[coeff[0], coeff[1]], [coeff[1], coeff[2]]], dtype=float)
    u = coeff[3:5]
    center = -np.linalg.solve(q, u)
    rhs = 1.0 - center @ u
    if rhs <= 0:
        q, u = -q, -u
        center = -np.linalg.solve(q, u)
        rhs = 1.0 - center @ u
    if rhs <= 0:
        raise ValueError("Invalid ellipse fit")
    q_norm = 0.5 * (q / rhs + (q / rhs).T)
    eigvals, eigvecs = np.linalg.eigh(q_norm)
    # Clamp tiny negative eigenvalues (numerical noise) to a small positive value
    eigvals = np.where(eigvals < 0, np.where(eigvals > -1e-6, 1e-10, eigvals), eigvals)
    if np.any(eigvals <= 0):
        raise ValueError(f"Non-positive eigenvalues: {eigvals}")
    radii = 1.0 / np.sqrt(eigvals)
    field_strength = float(np.mean(radii))
    soft_xy = eigvecs @ np.diag(field_strength / radii) @ eigvecs.T
    soft_xy = 0.5 * (soft_xy + soft_xy.T)
    return center, soft_xy, field_strength, radii


def build_2d_calibration(data, preserve_header):
    old_bias, old_soft, old_field = load_header(preserve_header)
    center_xy, soft_xy, field_strength, radii = fit_ellipse_2d(data)
    bias = old_bias.copy()
    bias[0:2] = center_xy
    soft_iron = np.zeros((3, 3), dtype=float)
    soft_iron[0:2, 0:2] = soft_xy
    soft_iron[2, 2] = old_soft[2, 2] if abs(old_soft[2, 2]) > 1e-6 else 1.0
    return bias, soft_iron, field_strength if field_strength > 0 else old_field, radii


def apply_calibration(data, bias, soft_iron):
    return (data - bias) @ soft_iron.T


def yaw_coverage_metrics(xy, bins=36):
    yaw = np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))
    hist, _ = np.histogram(yaw, bins=np.linspace(-180.0, 180.0, bins + 1))
    occupied = int(np.sum(hist > 0))
    wrapped = np.sort(np.mod(yaw, 360.0))
    if len(wrapped) > 1:
        gaps = np.diff(np.concatenate([wrapped, [wrapped[0] + 360.0]]))
        largest_gap = float(np.max(gaps))
    else:
        largest_gap = 360.0
    unwrapped = np.degrees(np.unwrap(np.radians(yaw)))
    unwrapped_span = float(np.max(unwrapped) - np.min(unwrapped)) if len(unwrapped) > 1 else 0.0
    nonzero = hist[hist > 0]
    balance = float(nonzero.min() / nonzero.max()) if len(nonzero) else 0.0
    return {
        "yaw_coverage_pct": float(occupied / bins * 100),
        "yaw_bins_occupied": occupied, "yaw_bin_count": bins,
        "yaw_bin_balance": balance, "yaw_largest_gap_deg": largest_gap,
        "yaw_circular_span_deg": float(max(0, 360.0 - largest_gap)),
        "yaw_unwrapped_span_deg": unwrapped_span,
    }


def quality_report_2d(data, corrected):
    xy = corrected[:, :2]
    try:
        design = np.column_stack([xy[:, 0]**2, 2*xy[:, 0]*xy[:, 1], xy[:, 1]**2, 2*xy[:, 0], 2*xy[:, 1]])
        coeff, *_ = np.linalg.lstsq(design, np.ones(len(xy)), rcond=None)
        q = np.array([[coeff[0], coeff[1]], [coeff[1], coeff[2]]], dtype=float)
        u = coeff[3:5]
        fit_center = -np.linalg.solve(q, u)
    except Exception:
        fit_center = np.array([xy[:, 0].mean(), xy[:, 1].mean()])
    r_from_center = np.sqrt((xy[:, 0] - fit_center[0])**2 + (xy[:, 1] - fit_center[1])**2)
    r_from_origin = np.linalg.norm(xy, axis=1)
    residual = float(np.std(r_from_center) / np.mean(r_from_center) * 100)
    ratio = float(np.max(r_from_center) / np.min(r_from_center)) if np.min(r_from_center) > 0 else 999.0
    ym = yaw_coverage_metrics(xy)
    score = 100.0
    score -= min(55.0, residual * 10.0)
    score -= min(30.0, abs(ratio - 1.0) * 100.0)
    score -= max(0, 90.0 - ym["yaw_coverage_pct"]) * 0.4
    score -= max(0, ym["yaw_largest_gap_deg"] - 30.0) * 0.45
    score -= max(0, 330.0 - ym["yaw_circular_span_deg"]) * 0.25
    score = max(0, min(100, score))
    return {
        "fit_center": fit_center.tolist(),
        "r_from_center_mean": float(np.mean(r_from_center)),
        "r_from_center_std": float(np.std(r_from_center)),
        "r_from_center_cv_pct": float(np.std(r_from_center) / np.mean(r_from_center) * 100),
        "r_from_origin_mean": float(np.mean(r_from_origin)),
        "r_from_origin_std": float(np.std(r_from_origin)),
        "residual_pct": residual, "max_min_ratio": ratio, "score": score, **ym,
    }


def find_best_2d_window(data, preserve_header, min_samples=2000, windows=None, stride_fraction=0.20):
    if windows is None:
        windows = [3000, 5000, 8000, 12000, 16000, 24000]
    candidates = []
    n = len(data)
    for size in windows:
        if size < min_samples or size > n:
            continue
        stride = max(200, int(size * stride_fraction))
        starts = list(range(0, n - size + 1, stride))
        if starts[-1] != n - size:
            starts.append(n - size)
        for start in starts:
            end = start + size
            window = data[start:end]
            try:
                bias, soft_iron, field_strength, radii = build_2d_calibration(window, preserve_header)
                corrected = apply_calibration(window, bias, soft_iron)
                report = quality_report_2d(window, corrected)
            except Exception:
                continue
            reject_reasons = []
            if report["yaw_bins_occupied"] < RAW_ACCEPT["min_bins"]:
                reject_reasons.append(f"bins {report['yaw_bins_occupied']} < {RAW_ACCEPT['min_bins']}")
            if report["yaw_largest_gap_deg"] > RAW_ACCEPT["max_gap_deg"]:
                reject_reasons.append(f"gap {report['yaw_largest_gap_deg']:.1f} > {RAW_ACCEPT['max_gap_deg']}")
            if report["yaw_circular_span_deg"] < RAW_ACCEPT["min_span_deg"]:
                reject_reasons.append(f"span {report['yaw_circular_span_deg']:.1f} < {RAW_ACCEPT['min_span_deg']}")
            if report["yaw_unwrapped_span_deg"] < RAW_ACCEPT["min_span_deg"]:
                reject_reasons.append(f"unwrapped {report['yaw_unwrapped_span_deg']:.1f} < {RAW_ACCEPT['min_span_deg']}")
            if report["residual_pct"] > RAW_ACCEPT["max_residual_pct"]:
                reject_reasons.append(f"residual {report['residual_pct']:.2f}% > {RAW_ACCEPT['max_residual_pct']}%")
            if report["max_min_ratio"] > RAW_ACCEPT["max_ratio"]:
                reject_reasons.append(f"ratio {report['max_min_ratio']:.3f} > {RAW_ACCEPT['max_ratio']}")
            for idx, val in enumerate([soft_iron[0,0], soft_iron[0,1], soft_iron[1,1]]):
                lo, hi = RAW_ACCEPT["soft_iron_range"]
                if not (lo <= abs(val) <= hi):
                    reject_reasons.append(f"soft_iron[{idx}]={val:.4f} out of [{lo},{hi}]")
            if reject_reasons:
                report["reject_reasons"] = reject_reasons
                candidates.append({"score": -1, "start": start, "end": end, "size": size,
                    "bias": bias, "soft_iron": soft_iron, "field_strength": field_strength,
                    "radii": radii, "report": report, "passed": False})
                continue
            score = report["score"]
            score += min(10.0, report["yaw_coverage_pct"] / 10.0)
            score += min(8.0, report["yaw_circular_span_deg"] / 45.0)
            score += min(4.0, report["yaw_bin_balance"] * 8.0)
            score -= max(0, report["max_min_ratio"] - 1.08) * 80.0
            score -= max(0, report["residual_pct"] - 1.8) * 8.0
            score += min(5.0, size / 5000.0)
            candidates.append({"score": score, "start": start, "end": end, "size": size,
                "bias": bias, "soft_iron": soft_iron, "field_strength": field_strength,
                "radii": radii, "report": report, "passed": True})
    if not candidates:
        raise ValueError("No usable 2D calibration window found.")
    candidates.sort(key=lambda c: c["score"], reverse=True)
    return candidates[0], candidates[:8]


def detect_plateaus(data, window=150, step=30, min_len=400, max_yaw_span=3.0, max_xy_std=40.0, merge_gap=120):
    wins = []
    for a in range(0, len(data) - window + 1, step):
        seg = data[a:a + window]
        xy = seg[:, :2]
        yy = np.degrees(np.unwrap(np.arctan2(xy[:, 1], xy[:, 0])))
        wins.append((a, a + window, yy.max() - yy.min(), xy[:, 0].std(), xy[:, 1].std()))
    plate, cur = [], None
    for a, b, yspan, xs, ys in wins:
        ok = (yspan < max_yaw_span and xs < max_xy_std and ys < max_xy_std)
        if ok and cur is None:
            cur = [a, b]
        elif ok:
            cur[1] = b
        elif cur is not None:
            if cur[1] - cur[0] >= min_len:
                plate.append(tuple(cur))
            cur = None
    if cur is not None and cur[1] - cur[0] >= min_len:
        plate.append(tuple(cur))
    merged = []
    for a, b in plate:
        if merged and a - merged[-1][1] <= merge_gap:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    return merged


def calibrated_verification_report(data, plateaus):
    xy = data[:, :2]
    try:
        design = np.column_stack([xy[:,0]**2, 2*xy[:,0]*xy[:,1], xy[:,1]**2, 2*xy[:,0], 2*xy[:,1]])
        coeff, *_ = np.linalg.lstsq(design, np.ones(len(xy)), rcond=None)
        q = np.array([[coeff[0], coeff[1]], [coeff[1], coeff[2]]], dtype=float)
        u = coeff[3:5]
        fit_center = -np.linalg.solve(q, u)
    except Exception:
        fit_center = np.array([xy[:,0].mean(), xy[:,1].mean()])
    stops = []
    for i, (a, b) in enumerate(plateaus, 1):
        seg = data[a:b]
        sxy = seg[:, :2]
        r_from_center = np.sqrt((sxy[:,0]-fit_center[0])**2 + (sxy[:,1]-fit_center[1])**2)
        r_from_origin = np.linalg.norm(sxy, axis=1)
        ang = math.degrees(math.atan2(sxy[:,1].mean()-fit_center[1], sxy[:,0].mean()-fit_center[0]))
        stops.append({"idx": i, "start": a, "end": b, "n": b - a, "yaw_deg": round(ang, 1),
            "r_center_mean": round(float(r_from_center.mean()), 1),
            "r_center_std": round(float(r_from_center.std()), 1),
            "r_origin_mean": round(float(r_from_origin.mean()), 1),
            "r_origin_std": round(float(r_from_origin.std()), 1),
            "z_mean": round(float(seg[:,2].mean()), 1), "z_std": round(float(seg[:,2].std()), 1),
            "x_mean": round(float(seg[:,0].mean()), 1), "y_mean": round(float(seg[:,1].mean()), 1)})
    angle_diffs = []
    for i in range(len(stops) - 1):
        diff = ((stops[i+1]["yaw_deg"] - stops[i]["yaw_deg"] + 540) % 360) - 180
        angle_diffs.append(round(diff, 1))
    z_means = [s["z_mean"] for s in stops]
    z_range = max(z_means) - min(z_means) if z_means else 0
    r_centers = [s["r_center_mean"] for s in stops]
    r_ratio = max(r_centers) / min(r_centers) if r_centers and min(r_centers) > 0 else 999
    r_cv = float(np.std(r_centers) / np.mean(r_centers) * 100) if r_centers else 999
    return {"fit_center": fit_center.tolist(), "n_plateaus": len(plateaus), "stops": stops,
        "angle_diffs": angle_diffs, "z_range": round(z_range, 1),
        "z_min": round(min(z_means), 1) if z_means else 0,
        "z_max": round(max(z_means), 1) if z_means else 0,
        "r_center_ratio": round(r_ratio, 3), "r_center_cv_pct": round(r_cv, 1),
        "center_offset": round(float(np.linalg.norm(fit_center)), 1),
        "z_3d_warning": z_range > CAL_ACCEPT["z_diff_warning"]}


def header_text(bias, soft_iron, field_strength, sample_count, report, mode):
    return f"""/* Magnetometer calibration params - generated by mag_calibrator_gui.py */
/* Mode: {mode.upper()}, Samples: {sample_count}, Quality: {report['score']:.1f}/100 */

#define MAG_HARD_IRON_X  {bias[0]:.6f}f
#define MAG_HARD_IRON_Y  {bias[1]:.6f}f
#define MAG_HARD_IRON_Z  {bias[2]:.6f}f

#define MAG_SOFT_IRON_XX {soft_iron[0, 0]:.6f}f
#define MAG_SOFT_IRON_XY {soft_iron[0, 1]:.6f}f
#define MAG_SOFT_IRON_XZ {soft_iron[0, 2]:.6f}f
#define MAG_SOFT_IRON_YX {soft_iron[1, 0]:.6f}f
#define MAG_SOFT_IRON_YY {soft_iron[1, 1]:.6f}f
#define MAG_SOFT_IRON_YZ {soft_iron[1, 2]:.6f}f
#define MAG_SOFT_IRON_ZX {soft_iron[2, 0]:.6f}f
#define MAG_SOFT_IRON_ZY {soft_iron[2, 1]:.6f}f
#define MAG_SOFT_IRON_ZZ {soft_iron[2, 2]:.6f}f

#define MAG_FIELD_STRENGTH {field_strength:.6f}f
"""


class SerialReaderThread(QThread):
    sample_received = Signal(float, float, float)
    connection_lost = Signal()
    error_message = Signal(str)

    def __init__(self, port, baud=115200, parent=None):
        super().__init__(parent)
        self.port, self.baud = port, baud
        self._running = False
        self._ser = None

    def run(self):
        self._running = True
        while self._running:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
                while self._running:
                    line = self._ser.readline().decode("ascii", errors="ignore").strip()
                    if not line:
                        continue
                    m = CSV_RE.match(line)
                    if not m:
                        continue
                    self.sample_received.emit(float(m.group(1)), float(m.group(2)), float(m.group(3)))
            except (serial.SerialException, AttributeError, TypeError, OSError) as e:
                if self._running:
                    self.error_message.emit(f"Serial error: {e}")
                    self.connection_lost.emit()
                    self.msleep(1000)
            finally:
                if self._ser and self._ser.is_open:
                    try: self._ser.close()
                    except: pass

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            try: self._ser.close()
            except: pass
        self.wait(3000)


class MagCalibratorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("\u78c1\u529b\u8ba1 2D \u79bb\u7ebf\u6821\u51c6\u5de5\u5177")
        self.setMinimumSize(1280, 800)
        self._mode = "raw"
        self._collecting = False
        self._reader = None
        self._all_data = []
        self._recent_data = deque(maxlen=15000)
        self._sample_count = 0
        self._start_time = 0
        self._current_bias = None
        self._current_soft_iron = None
        self._current_field = None
        self._current_report = None
        self._best_window = None
        self._plateaus = None
        self._verification_report = None
        self._build_ui()
        self._connect_signals()
        self._load_current_params()
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._update_plot)
        self._plot_timer.start(80)
        self._fit_timer = QTimer(self)
        self._fit_timer.timeout.connect(self._update_fit)
        self._fit_timer.start(1000)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        ml = QVBoxLayout(central)
        ml.setContentsMargins(6, 6, 6, 6)
        ml.addLayout(self._build_toolbar())
        splitter = QSplitter()
        ml.addWidget(splitter, stretch=1)
        splitter.addWidget(self._build_plots())
        splitter.addWidget(self._build_right_panel())
        splitter.setSizes([800, 480])
        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._statusbar.showMessage("\u5c31\u7eea")

    def _build_toolbar(self):
        lo = QHBoxLayout()
        lo.addWidget(QLabel("\u4e32\u53e3:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(120)
        self._refresh_ports()
        lo.addWidget(self._port_combo)
        self._refresh_btn = QPushButton("\u5237\u65b0")
        self._refresh_btn.setFixedWidth(60)
        lo.addWidget(self._refresh_btn)
        lo.addWidget(QLabel("\u6ce2\u7279\u7387:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["115200", "460800", "921600"])
        self._baud_combo.setCurrentText("115200")
        self._baud_combo.setFixedWidth(90)
        lo.addWidget(self._baud_combo)
        lo.addSpacing(20)
        lo.addWidget(QLabel("\u6a21\u5f0f:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["RAW \u539f\u59cb\u6570\u636e (\u6821\u51c6)", "CALIBRATED \u6821\u51c6\u540e\u6570\u636e (\u9a8c\u8bc1)"])
        self._mode_combo.setCurrentIndex(0)
        self._mode_combo.setFixedWidth(200)
        lo.addWidget(self._mode_combo)
        lo.addSpacing(20)
        self._open_btn = QPushButton("\u6253\u5f00\u4e32\u53e3")
        self._open_btn.setFixedWidth(90)
        lo.addWidget(self._open_btn)
        self._start_btn = QPushButton("\u5f00\u59cb\u91c7\u96c6")
        self._start_btn.setFixedWidth(80)
        self._start_btn.setEnabled(False)
        lo.addWidget(self._start_btn)
        self._stop_btn = QPushButton("\u505c\u6b62\u91c7\u96c6")
        self._stop_btn.setFixedWidth(80)
        self._stop_btn.setEnabled(False)
        lo.addWidget(self._stop_btn)
        lo.addStretch()
        self._load_btn = QPushButton("\u52a0\u8f7d\u6587\u4ef6")
        self._load_btn.setFixedWidth(80)
        lo.addWidget(self._load_btn)
        return lo

    def _build_plots(self):
        w = QWidget()
        lo = QVBoxLayout(w)
        lo.setContentsMargins(0, 0, 0, 0)
        self._xy_plot = PlotWidget()
        self._xy_plot.setAspectLocked(True)
        self._xy_plot.setLabel("bottom", "X \u78c1\u529b")
        self._xy_plot.setLabel("left", "Y \u78c1\u529b")
        self._xy_plot.setTitle("XY \u6563\u70b9\u56fe")
        self._xy_plot.addLegend(offset=(10, 10))
        self._scatter_item = self._xy_plot.plot(pen=None, symbol="o", symbolSize=2, symbolBrush=mkBrush(100, 100, 255, 80), name="\u6570\u636e\u70b9")
        self._current_point = self._xy_plot.plot(pen=None, symbol="o", symbolSize=10, symbolBrush=mkBrush(255, 50, 50, 255), name="\u5f53\u524d\u70b9")
        self._ellipse_curve = self._xy_plot.plot(pen=mkPen("r", width=2), name="\u62df\u5408\u692d\u5706")
        self._center_mark = self._xy_plot.plot(pen=None, symbol="+", symbolSize=15, symbolBrush=mkBrush(0, 200, 0, 255), name="\u5706\u5fc3")
        self._plateau_scatter = self._xy_plot.plot(pen=None, symbol="o", symbolSize=4, symbolBrush=mkBrush(255, 165, 0, 150), name="\u9759\u6b62\u6bb5")
        lo.addWidget(self._xy_plot, stretch=3)
        self._coverage_label = QLabel("\u8986\u76d6\u7387: --")
        self._coverage_label.setStyleSheet("font-size: 12px; padding: 4px;")
        lo.addWidget(self._coverage_label)
        return w

    def _build_right_panel(self):
        tabs = QTabWidget()
        tabs.addTab(self._build_status_tab(), "\u72b6\u6001")
        tabs.addTab(self._build_quality_tab(), "\u8d28\u91cf")
        tabs.addTab(self._build_analysis_tab(), "\u5206\u6790")
        tabs.addTab(self._build_params_tab(), "\u53c2\u6570")
        return tabs

    def _build_status_tab(self):
        w = QWidget()
        lo = QVBoxLayout(w)
        pg = QGroupBox("\u5f53\u524d\u56fa\u4ef6\u53c2\u6570 (code/ins/calibration_params.h)")
        pgl = QGridLayout(pg)
        self._param_labels = {}
        row = 0
        for name in ["HARD_IRON_X", "HARD_IRON_Y", "HARD_IRON_Z", "SOFT_IRON_XX", "SOFT_IRON_XY", "SOFT_IRON_YX", "SOFT_IRON_YY", "SOFT_IRON_ZZ", "FIELD_STRENGTH"]:
            pgl.addWidget(QLabel(name + ":"), row, 0)
            lbl = QLabel("--")
            lbl.setStyleSheet("font-family: monospace; font-size: 11px;")
            pgl.addWidget(lbl, row, 1)
            self._param_labels[name] = lbl
            row += 1
        lo.addWidget(pg)
        lg = QGroupBox("\u5b9e\u65f6\u6570\u636e")
        lgl = QGridLayout(lg)
        self._live_labels = {}
        for row, name in enumerate(["\u91c7\u6837\u6570", "\u9891\u7387(Hz)", "X", "Y", "Z", "\u822a\u5411(\u00b0)", "\u534a\u5f84"]):
            lgl.addWidget(QLabel(name + ":"), row, 0)
            lbl = QLabel("--")
            lbl.setStyleSheet("font-family: monospace; font-size: 12px;")
            lgl.addWidget(lbl, row, 1)
            self._live_labels[name] = lbl
        lo.addWidget(lg)
        self._mode_warning_label = QLabel("")
        self._mode_warning_label.setStyleSheet("color: red; font-weight: bold; padding: 4px;")
        self._mode_warning_label.setWordWrap(True)
        lo.addWidget(self._mode_warning_label)
        lo.addStretch()
        return w

    def _build_quality_tab(self):
        w = QWidget()
        lo = QVBoxLayout(w)
        self._quality_text = QTextEdit()
        self._quality_text.setReadOnly(True)
        self._quality_text.setStyleSheet("font-family: monospace; font-size: 11px;")
        lo.addWidget(self._quality_text)
        return w

    def _build_analysis_tab(self):
        w = QWidget()
        lo = QVBoxLayout(w)
        blo = QHBoxLayout()
        self._auto_window_btn = QPushButton("\u81ea\u52a8\u9009\u7a97 (RAW)")
        self._auto_window_btn.setEnabled(False)
        blo.addWidget(self._auto_window_btn)
        self._fit_btn = QPushButton("2D\u62df\u5408 (RAW)")
        self._fit_btn.setEnabled(False)
        blo.addWidget(self._fit_btn)
        self._detect_plateaus_btn = QPushButton("\u68c0\u6d4b\u9759\u6b62\u6bb5 (CAL)")
        self._detect_plateaus_btn.setEnabled(False)
        blo.addWidget(self._detect_plateaus_btn)
        self._verify_btn = QPushButton("\u9a8c\u8bc1\u62a5\u544a (CAL)")
        self._verify_btn.setEnabled(False)
        blo.addWidget(self._verify_btn)
        lo.addLayout(blo)
        self._analysis_text = QTextEdit()
        self._analysis_text.setReadOnly(True)
        self._analysis_text.setStyleSheet("font-family: monospace; font-size: 11px;")
        lo.addWidget(self._analysis_text, stretch=1)
        return w

    def _build_params_tab(self):
        w = QWidget()
        lo = QVBoxLayout(w)
        pvg = QGroupBox("\u5934\u6587\u4ef6\u9884\u89c8")
        pvl = QVBoxLayout(pvg)
        self._header_preview = QPlainTextEdit()
        self._header_preview.setReadOnly(True)
        self._header_preview.setStyleSheet("font-family: monospace; font-size: 11px;")
        self._header_preview.setMaximumHeight(250)
        pvl.addWidget(self._header_preview)
        lo.addWidget(pvg)
        dfg = QGroupBox("\u53c2\u6570\u5bf9\u6bd4 (\u65e7 -> \u65b0)")
        dfl = QVBoxLayout(dfg)
        self._diff_text = QTextEdit()
        self._diff_text.setReadOnly(True)
        self._diff_text.setStyleSheet("font-family: monospace; font-size: 11px;")
        self._diff_text.setMaximumHeight(200)
        dfl.addWidget(self._diff_text)
        lo.addWidget(dfg)
        blo = QHBoxLayout()
        self._write_btn = QPushButton("\u5199\u5165 code/ins/calibration_params.h")
        self._write_btn.setEnabled(False)
        self._write_btn.setStyleSheet("background-color: #ff9800; font-weight: bold; padding: 8px;")
        blo.addWidget(self._write_btn)
        self._save_data_btn = QPushButton("\u4fdd\u5b58\u6570\u636e")
        self._save_data_btn.setEnabled(False)
        blo.addWidget(self._save_data_btn)
        self._save_screenshot_btn = QPushButton("\u4fdd\u5b58\u622a\u56fe")
        blo.addWidget(self._save_screenshot_btn)
        lo.addLayout(blo)
        lo.addStretch()
        return w

    def _connect_signals(self):
        self._refresh_btn.clicked.connect(self._refresh_ports)
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        self._open_btn.clicked.connect(self._toggle_serial)
        self._start_btn.clicked.connect(self._start_collecting)
        self._stop_btn.clicked.connect(self._stop_collecting)
        self._load_btn.clicked.connect(self._load_file)
        self._auto_window_btn.clicked.connect(self._do_auto_window)
        self._fit_btn.clicked.connect(self._do_fit)
        self._detect_plateaus_btn.clicked.connect(self._do_detect_plateaus)
        self._verify_btn.clicked.connect(self._do_verify)
        self._write_btn.clicked.connect(self._do_write_params)
        self._save_data_btn.clicked.connect(self._do_save_data)
        self._save_screenshot_btn.clicked.connect(self._do_save_screenshot)

    def _refresh_ports(self):
        self._port_combo.clear()
        for p in sorted(serial.tools.list_ports.comports(), key=lambda x: x.device):
            desc = f"{p.device} - {p.description}"
            if p.vid: desc += f" [VID={p.vid:04X} PID={p.pid:04X}]"
            self._port_combo.addItem(desc, p.device)
        if self._port_combo.count() == 0:
            self._port_combo.addItem("\u672a\u627e\u5230\u4e32\u53e3", "")

    def _get_selected_port(self):
        return self._port_combo.currentData() or ""

    def _on_mode_changed(self, idx):
        self._mode = "raw" if idx == 0 else "calibrated"
        has = len(self._all_data) > 100
        self._auto_window_btn.setEnabled(self._mode == "raw" and has)
        self._fit_btn.setEnabled(self._mode == "raw" and has)
        self._detect_plateaus_btn.setEnabled(self._mode == "calibrated" and has)
        self._verify_btn.setEnabled(self._mode == "calibrated" and has)

    def _toggle_serial(self):
        if self._reader and self._reader.isRunning():
            self._reader.stop()
            self._reader = None
            self._open_btn.setText("\u6253\u5f00\u4e32\u53e3")
            self._start_btn.setEnabled(False)
            self._statusbar.showMessage("\u4e32\u53e3\u5df2\u5173\u95ed")
        else:
            port = self._get_selected_port()
            if not port:
                QMessageBox.warning(self, "\u9519\u8bef", "\u8bf7\u5148\u9009\u62e9\u4e32\u53e3")
                return
            baud = int(self._baud_combo.currentText())
            self._reader = SerialReaderThread(port, baud)
            self._reader.sample_received.connect(self._on_sample)
            self._reader.connection_lost.connect(self._on_connection_lost)
            self._reader.error_message.connect(lambda msg: self._statusbar.showMessage(msg))
            self._reader.start()
            self._open_btn.setText("\u5173\u95ed\u4e32\u53e3")
            self._start_btn.setEnabled(True)
            self._statusbar.showMessage(f"\u4e32\u53e3\u5df2\u6253\u5f00: {port} @ {baud}")

    def _on_sample(self, x, y, z):
        if not self._collecting: return
        self._all_data.append([x, y, z])
        self._recent_data.append([x, y, z])
        self._sample_count += 1

    def _on_connection_lost(self):
        self._statusbar.showMessage("\u8fde\u63a5\u65ad\u5f00\uff0c\u6b63\u5728\u91cd\u8fde...")

    def _start_collecting(self):
        self._all_data.clear()
        self._recent_data.clear()
        self._sample_count = 0
        self._start_time = time.time()
        self._collecting = True
        self._current_bias = self._current_soft_iron = self._current_field = None
        self._current_report = self._best_window = self._plateaus = self._verification_report = None
        self._mode_warning_label.setText("")
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._mode_combo.setEnabled(False)
        self._save_data_btn.setEnabled(False)
        info = MODE_INFO[self._mode]
        self._statusbar.showMessage(f"\u91c7\u96c6\u4e2d... \u6a21\u5f0f={self._mode.upper()} \u9700\u8981C\u8c03\u7528: {info['c_function']}")

    def _stop_collecting(self):
        self._collecting = False
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._mode_combo.setEnabled(True)
        self._save_data_btn.setEnabled(len(self._all_data) > 0)
        elapsed = time.time() - self._start_time
        self._statusbar.showMessage(f"\u91c7\u96c6\u5b8c\u6210: {self._sample_count} \u4e2a\u6837\u672c, \u7528\u65f6 {elapsed:.1f}s")
        has = len(self._all_data) > 100
        self._auto_window_btn.setEnabled(self._mode == "raw" and has)
        self._fit_btn.setEnabled(self._mode == "raw" and has)
        self._detect_plateaus_btn.setEnabled(self._mode == "calibrated" and has)
        self._verify_btn.setEnabled(self._mode == "calibrated" and has)
        if len(self._all_data) > 0:
            self._do_save_data()

    def _load_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "\u52a0\u8f7d\u78c1\u529b\u8ba1\u6570\u636e", str(DATA_DIR), "\u6587\u672c\u6587\u4ef6 (*.txt);;\u6240\u6709\u6587\u4ef6 (*)")
        if not path: return
        try:
            data = load_mag_data(path)
            data = filter_valid(data)
            self._all_data = data.tolist()
            self._recent_data = deque(self._all_data[-15000:], maxlen=15000)
            self._sample_count = len(self._all_data)
            self._start_time = time.time()
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                hdr = []
                for line in f:
                    if line.startswith("#"): hdr.append(line.strip())
                    else: break
            ht = "\n".join(hdr)
            if "calibrated" in ht.lower():
                self._mode = "calibrated"; self._mode_combo.setCurrentIndex(1)
            else:
                self._mode = "raw"; self._mode_combo.setCurrentIndex(0)
            has = len(self._all_data) > 100
            self._auto_window_btn.setEnabled(self._mode == "raw" and has)
            self._fit_btn.setEnabled(self._mode == "raw" and has)
            self._detect_plateaus_btn.setEnabled(self._mode == "calibrated" and has)
            self._verify_btn.setEnabled(self._mode == "calibrated" and has)
            self._save_data_btn.setEnabled(True)
            self._statusbar.showMessage(f"\u5df2\u52a0\u8f7d: {path} ({self._sample_count} \u4e2a\u6837\u672c, \u6a21\u5f0f={self._mode})")
        except Exception as e:
            QMessageBox.critical(self, "\u52a0\u8f7d\u5931\u8d25", str(e))

    def _update_plot(self):
        if not self._recent_data: return
        data = np.array(list(self._recent_data), dtype=float)
        if len(data) == 0: return
        if len(data) > 10000:
            idx = np.linspace(0, len(data) - 1, 10000, dtype=int)
            display = data[idx]
        else:
            display = data
        self._scatter_item.setData(display[:, 0], display[:, 1])
        if len(data) > 0:
            self._current_point.setData([data[-1, 0]], [data[-1, 1]])
        elapsed = time.time() - self._start_time if self._start_time else 1
        freq = self._sample_count / elapsed if elapsed > 0 else 0
        self._live_labels["\u91c7\u6837\u6570"].setText(str(self._sample_count))
        self._live_labels["\u9891\u7387(Hz)"].setText(f"{freq:.1f}")
        if len(data) > 0:
            last = data[-1]
            self._live_labels["X"].setText(f"{last[0]:.1f}")
            self._live_labels["Y"].setText(f"{last[1]:.1f}")
            self._live_labels["Z"].setText(f"{last[2]:.1f}")
            yaw = np.degrees(np.arctan2(last[1], last[0]))
            r = np.sqrt(last[0]**2 + last[1]**2)
            self._live_labels["\u822a\u5411(\u00b0)"].setText(f"{yaw:.1f}")
            self._live_labels["\u534a\u5f84"].setText(f"{r:.1f}")

    def _update_fit(self):
        if len(self._all_data) < 200: return
        data = np.array(self._all_data, dtype=float)
        data = filter_valid(data)
        if len(data) < 200: return
        try:
            if self._mode == "raw": self._update_raw_fit(data)
            else: self._update_calibrated_fit(data)
        except Exception: pass

    def _draw_ellipse(self, xy):
        from numpy.linalg import eigh
        design = np.column_stack([xy[:,0]**2, 2*xy[:,0]*xy[:,1], xy[:,1]**2, 2*xy[:,0], 2*xy[:,1]])
        coeff, *_ = np.linalg.lstsq(design, np.ones(len(xy)), rcond=None)
        q = np.array([[coeff[0], coeff[1]], [coeff[1], coeff[2]]], dtype=float)
        u = coeff[3:5]
        c = -np.linalg.solve(q, u)
        rhs = 1.0 - c @ u
        if rhs <= 0: return None
        q_norm = 0.5 * (q / rhs + (q / rhs).T)
        evals, evecs = eigh(q_norm)
        if np.any(evals <= 0): return None
        r = 1.0 / np.sqrt(evals)
        theta = np.linspace(0, 2 * np.pi, 200)
        angle = np.arctan2(evecs[1, 0], evecs[0, 0])
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        ex, ey = r[0] * np.cos(theta), r[1] * np.sin(theta)
        rx = cos_a * ex - sin_a * ey + c[0]
        ry = sin_a * ex + cos_a * ey + c[1]
        self._ellipse_curve.setData(rx, ry)
        self._center_mark.setData([c[0]], [c[1]])
        return c, r

    def _update_raw_fit(self, data):
        xy = data[:, :2]
        try: center, soft_xy, field, radii = fit_ellipse_2d(data)
        except: return
        result = self._draw_ellipse(xy)
        if result is None: return
        c, r = result
        ym = yaw_coverage_metrics(xy - c)
        self._coverage_label.setText(f"\u8986\u76d6\u7387: {ym['yaw_bins_occupied']}/{ym['yaw_bin_count']} \u533a\u95f4 ({ym['yaw_coverage_pct']:.0f}%)  \u6700\u5927\u95f4\u9699: {ym['yaw_largest_gap_deg']:.1f}\u00b0  \u8de8\u5ea6: {ym['yaw_circular_span_deg']:.1f}\u00b0")
        self._quality_text.setText(f"RAW \u6570\u636e\u8d28\u91cf\n{'='*40}\n\u91c7\u6837\u6570: {len(data)}\nX \u8303\u56f4: {data[:,0].min():.1f} ~ {data[:,0].max():.1f}\nY \u8303\u56f4: {data[:,1].min():.1f} ~ {data[:,1].max():.1f}\nZ \u8303\u56f4: {data[:,2].min():.1f} ~ {data[:,2].max():.1f}\n\n\u692d\u5706\u62df\u5408\n\u5706\u5fc3: ({c[0]:.1f}, {c[1]:.1f})\n\u534a\u957f\u8f74: {max(r):.1f}  \u534a\u77ed\u8f74: {min(r):.1f}\n\u8f74\u6bd4: {max(r)/min(r):.4f}\n\u573a\u5f3a: {field:.1f}\n\n\u822a\u5411\u8986\u76d6\n\u533a\u95f4\u6570: {ym['yaw_bins_occupied']}/{ym['yaw_bin_count']}\n\u8986\u76d6\u7387: {ym['yaw_coverage_pct']:.1f}%\n\u6700\u5927\u95f4\u9699: {ym['yaw_largest_gap_deg']:.1f}\u00b0\n\u5706\u5468\u8de8\u5ea6: {ym['yaw_circular_span_deg']:.1f}\u00b0\n\u5c55\u5f00\u8de8\u5ea6: {ym['yaw_unwrapped_span_deg']:.1f}\u00b0\n\u533a\u95f4\u5e73\u8861: {ym['yaw_bin_balance']:.3f}")

    def _update_calibrated_fit(self, data):
        xy = data[:, :2]
        try: center, soft_xy, field, radii = fit_ellipse_2d(data)
        except: return
        result = self._draw_ellipse(xy)
        if result is None: return
        c, r = result
        r_from_center = np.sqrt((xy[:,0]-c[0])**2 + (xy[:,1]-c[1])**2)
        r_from_origin = np.linalg.norm(xy, axis=1)
        ym = yaw_coverage_metrics(xy - c)
        self._coverage_label.setText(f"\u8986\u76d6\u7387: {ym['yaw_bins_occupied']}/{ym['yaw_bin_count']} \u533a\u95f4 ({ym['yaw_coverage_pct']:.0f}%)  \u5706\u5fc3\u504f\u79fb: {np.linalg.norm(c):.1f}  \u534a\u5f84\u6bd4: {r_from_center.max()/r_from_center.min():.3f}")
        self._quality_text.setText(f"CALIBRATED \u6570\u636e\u8d28\u91cf\n{'='*40}\n\u91c7\u6837\u6570: {len(data)}\n\n\u62df\u5408\u5706\u5fc3: ({c[0]:.2f}, {c[1]:.2f})\n\u5706\u5fc3\u504f\u79fb: {np.linalg.norm(c):.1f}\n\n\u5230\u539f\u70b9\u534a\u5f84: {r_from_origin.mean():.1f} +/- {r_from_origin.std():.1f}\n\u5230\u5706\u5fc3\u534a\u5f84: {r_from_center.mean():.1f} +/- {r_from_center.std():.1f}\n\u534a\u5f84 CV: {r_from_center.std()/r_from_center.mean()*100:.2f}%\n\u534a\u5f84 \u6700\u5927/\u6700\u5c0f: {r_from_center.max()/r_from_center.min():.3f}\n\n\u6b8b\u5dee\u692d\u5706\u8f74\u6bd4: {max(r)/min(r):.4f}\n\u534a\u957f\u8f74: {max(r):.1f}  \u534a\u77ed\u8f74: {min(r):.1f}")

    def _do_auto_window(self):
        if self._mode != "raw":
            QMessageBox.warning(self, "\u9519\u8bef", "\u81ea\u52a8\u9009\u7a97\u4ec5\u9002\u7528\u4e8e RAW \u6a21\u5f0f\u6570\u636e"); return
        data = np.array(self._all_data, dtype=float)
        data = filter_valid(data)
        if len(data) < 100:
            QMessageBox.warning(self, "\u6570\u636e\u4e0d\u8db3", "\u6709\u6548\u6837\u672c\u4e0d\u8db3 100 \u4e2a"); return
        self._statusbar.showMessage("\u6b63\u5728\u641c\u7d22\u6700\u4f18\u7a97\u53e3...")
        QApplication.processEvents()
        try:
            best, top = find_best_2d_window(data, str(DEFAULT_HEADER))
            self._best_window = best
            text = "\u81ea\u52a8\u9009\u7a97\u7ed3\u679c\n" + "=" * 50 + "\n\n"
            for idx, item in enumerate(top, 1):
                rep = item["report"]
                status = "\u901a\u8fc7" if item.get("passed", True) else "\u5931\u8d25"
                text += f"\u7a97\u53e3 {idx}: samples[{item['start']}:{item['end']}] n={item['size']} {status}\n  \u8bc4\u5206: {rep['score']:.1f}  \u6b8b\u5dee: {rep['residual_pct']:.2f}%  \u6bd4\u503c: {rep['max_min_ratio']:.3f}\n  \u533a\u95f4: {rep['yaw_bins_occupied']}/{rep['yaw_bin_count']}  \u95f4\u9699: {rep['yaw_largest_gap_deg']:.1f}\u00b0  \u8de8\u5ea6: {rep['yaw_circular_span_deg']:.1f}\u00b0\n"
                if not item.get("passed", True):
                    text += f"  \u62d2\u7edd: {', '.join(rep.get('reject_reasons', []))}\n"
                text += "\n"
            if best.get("passed", True):
                text += f"\u6700\u4f18\u7a97\u53e3: samples[{best['start']}:{best['end']}]\n  \u91c7\u6837\u6570: {best['size']}  \u8bc4\u5206: {best['report']['score']:.1f}\n  \u6b8b\u5dee: {best['report']['residual_pct']:.2f}%  \u6bd4\u503c: {best['report']['max_min_ratio']:.3f}\n  \u533a\u95f4: {best['report']['yaw_bins_occupied']}/{best['report']['yaw_bin_count']}  \u95f4\u9699: {best['report']['yaw_largest_gap_deg']:.1f}\u00b0  \u8de8\u5ea6: {best['report']['yaw_circular_span_deg']:.1f}\u00b0\n\n\u53ef\u4ee5\u5199\u5165\u53c2\u6570"
            else:
                text += "\u8b66\u544a: \u6ca1\u6709\u7a97\u53e3\u901a\u8fc7\u9a8c\u6536\u6807\u51c6\n\u5efa\u8bae: \u91cd\u65b0\u91c7\u96c6\uff0c\u8fde\u7eed\u5e73\u6ed1\u65cb\u8f6c\uff0c\u8986\u76d6\u5b8c\u6574 360\u00b0"
            self._analysis_text.setText(text)
            self._statusbar.showMessage("\u7a97\u53e3\u641c\u7d22\u5b8c\u6210")
        except ValueError as e:
            self._analysis_text.setText(f"\u7a97\u53e3\u641c\u7d22\u5931\u8d25: {e}")
            self._statusbar.showMessage("\u7a97\u53e3\u641c\u7d22\u5931\u8d25")

    def _do_fit(self):
        if self._mode != "raw":
            QMessageBox.warning(self, "\u9519\u8bef", "2D\u62df\u5408\u4ec5\u9002\u7528\u4e8e RAW \u6a21\u5f0f\u6570\u636e"); return
        data = np.array(self._all_data, dtype=float)
        data = filter_valid(data)
        if self._best_window and self._best_window.get("passed", True):
            wd = data[self._best_window["start"]:self._best_window["end"]]
            bias, soft_iron, field, radii = self._best_window["bias"], self._best_window["soft_iron"], self._best_window["field_strength"], self._best_window["radii"]
            source = f"\u6700\u4f18\u7a97\u53e3 [{self._best_window['start']}:{self._best_window['end']}]"
        else:
            try:
                bias, soft_iron, field, radii = build_2d_calibration(data, str(DEFAULT_HEADER))
                wd = data; source = "\u5168\u90e8\u6570\u636e"
            except Exception as e:
                QMessageBox.critical(self, "\u62df\u5408\u5931\u8d25", str(e)); return
        corrected = apply_calibration(wd, bias, soft_iron)
        report = quality_report_2d(wd, corrected)
        self._current_bias, self._current_soft_iron, self._current_field, self._current_report = bias, soft_iron, field, report
        self._header_preview.setPlainText(header_text(bias, soft_iron, field, len(wd), report, "2d"))
        self._update_diff(bias, soft_iron, field)
        self._write_btn.setEnabled(True)
        text = f"2D \u62df\u5408\u7ed3\u679c ({source})\n{'='*50}\n\n\u91c7\u6837\u6570: {len(wd)}\n\u786c\u94c1: X={bias[0]:.3f}, Y={bias[1]:.3f}, Z={bias[2]:.3f} (Z\u4fdd\u6301\u4e0d\u53d8)\n\u8f6f\u94c1 XY:\n  [{soft_iron[0,0]:.6f}, {soft_iron[0,1]:.6f}]\n  [{soft_iron[1,0]:.6f}, {soft_iron[1,1]:.6f}]\n\u573a\u5f3a: {field:.3f}\nXY\u692d\u5706\u534a\u5f84: {radii[0]:.3f}, {radii[1]:.3f}\n\u8f74\u6bd4: {max(radii)/min(radii):.4f}\n\n\u6821\u51c6\u540e\u8d28\u91cf\n  \u6b8b\u5dee: {report['residual_pct']:.3f}%\n  \u534a\u5f84\u6bd4: {report['max_min_ratio']:.4f}\n  \u62df\u5408\u5706\u5fc3: ({report['fit_center'][0]:.2f}, {report['fit_center'][1]:.2f})\n  \u5230\u5706\u5fc3\u534a\u5f84: {report['r_from_center_mean']:.1f} +/- {report['r_from_center_std']:.1f}\n  CV: {report['r_from_center_cv_pct']:.2f}%\n  \u8bc4\u5206: {report['score']:.1f}/100"
        self._analysis_text.setText(text)
        self._statusbar.showMessage("2D\u62df\u5408\u5b8c\u6210")

    def _do_detect_plateaus(self):
        if self._mode != "calibrated":
            QMessageBox.warning(self, "\u9519\u8bef", "\u9759\u6b62\u6bb5\u68c0\u6d4b\u4ec5\u9002\u7528\u4e8e CALIBRATED \u6a21\u5f0f\u6570\u636e"); return
        data = np.array(self._all_data, dtype=float)
        data = filter_valid(data)
        plateaus = detect_plateaus(data)
        self._plateaus = plateaus
        if plateaus:
            pts = np.vstack([data[a:b, :2] for a, b in plateaus])
            self._plateau_scatter.setData(pts[:, 0], pts[:, 1])
        text = f"\u9759\u6b62\u6bb5\u68c0\u6d4b\n{'='*50}\n\n\u68c0\u6d4b\u5230 {len(plateaus)} \u4e2a\u9759\u6b62\u6bb5:\n\n"
        xy = data[:, :2]
        try:
            design = np.column_stack([xy[:,0]**2, 2*xy[:,0]*xy[:,1], xy[:,1]**2, 2*xy[:,0], 2*xy[:,1]])
            coeff, *_ = np.linalg.lstsq(design, np.ones(len(xy)), rcond=None)
            q = np.array([[coeff[0], coeff[1]], [coeff[1], coeff[2]]], dtype=float)
            u = coeff[3:5]; fc = -np.linalg.solve(q, u)
        except: fc = np.array([xy[:,0].mean(), xy[:,1].mean()])
        for i, (a, b) in enumerate(plateaus, 1):
            seg = data[a:b]; sxy = seg[:, :2]
            r_c = np.sqrt((sxy[:,0]-fc[0])**2 + (sxy[:,1]-fc[1])**2)
            ang = math.degrees(math.atan2(sxy[:,1].mean()-fc[1], sxy[:,0].mean()-fc[0]))
            text += f"  {i}: [{a}:{b}] n={b-a} \u822a\u5411={ang:.1f}\u00b0 r={r_c.mean():.1f}+/-{r_c.std():.1f} Z={seg[:,2].mean():.1f}+/-{seg[:,2].std():.1f}\n"
        self._analysis_text.setText(text)
        self._statusbar.showMessage(f"\u68c0\u6d4b\u5230 {len(plateaus)} \u4e2a\u9759\u6b62\u6bb5")

    def _do_verify(self):
        if self._mode != "calibrated":
            QMessageBox.warning(self, "\u9519\u8bef", "\u9a8c\u8bc1\u4ec5\u9002\u7528\u4e8e CALIBRATED \u6a21\u5f0f\u6570\u636e"); return
        data = np.array(self._all_data, dtype=float)
        data = filter_valid(data)
        if self._plateaus is None: self._plateaus = detect_plateaus(data)
        report = calibrated_verification_report(data, self._plateaus)
        self._verification_report = report
        text = f"CALIBRATED \u9a8c\u8bc1\u62a5\u544a\n{'='*50}\n\n\u62df\u5408\u5706\u5fc3: ({report['fit_center'][0]:.2f}, {report['fit_center'][1]:.2f})\n\u5706\u5fc3\u504f\u79fb: {report['center_offset']:.1f}\n\u9759\u6b62\u6bb5\u6570: {report['n_plateaus']}\n\n\u9759\u6b62\u6bb5\u8be6\u60c5:\n{'\u6bb5':>3s}  {'\u822a\u5411':>7s}  {'\u5706\u5fc3\u534a\u5f84':>10s}  {'\u539f\u70b9\u534a\u5f84':>10s}  {'Z\u5747\u503c':>8s}  {'Z\u6807\u51c6\u5dee':>6s}\n{'-'*55}\n"
        for s in report["stops"]:
            text += f"{s['idx']:3d}  {s['yaw_deg']:7.1f}\u00b0  {s['r_center_mean']:7.1f}+/-{s['r_center_std']:.1f}  {s['r_origin_mean']:7.1f}+/-{s['r_origin_std']:.1f}  {s['z_mean']:8.1f}  {s['z_std']:6.1f}\n"
        text += f"\n\u76f8\u90bb\u89d2\u5ea6\u5dee: {report['angle_diffs']}\n\n\u534a\u5f84\u7edf\u8ba1 (\u5230\u62df\u5408\u5706\u5fc3):\n  \u6700\u5927/\u6700\u5c0f = {report['r_center_ratio']:.3f}\n  CV = {report['r_center_cv_pct']:.1f}%\n"
        ratio, cv = report["r_center_ratio"], report["r_center_cv_pct"]
        if ratio <= CAL_ACCEPT["max_radius_ratio_good"] and cv <= CAL_ACCEPT["max_cv_good_pct"]: rating = "*** \u4f18\u79c0"
        elif ratio <= CAL_ACCEPT["max_radius_ratio_ok"] and cv <= CAL_ACCEPT["max_cv_ok_pct"]: rating = "** \u5408\u683c"
        else: rating = "* \u9700\u6539\u8fdb"
        text += f"  \u8bc4\u7ea7: {rating}\n"
        if report["z_3d_warning"]:
            text += f"\n\u8b66\u544a: Z\u8f74 3D \u6548\u5e94:\n  Z \u5747\u503c\u8303\u56f4: {report['z_min']:.1f} ~ {report['z_max']:.1f} (\u5dee\u503c {report['z_range']:.1f})\n  2D\u6821\u51c6\u53d7 3D \u6548\u5e94\u9650\u5236\n  \u68c0\u67e5: \u78c1\u529b\u8ba1\u5b89\u88c5\u4f4d\u7f6e\u3001\u9644\u8fd1\u94c1\u78c1\u4f53\u3001\u7535\u673a\u8d70\u7ebf\u3001\u8f66\u8f86\u503e\u659c\n"
        if report["center_offset"] > CAL_ACCEPT["max_center_offset"]:
            text += f"\n\u8b66\u544a: \u62df\u5408\u5706\u5fc3\u504f\u79fb {report['center_offset']:.1f} > {CAL_ACCEPT['max_center_offset']}\n  hard_iron \u53ef\u80fd\u9700\u8981\u4fee\u6b63\n"
        self._analysis_text.setText(text)
        self._statusbar.showMessage("\u9a8c\u8bc1\u62a5\u544a\u5df2\u751f\u6210")

    def _update_diff(self, new_bias, new_soft, new_field):
        try: old_bias, old_soft, old_field = load_header(str(DEFAULT_HEADER))
        except: self._diff_text.setText("\u65e0\u6cd5\u8bfb\u53d6\u5f53\u524d\u53c2\u6570\u6587\u4ef6"); return
        text = "\u53c2\u6570\u5bf9\u6bd4\n" + "=" * 50 + "\n\n"
        params = [("HARD_IRON_X", old_bias[0], new_bias[0]), ("HARD_IRON_Y", old_bias[1], new_bias[1]),
                  ("SOFT_IRON_XX", old_soft[0,0], new_soft[0,0]), ("SOFT_IRON_XY", old_soft[0,1], new_soft[0,1]),
                  ("SOFT_IRON_YX", old_soft[1,0], new_soft[1,0]), ("SOFT_IRON_YY", old_soft[1,1], new_soft[1,1]),
                  ("SOFT_IRON_ZZ", old_soft[2,2], new_soft[2,2]), ("FIELD_STRENGTH", old_field, new_field)]
        text += f"{'\u53c2\u6570':<20s} {'\u65e7\u503c':>12s} {'\u65b0\u503c':>12s} {'\u5dee\u503c':>12s}\n{'-'*58}\n"
        for name, old, new in params:
            text += f"{name:<20s} {old:>12.6f} {new:>12.6f} {new-old:>+12.6f}\n"
        self._diff_text.setText(text)

    def _do_write_params(self):
        if self._current_bias is None:
            QMessageBox.warning(self, "\u9519\u8bef", "\u8bf7\u5148\u6267\u884c 2D \u62df\u5408"); return
        if self._mode != "raw":
            QMessageBox.warning(self, "\u5199\u5165\u88ab\u62d2\u7edd", "\u53c2\u6570\u5199\u5165\u4ec5\u5141\u8bb8\u4ece RAW \u6a21\u5f0f\u6570\u636e!\nCALIBRATED \u6570\u636e\u4ec5\u7528\u4e8e\u9a8c\u8bc1\u3002"); return
        header_path = str(DEFAULT_HEADER)
        reply = QMessageBox.question(self, "\u786e\u8ba4\u5199\u5165",
            f"\u5373\u5c06\u5199\u5165\u53c2\u6570\u5230:\n{header_path}\n\n\u8bf7\u786e\u8ba4:\n1. \u6570\u636e\u6765\u6e90\u4e3a RAW \u6a21\u5f0f\n2. \u5df2\u5ba1\u67e5\u53c2\u6570\u5bf9\u6bd4\n3. \u8d28\u91cf\u6307\u6807\u901a\u8fc7\u9a8c\u6536\n\n\u7ee7\u7eed?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes: return
        try:
            h_text = header_text(self._current_bias, self._current_soft_iron, self._current_field, len(self._all_data), self._current_report, "2d")
            Path(header_path).write_text(h_text, encoding="utf-8")
            bias_v, soft_v, field_v = load_header(header_path)
            if not np.allclose(bias_v, self._current_bias, atol=1e-4):
                raise ValueError("Verification failed: bias mismatch")
            QMessageBox.information(self, "\u5199\u5165\u6210\u529f",
                f"\u53c2\u6570\u5df2\u5199\u5165: {header_path}\n\n\u540e\u7eed\u6b65\u9aa4:\n1. \u5c06C\u4ee3\u7801\u5207\u6362\u4e3a imu_mag_send_calibrated_data_to_pc()\n2. \u91cd\u65b0\u7f16\u8bd1\u5e76\u70e7\u5f55\u56fa\u4ef6\n3. \u91c7\u96c6 CALIBRATED \u6a21\u5f0f\u6570\u636e\u8fdb\u884c\u9a8c\u8bc1\n4. \u8fd0\u884c\u9a8c\u8bc1\u62a5\u544a\u786e\u8ba4")
            self._statusbar.showMessage("\u53c2\u6570\u5199\u5165\u6210\u529f")
            self._load_current_params()
        except Exception as e:
            QMessageBox.critical(self, "\u5199\u5165\u5931\u8d25", str(e))

    def _do_save_data(self):
        if not self._all_data: return
        info = MODE_INFO[self._mode]
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = DATA_DIR / f"{info['prefix']}_{timestamp}.txt"
        meta_path = DATA_DIR / f"{info['prefix']}_{timestamp}.meta.json"
        with open(save_path, "w", encoding="utf-8") as f:
            f.write(f"# {info['header_comment']}\n# mode: {self._mode}\n# c_function: {info['c_function']}\n# data_field: {info['data_field']}\n# timestamp: {timestamp}\n# calibration_params: {DEFAULT_HEADER}\n")
            for row in self._all_data:
                f.write(f"{row[0]:.6f},{row[1]:.6f},{row[2]:.6f}\n")
        elapsed = time.time() - self._start_time if self._start_time else 0
        meta = {"mode": self._mode, "c_function": info["c_function"], "data_field": info["data_field"],
                "timestamp": timestamp, "samples": len(self._all_data), "duration_s": round(elapsed, 1),
                "file": save_path.name, "calibration_params": str(DEFAULT_HEADER)}
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        self._statusbar.showMessage(f"\u6570\u636e\u5df2\u4fdd\u5b58: {save_path}")

    def _do_save_screenshot(self):
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        info = MODE_INFO[self._mode]
        path = DATA_DIR / f"{info['prefix']}_{timestamp}.png"
        pixmap = self._xy_plot.grab()
        pixmap.save(str(path))
        self._statusbar.showMessage(f"\u622a\u56fe\u5df2\u4fdd\u5b58: {path}")

    def _load_current_params(self):
        try:
            bias, soft_iron, field = load_header(str(DEFAULT_HEADER))
            self._param_labels["HARD_IRON_X"].setText(f"{bias[0]:.6f}")
            self._param_labels["HARD_IRON_Y"].setText(f"{bias[1]:.6f}")
            self._param_labels["HARD_IRON_Z"].setText(f"{bias[2]:.6f}")
            self._param_labels["SOFT_IRON_XX"].setText(f"{soft_iron[0,0]:.6f}")
            self._param_labels["SOFT_IRON_XY"].setText(f"{soft_iron[0,1]:.6f}")
            self._param_labels["SOFT_IRON_YX"].setText(f"{soft_iron[1,0]:.6f}")
            self._param_labels["SOFT_IRON_YY"].setText(f"{soft_iron[1,1]:.6f}")
            self._param_labels["SOFT_IRON_ZZ"].setText(f"{soft_iron[2,2]:.6f}")
            self._param_labels["FIELD_STRENGTH"].setText(f"{field:.6f}")
        except Exception:
            for lbl in self._param_labels.values():
                lbl.setText("(\u65e0\u6cd5\u8bfb\u53d6)")

    def closeEvent(self, event):
        if self._reader and self._reader.isRunning():
            self._reader.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    from PySide6.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.Base, QColor(35, 35, 35))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
    palette.setColor(QPalette.ToolTipText, QColor(220, 220, 220))
    palette.setColor(QPalette.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
    palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)
    window = MagCalibratorWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
