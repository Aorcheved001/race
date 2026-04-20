# -*- coding: utf-8 -*-
"""
INS Trajectory Visualizer
Real-time 2D trajectory plot with car model in a bounded arena.
Supports dual serial ports: INS (wireless) + RTK (reference).
"""
import sys
import serial
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# =================================================================
# Serial port config
# =================================================================
INS_PORT = 'COM24'
RTK_PORT = 'COM33'  # Future: RTK reference
BAUD_RATE = 115200
TIMEOUT = 0.1

# Arena config (meters)
ARENA_X_MIN, ARENA_X_MAX = -5.0, 5.0
ARENA_Y_MIN, ARENA_Y_MAX = -5.0, 5.0

# Car model config (meters)
CAR_LENGTH = 0.25
CAR_WIDTH = 0.15

# =================================================================
# Serial readers
# =================================================================
class SerialReader(QtCore.QThread):
    """Background thread to read serial port and emit parsed data."""
    data_received = QtCore.pyqtSignal(float, float, float)  # x, y, yaw

    def __init__(self, port, baud=BAUD_RATE, timeout=TIMEOUT):
        super().__init__()
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=self.timeout)
            print(f"[{self.port_name}] Connected")
        except Exception as e:
            print(f"[{self.port_name}] Failed: {e}")
            return

        buffer = ""
        while self.running:
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buffer += chunk.decode('utf-8', errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip().strip('\r')
                    # Parse format: "x,y,yaw" or "[timestamp] x,y,yaw"
                    parts = line.replace(']', ',').split(',')
                    if len(parts) >= 3:
                        try:
                            x = float(parts[-3])
                            y = float(parts[-2])
                            yaw = float(parts[-1])
                            self.data_received.emit(x, y, yaw)
                        except ValueError:
                            pass
            except Exception as e:
                print(f"[{self.port_name}] Read error: {e}")
                break

        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[{self.port_name}] Disconnected")

    def stop(self):
        self.running = False
        self.wait(1000)


# =================================================================
# Main Window
# =================================================================
class TrajectoryWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("INS Trajectory Visualizer")
        self.resize(900, 700)

        # Data buffers
        self.ins_x, self.ins_y, self.ins_yaw = [], [], []
        self.rtk_x, self.rtk_y, self.rtk_yaw = [], [], []
        self.max_points = 10000

        # Setup UI
        self.setup_ui()
        self.setup_plots()

        # Start INS serial reader
        self.ins_reader = SerialReader(INS_PORT)
        self.ins_reader.data_received.connect(self.on_ins_data)
        self.ins_reader.start()

        # RTK reader (commented out for future use)
        # self.rtk_reader = SerialReader(RTK_PORT)
        # self.rtk_reader.data_received.connect(self.on_rtk_data)
        # self.rtk_reader.start()

    def setup_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # Plot widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        # Info panel
        info_layout = QtWidgets.QHBoxLayout()
        self.lbl_ins = QtWidgets.QLabel("INS: x=0.00, y=0.00, yaw=0.00")
        self.lbl_rtk = QtWidgets.QLabel("RTK: (not connected)")
        self.lbl_count = QtWidgets.QLabel("Points: 0")
        info_layout.addWidget(self.lbl_ins)
        info_layout.addWidget(self.lbl_rtk)
        info_layout.addWidget(self.lbl_count)
        layout.addLayout(info_layout)

        # Control buttons
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_clear = QtWidgets.QPushButton("Clear Trajectory")
        self.btn_clear.clicked.connect(self.clear_trajectory)
        btn_layout.addWidget(self.btn_clear)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def setup_plots(self):
        self.plot = self.plot_widget.addPlot(title="Vehicle Trajectory (Top View)")
        self.plot.setAspectLocked(True)
        self.plot.setRange(xRange=(ARENA_X_MIN, ARENA_X_MAX),
                           yRange=(ARENA_Y_MIN, ARENA_Y_MAX))
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel('left', 'Y (m)')
        self.plot.setLabel('bottom', 'X (m)')

        # Arena boundary
        arena_rect = pg.QtGui.QGraphicsRectItem(
            ARENA_X_MIN, ARENA_Y_MIN,
            ARENA_X_MAX - ARENA_X_MIN, ARENA_Y_MAX - ARENA_Y_MIN
        )
        arena_rect.setPen(pg.mkPen('w', width=2))
        self.plot.addItem(arena_rect)

        # INS trajectory line
        self.ins_curve = self.plot.plot(pen=pg.mkPen('y', width=2), name='INS')
        # RTK trajectory line (future)
        self.rtk_curve = self.plot.plot(pen=pg.mkPen('c', width=1, style=QtCore.Qt.DashLine), name='RTK')

        # Car model (INS)
        self.car_ins = pg.QtGui.QGraphicsPolygonItem()
        self.car_ins.setPen(pg.mkPen('y', width=2))
        self.car_ins.setBrush(pg.mkBrush(100, 100, 0, 150))
        self.plot.addItem(self.car_ins)

        # Car model (RTK, future)
        self.car_rtk = pg.QtGui.QGraphicsPolygonItem()
        self.car_rtk.setPen(pg.mkPen('c', width=1))
        self.car_rtk.setBrush(pg.mkBrush(0, 150, 150, 100))
        self.plot.addItem(self.car_rtk)

        # Legend
        legend = self.plot.addLegend()
        legend.addItem(self.ins_curve, 'INS Trajectory')
        legend.addItem(self.rtk_curve, 'RTK Trajectory')

    def on_ins_data(self, x, y, yaw):
        self.ins_x.append(x)
        self.ins_y.append(y)
        self.ins_yaw.append(yaw)

        # Limit buffer size
        if len(self.ins_x) > self.max_points:
            self.ins_x = self.ins_x[-self.max_points:]
            self.ins_y = self.ins_y[-self.max_points:]
            self.ins_yaw = self.ins_yaw[-self.max_points:]

        self.update_plot()
        self.lbl_ins.setText(f"INS: x={x:.2f}, y={y:.2f}, yaw={np.degrees(yaw):.1f} deg")
        self.lbl_count.setText(f"Points: {len(self.ins_x)}")

    def on_rtk_data(self, x, y, yaw):
        self.rtk_x.append(x)
        self.rtk_y.append(y)
        self.rtk_yaw.append(yaw)
        if len(self.rtk_x) > self.max_points:
            self.rtk_x = self.rtk_x[-self.max_points:]
            self.rtk_y = self.rtk_y[-self.max_points:]
            self.rtk_yaw = self.rtk_yaw[-self.max_points:]
        self.update_plot()
        self.lbl_rtk.setText(f"RTK: x={x:.2f}, y={y:.2f}, yaw={np.degrees(yaw):.1f} deg")

    def update_plot(self):
        # Update INS trajectory
        if len(self.ins_x) > 1:
            self.ins_curve.setData(self.ins_x, self.ins_y)
        else:
            self.ins_curve.setData([], [])

        # Update RTK trajectory
        if len(self.rtk_x) > 1:
            self.rtk_curve.setData(self.rtk_x, self.rtk_y)
        else:
            self.rtk_curve.setData([], [])

        # Update INS car model
        if self.ins_x:
            cx, cy, cyaw = self.ins_x[-1], self.ins_y[-1], self.ins_yaw[-1]
            self.car_ins.setPolygon(self.make_car_polygon(cx, cy, cyaw, CAR_LENGTH, CAR_WIDTH))

        # Update RTK car model
        if self.rtk_x:
            cx, cy, cyaw = self.rtk_x[-1], self.rtk_y[-1], self.rtk_yaw[-1]
            self.car_rtk.setPolygon(self.make_car_polygon(cx, cy, cyaw, CAR_LENGTH, CAR_WIDTH))

    @staticmethod
    def make_car_polygon(x, y, yaw, length, width):
        """Create car rectangle polygon rotated by yaw angle."""
        half_l, half_w = length / 2, width / 2
        # Car corners in local frame (front is +x)
        corners = np.array([
            [half_l, -half_w],
            [half_l, half_w],
            [-half_l, half_w],
            [-half_l, -half_w],
        ])
        # Rotation matrix
        cos_a, sin_a = np.cos(yaw), np.sin(yaw)
        R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        # Rotate and translate
        rotated = corners @ R.T
        rotated[:, 0] += x
        rotated[:, 1] += y
        # Convert to QGraphicsPolygonItem format
        qpoly = pg.QtGui.QPolygonF()
        for px, py in rotated:
            qpoly.append(pg.QtCore.QPointF(px, py))
        return qpoly

    def clear_trajectory(self):
        self.ins_x, self.ins_y, self.ins_yaw = [], [], []
        self.rtk_x, self.rtk_y, self.rtk_yaw = [], [], []
        self.ins_curve.setData([], [])
        self.rtk_curve.setData([], [])
        self.lbl_count.setText("Points: 0")

    def closeEvent(self, event):
        self.ins_reader.stop()
        # self.rtk_reader.stop()
        event.accept()


# =================================================================
# Main
# =================================================================
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = TrajectoryWindow()
    win.show()
    sys.exit(app.exec_())
