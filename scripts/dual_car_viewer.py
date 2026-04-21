#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双车轨迹可视化器 v3

功能:
- 同时接收两个串口数据
- 实时显示两辆车轨迹
- 车头朝向根据yaw角度
- 可调整场地大小 (长x宽)
- 可放置半径15cm的锥桶
- 支持模拟模式演示
- 自动检测串口
- 发车区/停车区设置
- 数据收集与分析

使用方法:
    python scripts/dual_car_viewer.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import math
import time
from collections import deque
import copy
import json
from datetime import datetime


# ==================== 配置类 ====================
class Config:
    # 场地配置
    FIELD_WIDTH = 3.0       # 场地宽度 (米)
    FIELD_HEIGHT = 4.0      # 场地高度 (米)

    # 发车区配置
    START_ZONE_X = 0.0      # 发车区中心X
    START_ZONE_Y = 0.0      # 发车区中心Y
    START_ZONE_WIDTH = 0.5  # 发车区宽度
    START_ZONE_HEIGHT = 0.5 # 发车区高度

    # 停车区配置
    PARK_ZONE_X = 2.0       # 停车区中心X
    PARK_ZONE_Y = 3.0       # 停车区中心Y
    PARK_ZONE_WIDTH = 0.5   # 停车区宽度
    PARK_ZONE_HEIGHT = 0.5  # 停车区高度

    # 车辆配置
    CAR_LENGTH = 0.25       # 车长 (米)
    CAR_WIDTH = 0.15        # 车宽 (米)
    TRAIL_LENGTH = None     # 轨迹点数 (None=无限)

    # 锥桶配置
    CONE_RADIUS = 0.15      # 锥桶半径 (米)

    # 显示配置
    SCALE = 120             # 像素/米
    UPDATE_INTERVAL = 33    # 更新间隔 (ms) ~30fps
    PORT_SCAN_INTERVAL = 2000  # 串口扫描间隔 (ms)

    # 串口配置
    BAUDRATE = 115200
    TIMEOUT = 0.1


# ==================== 数据记录器 ====================
class DataRecorder:
    """数据记录器 - 记录双车差距数据"""

    def __init__(self):
        self.records = []
        self._lock = threading.Lock()
        self.recording = False
        self.start_time = None

    def start(self):
        """开始记录"""
        with self._lock:
            self.records.clear()
            self.recording = True
            self.start_time = time.time()

    def stop(self):
        """停止记录"""
        self.recording = False

    def record(self, car1_state, car2_state):
        """记录一条数据"""
        if not self.recording:
            return

        timestamp = time.time() - self.start_time

        # 计算差距
        dx = car2_state[0] - car1_state[0]
        dy = car2_state[1] - car1_state[1]
        distance = math.sqrt(dx*dx + dy*dy)
        yaw_diff = math.degrees(car2_state[2] - car1_state[2])

        # 角度差归一化
        while yaw_diff > 180:
            yaw_diff -= 360
        while yaw_diff < -180:
            yaw_diff += 360

        record = {
            't': round(timestamp, 3),
            'c1': {'x': round(car1_state[0], 4), 'y': round(car1_state[1], 4), 'yaw': round(math.degrees(car1_state[2]), 2)},
            'c2': {'x': round(car2_state[0], 4), 'y': round(car2_state[1], 4), 'yaw': round(math.degrees(car2_state[2]), 2)},
            'diff': {'dist': round(distance, 4), 'yaw': round(yaw_diff, 2)}
        }

        with self._lock:
            self.records.append(record)

    def get_statistics(self):
        """获取统计数据"""
        with self._lock:
            if len(self.records) < 2:
                return None

            distances = [r['diff']['dist'] for r in self.records]
            yaw_diffs = [abs(r['diff']['yaw']) for r in self.records]

            return {
                'count': len(self.records),
                'duration': self.records[-1]['t'],
                'distance': {
                    'min': min(distances),
                    'max': max(distances),
                    'avg': sum(distances) / len(distances),
                    'std': self._std(distances)
                },
                'yaw_diff': {
                    'min': min(yaw_diffs),
                    'max': max(yaw_diffs),
                    'avg': sum(yaw_diffs) / len(yaw_diffs),
                    'std': self._std(yaw_diffs)
                }
            }

    def _std(self, data):
        if len(data) < 2:
            return 0
        avg = sum(data) / len(data)
        return math.sqrt(sum((x - avg) ** 2 for x in data) / len(data))

    def export(self, filename):
        """导出数据"""
        with self._lock:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump({'time': datetime.now().isoformat(), 'records': self.records}, f, indent=2)


# ==================== 数据解析器 ====================
class DataParser:
    """解析串口数据"""

    @staticmethod
    def parse_line(line):
        """解析一行数据,返回 (x, y, yaw) 或 None"""
        line = line.strip()
        if not line:
            return None

        try:
            if line.startswith("pos:"):
                parts = line[4:].split(",")
                if len(parts) >= 3:
                    return (float(parts[0]), float(parts[1]), float(parts[2]))
            elif line.startswith("imu_att:"):
                parts = line[8:].split(",")
                if len(parts) >= 6:
                    return (0.0, 0.0, float(parts[5]))
            elif line.startswith("imu_yaw:"):
                parts = line[8:].split(",")
                if len(parts) >= 3:
                    return (0.0, 0.0, float(parts[2]))
            else:
                parts = line.split(",")
                if len(parts) >= 3:
                    return (float(parts[0]), float(parts[1]), float(parts[2]))
        except (ValueError, IndexError):
            pass
        return None


# ==================== 车辆类 ====================
class Car:
    """车辆数据类 - 线程安全"""

    def __init__(self, name, color):
        self.name = name
        self.color = color
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        # 轨迹无长度限制
        self._trail = deque()
        self.connected = False
        self.last_update = 0
        self.packet_count = 0

    @property
    def x(self):
        with self._lock:
            return self._x

    @property
    def y(self):
        with self._lock:
            return self._y

    @property
    def yaw(self):
        with self._lock:
            return self._yaw

    def get_trail_copy(self):
        """获取轨迹的副本，避免迭代时修改"""
        with self._lock:
            return list(self._trail)

    def update(self, x, y, yaw_deg):
        """更新车辆状态"""
        with self._lock:
            self._x = x
            self._y = y
            self._yaw = math.radians(yaw_deg)
            self._trail.append((x, y))
        self.last_update = time.time()
        self.packet_count += 1

    def clear_trail(self):
        """清除轨迹"""
        with self._lock:
            self._trail.clear()

    def get_state(self):
        """获取当前状态"""
        with self._lock:
            return self._x, self._y, self._yaw


# ==================== 模拟器类 ====================
class Simulator:
    """模拟器 - 生成两辆车的运动数据"""

    def __init__(self, car1, car2, callback):
        self.car1 = car1
        self.car2 = car2
        self.callback = callback
        self.running = False
        self.thread = None
        self._lock = threading.Lock()

        # 车辆1参数 - 绕外圈行驶
        self.car1_t = 0.0
        self.car1_speed = 0.5  # m/s
        self.car1_path = []    # 路径点
        self.car1_idx = 0

        # 车辆2参数 - 绕内圈8字
        self.car2_t = 0.0
        self.car2_speed = 0.4
        self.car2_path = []
        self.car2_idx = 0

        self._init_paths()

    def _init_paths(self):
        """初始化路径"""
        w, h = Config.FIELD_WIDTH, Config.FIELD_HEIGHT

        # 车辆1路径：绕场地外圈 (矩形)
        margin = 0.3
        self.car1_path = [
            (margin, margin),
            (w - margin, margin),
            (w - margin, h - margin),
            (margin, h - margin),
        ]

        # 车辆2路径：中间8字形
        cx, cy = w / 2, h / 2
        r = min(w, h) * 0.25
        self.car2_path = self._generate_figure8(cx, cy, r, 40)

    def _generate_figure8(self, cx, cy, r, n_points):
        """生成8字形路径点"""
        points = []
        for i in range(n_points + 1):
            t = 2 * math.pi * i / n_points
            x = cx + r * math.sin(t)
            y = cy + r * math.sin(2 * t) / 2
            points.append((x, y))
        return points

    def start(self):
        """启动模拟"""
        with self._lock:
            if self.running:
                return
            self.running = True

        # 初始化位置到发车区
        self.car1.update(Config.START_ZONE_X, Config.START_ZONE_Y, 0)
        self.car2.update(Config.START_ZONE_X + 0.2, Config.START_ZONE_Y + 0.2, 0)

        self.car1.connected = True
        self.car2.connected = True
        self.car1.clear_trail()
        self.car2.clear_trail()

        def run():
            print("模拟器线程启动")
            last_time = time.time()
            while self.running:
                try:
                    now = time.time()
                    dt = now - last_time
                    last_time = now

                    self._update_car1(dt)
                    self._update_car2(dt)
                except Exception as e:
                    print(f"模拟器错误: {e}")
                time.sleep(0.03)
            print("模拟器线程结束")

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def stop(self):
        """停止模拟"""
        with self._lock:
            self.running = False
        self.car1.connected = False
        self.car2.connected = False

    def _update_car1(self, dt):
        """车辆1 - 沿外圈矩形路径行驶"""
        if not self.car1_path:
            return

        # 当前目标点
        target = self.car1_path[self.car1_idx]
        x, y, yaw = self.car1.get_state()

        # 计算到目标的方向
        dx = target[0] - x
        dy = target[1] - y
        dist = math.sqrt(dx*dx + dy*dy)

        # 到达目标点，切换到下一个
        if dist < 0.1:
            self.car1_idx = (self.car1_idx + 1) % len(self.car1_path)
            target = self.car1_path[self.car1_idx]
            dx = target[0] - x
            dy = target[1] - y
            dist = math.sqrt(dx*dx + dy*dy)

        # 计算航向
        target_yaw = math.atan2(dy, dx)

        # 移动
        move_dist = self.car1_speed * dt
        if dist > 0:
            ratio = min(move_dist / dist, 1.0)
            new_x = x + dx * ratio
            new_y = y + dy * ratio
        else:
            new_x, new_y = x, y

        self.car1.update(new_x, new_y, math.degrees(target_yaw))

    def _update_car2(self, dt):
        """车辆2 - 沿8字路径行驶"""
        if not self.car2_path:
            return

        # 沿路径点移动
        self.car2_t += dt * 0.5  # 参数t的增速

        # 循环
        if self.car2_t >= 2 * math.pi:
            self.car2_t -= 2 * math.pi

        # 计算8字轨迹上的位置
        cx = Config.FIELD_WIDTH / 2
        cy = Config.FIELD_HEIGHT / 2
        r = min(Config.FIELD_WIDTH, Config.FIELD_HEIGHT) * 0.25

        t = self.car2_t
        x = cx + r * math.sin(t)
        y = cy + r * math.sin(2 * t) / 2

        # 计算切线方向 (yaw)
        dx = r * math.cos(t)
        dy = r * math.cos(2 * t)
        yaw = math.atan2(dy, dx)

        self.car2.update(x, y, math.degrees(yaw))


# ==================== 串口读取线程 ====================
class SerialThread(threading.Thread):
    """串口读取线程"""

    def __init__(self, port, car, callback):
        super().__init__(daemon=True)
        self.port = port
        self.car = car
        self.callback = callback
        self.running = False
        self.serial = None

    def run(self):
        self.running = True
        try:
            self.serial = serial.Serial(
                self.port,
                Config.BAUDRATE,
                timeout=Config.TIMEOUT
            )
            self.car.connected = True
            print(f"串口 {self.port} 已连接")

            while self.running:
                try:
                    line = self.serial.readline().decode("utf-8", errors="ignore")
                    if line:
                        data = DataParser.parse_line(line)
                        if data:
                            self.car.update(*data)
                            if self.callback:
                                self.callback()
                except serial.SerialException:
                    break
        except Exception as e:
            print(f"串口错误 {self.port}: {e}")
        finally:
            self.car.connected = False
            if self.serial and self.serial.is_open:
                self.serial.close()
            print(f"串口 {self.port} 已断开")

    def stop(self):
        self.running = False


# ==================== 主窗口类 ====================
class DualCarViewer:
    """双车轨迹可视化主窗口"""

    def __init__(self, root):
        self.root = root
        self.root.title("双车轨迹可视化器 v2")
        self.root.geometry("1300x850")

        # 车辆
        self.car1 = Car("车辆1 (红)", "#E74C3C")
        self.car2 = Car("车辆2 (蓝)", "#3498DB")

        # 串口线程
        self.thread1 = None
        self.thread2 = None

        # 模拟器
        self.simulator = Simulator(self.car1, self.car2, None)
        self.sim_mode = False

        # 锥桶列表
        self.cones = []

        # 原点偏移 (车辆1初始位置)
        self.origin_x = 0.0
        self.origin_y = 0.0

        # 可用端口列表
        self.available_ports = []

        # 数据记录器
        self.recorder = DataRecorder()

        # 构建界面
        self._build_ui()

        # 启动更新循环
        self._scan_ports()
        self._update_loop()

    def _build_ui(self):
        """构建用户界面"""
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左侧控制面板
        control_frame = ttk.Frame(main_frame, width=280)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)
        control_frame.pack_propagate(False)

        # 创建滚动区域
        canvas_scroll = tk.Canvas(control_frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(control_frame, orient="vertical", command=canvas_scroll.yview)
        scroll_frame = ttk.Frame(canvas_scroll)

        scroll_frame.bind("<Configure>", lambda e: canvas_scroll.configure(scrollregion=canvas_scroll.bbox("all")))
        canvas_scroll.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas_scroll.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas_scroll.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ===== 串口配置 =====
        port_frame = ttk.LabelFrame(scroll_frame, text="串口配置 (自动检测)")
        port_frame.pack(fill=tk.X, padx=5, pady=5)

        # 状态标签
        self.port_status = ttk.Label(port_frame, text="扫描中...", foreground="gray")
        self.port_status.pack(anchor=tk.W, padx=5, pady=2)

        # 车辆1串口
        ttk.Label(port_frame, text="车辆1 (红):").pack(anchor=tk.W, padx=5)
        self.port1_var = tk.StringVar()
        self.port1_combo = ttk.Combobox(port_frame, textvariable=self.port1_var, width=20, state="readonly")
        self.port1_combo.pack(padx=5, pady=2)

        # 车辆2串口
        ttk.Label(port_frame, text="车辆2 (蓝):").pack(anchor=tk.W, padx=5)
        self.port2_var = tk.StringVar()
        self.port2_combo = ttk.Combobox(port_frame, textvariable=self.port2_var, width=20, state="readonly")
        self.port2_combo.pack(padx=5, pady=2)

        # 按钮
        btn_frame = ttk.Frame(port_frame)
        btn_frame.pack(fill=tk.X, padx=5, pady=5)
        self.connect_btn = ttk.Button(btn_frame, text="连接串口", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.sim_btn = ttk.Button(port_frame, text="启动模拟模式", command=self._toggle_sim)
        self.sim_btn.pack(fill=tk.X, padx=5, pady=2)

        # ===== 场地配置 =====
        field_frame = ttk.LabelFrame(scroll_frame, text="场地配置")
        field_frame.pack(fill=tk.X, padx=5, pady=5)

        # 场地尺寸
        size_frame = ttk.Frame(field_frame)
        size_frame.pack(fill=tk.X, padx=5, pady=2)

        ttk.Label(size_frame, text="宽度(m):").grid(row=0, column=0, sticky=tk.W)
        self.width_var = tk.StringVar(value=str(Config.FIELD_WIDTH))
        ttk.Entry(size_frame, textvariable=self.width_var, width=8).grid(row=0, column=1, padx=2)

        ttk.Label(size_frame, text="高度(m):").grid(row=0, column=2, sticky=tk.W, padx=(10, 0))
        self.height_var = tk.StringVar(value=str(Config.FIELD_HEIGHT))
        ttk.Entry(size_frame, textvariable=self.height_var, width=8).grid(row=0, column=3, padx=2)

        ttk.Label(field_frame, text="缩放(像素/米):").pack(anchor=tk.W, padx=5)
        self.scale_var = tk.StringVar(value=str(Config.SCALE))
        ttk.Entry(field_frame, textvariable=self.scale_var, width=10).pack(padx=5, pady=2)

        ttk.Button(field_frame, text="应用场地设置", command=self._apply_field_settings).pack(pady=5)

        # ===== 发车区配置 =====
        start_frame = ttk.LabelFrame(scroll_frame, text="发车区 (原点)")
        start_frame.pack(fill=tk.X, padx=5, pady=5)

        pos_frame1 = ttk.Frame(start_frame)
        pos_frame1.pack(fill=tk.X, padx=5, pady=2)

        ttk.Label(pos_frame1, text="X:").grid(row=0, column=0)
        self.start_x_var = tk.StringVar(value=str(Config.START_ZONE_X))
        ttk.Entry(pos_frame1, textvariable=self.start_x_var, width=6).grid(row=0, column=1, padx=2)

        ttk.Label(pos_frame1, text="Y:").grid(row=0, column=2, padx=(10, 0))
        self.start_y_var = tk.StringVar(value=str(Config.START_ZONE_Y))
        ttk.Entry(pos_frame1, textvariable=self.start_y_var, width=6).grid(row=0, column=3, padx=2)

        size_frame1 = ttk.Frame(start_frame)
        size_frame1.pack(fill=tk.X, padx=5, pady=2)

        ttk.Label(size_frame1, text="宽:").grid(row=0, column=0)
        self.start_w_var = tk.StringVar(value=str(Config.START_ZONE_WIDTH))
        ttk.Entry(size_frame1, textvariable=self.start_w_var, width=6).grid(row=0, column=1, padx=2)

        ttk.Label(size_frame1, text="高:").grid(row=0, column=2, padx=(10, 0))
        self.start_h_var = tk.StringVar(value=str(Config.START_ZONE_HEIGHT))
        ttk.Entry(size_frame1, textvariable=self.start_h_var, width=6).grid(row=0, column=3, padx=2)

        ttk.Button(start_frame, text="设置原点到发车区中心", command=self._set_origin_to_start).pack(pady=2)

        # ===== 停车区配置 =====
        park_frame = ttk.LabelFrame(scroll_frame, text="停车区")
        park_frame.pack(fill=tk.X, padx=5, pady=5)

        pos_frame2 = ttk.Frame(park_frame)
        pos_frame2.pack(fill=tk.X, padx=5, pady=2)

        ttk.Label(pos_frame2, text="X:").grid(row=0, column=0)
        self.park_x_var = tk.StringVar(value=str(Config.PARK_ZONE_X))
        ttk.Entry(pos_frame2, textvariable=self.park_x_var, width=6).grid(row=0, column=1, padx=2)

        ttk.Label(pos_frame2, text="Y:").grid(row=0, column=2, padx=(10, 0))
        self.park_y_var = tk.StringVar(value=str(Config.PARK_ZONE_Y))
        ttk.Entry(pos_frame2, textvariable=self.park_y_var, width=6).grid(row=0, column=3, padx=2)

        size_frame2 = ttk.Frame(park_frame)
        size_frame2.pack(fill=tk.X, padx=5, pady=2)

        ttk.Label(size_frame2, text="宽:").grid(row=0, column=0)
        self.park_w_var = tk.StringVar(value=str(Config.PARK_ZONE_WIDTH))
        ttk.Entry(size_frame2, textvariable=self.park_w_var, width=6).grid(row=0, column=1, padx=2)

        ttk.Label(size_frame2, text="高:").grid(row=0, column=2, padx=(10, 0))
        self.park_h_var = tk.StringVar(value=str(Config.PARK_ZONE_HEIGHT))
        ttk.Entry(size_frame2, textvariable=self.park_h_var, width=6).grid(row=0, column=3, padx=2)

        # ===== 锥桶管理 =====
        cone_frame = ttk.LabelFrame(scroll_frame, text="锥桶 (半径15cm)")
        cone_frame.pack(fill=tk.X, padx=5, pady=5)

        cone_input = ttk.Frame(cone_frame)
        cone_input.pack(fill=tk.X, padx=5, pady=2)

        ttk.Label(cone_input, text="X:").grid(row=0, column=0)
        self.cone_x_var = tk.StringVar(value="1.0")
        ttk.Entry(cone_input, textvariable=self.cone_x_var, width=6).grid(row=0, column=1, padx=2)

        ttk.Label(cone_input, text="Y:").grid(row=0, column=2, padx=(10, 0))
        self.cone_y_var = tk.StringVar(value="1.0")
        ttk.Entry(cone_input, textvariable=self.cone_y_var, width=6).grid(row=0, column=3, padx=2)

        btn_cone = ttk.Frame(cone_frame)
        btn_cone.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(btn_cone, text="添加", command=self._add_cone).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(btn_cone, text="清空", command=self._clear_cones).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        ttk.Label(cone_frame, text="提示: 左键画布添加锥桶", foreground="gray").pack(padx=5)
        ttk.Label(cone_frame, text="      右键删除锥桶", foreground="gray").pack(padx=5)

        # ===== 操作 =====
        op_frame = ttk.LabelFrame(scroll_frame, text="操作")
        op_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(op_frame, text="清除轨迹", command=self._clear_trails).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(op_frame, text="重置原点", command=self._reset_origin).pack(fill=tk.X, padx=5, pady=2)

        # ===== 数据收集 =====
        data_frame = ttk.LabelFrame(scroll_frame, text="数据收集与分析")
        data_frame.pack(fill=tk.X, padx=5, pady=5)

        self.record_btn = ttk.Button(data_frame, text="开始收集数据", command=self._toggle_record)
        self.record_btn.pack(fill=tk.X, padx=5, pady=2)

        ttk.Button(data_frame, text="停止并分析", command=self._stop_and_analyze).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(data_frame, text="导出数据", command=self._export_data).pack(fill=tk.X, padx=5, pady=2)

        self.record_status = ttk.Label(data_frame, text="未记录", foreground="gray")
        self.record_status.pack(anchor=tk.W, padx=5, pady=2)

        # 分析结果显示
        self.analysis_label = ttk.Label(data_frame, text="", wraplength=240)
        self.analysis_label.pack(anchor=tk.W, padx=5, pady=2)

        # ===== 状态 =====
        status_frame = ttk.LabelFrame(scroll_frame, text="状态")
        status_frame.pack(fill=tk.X, padx=5, pady=5)

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(status_frame, textvariable=self.status_var, foreground="gray").pack(anchor=tk.W, padx=5, pady=2)

        self.car1_label = ttk.Label(status_frame, text="车辆1: 等待...", foreground=self.car1.color)
        self.car1_label.pack(anchor=tk.W, padx=5, pady=2)

        self.car2_label = ttk.Label(status_frame, text="车辆2: 等待...", foreground=self.car2.color)
        self.car2_label.pack(anchor=tk.W, padx=5, pady=2)

        self.mouse_label = ttk.Label(status_frame, text="鼠标: (-, -)")
        self.mouse_label.pack(anchor=tk.W, padx=5, pady=2)

        self.cone_label = ttk.Label(status_frame, text="锥桶: 0")
        self.cone_label.pack(anchor=tk.W, padx=5, pady=2)

        # 右侧画布
        canvas_frame = ttk.LabelFrame(main_frame, text="场地视图 (左键添加锥桶, 右键删除)")
        canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        self.canvas = tk.Canvas(canvas_frame, bg="#F5F5F5")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<Button-3>", self._on_canvas_right_click)
        self.canvas.bind("<Motion>", self._on_canvas_motion)

        # 滚轮缩放
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)      # Windows
        self.canvas.bind("<Button-4>", self._on_mouse_wheel)        # Linux scroll up
        self.canvas.bind("<Button-5>", self._on_mouse_wheel)        # Linux scroll down

        # 中键拖动平移
        self.canvas.bind("<Button-2>", self._on_middle_press)       # 中键按下
        self.canvas.bind("<B2-Motion>", self._on_middle_drag)       # 中键拖动
        self.canvas.bind("<ButtonRelease-2>", self._on_middle_release)  # 中键释放

        # 拖动状态
        self._drag_start_x = 0
        self._drag_start_y = 0
        self._drag_origin_x = 0
        self._drag_origin_y = 0

    def _scan_ports(self):
        """扫描可用串口"""
        self.available_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port1_combo["values"] = self.available_ports
        self.port2_combo["values"] = self.available_ports
        self.port_status.config(text=f"检测到 {len(self.available_ports)} 个端口")
        self.root.after(Config.PORT_SCAN_INTERVAL, self._scan_ports)

    def _toggle_connect(self):
        if self.thread1 and self.thread1.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port1 = self.port1_var.get()
        port2 = self.port2_var.get()

        if not port1 and not port2:
            messagebox.showwarning("警告", "请至少选择一个串口!")
            return

        if port1:
            self.thread1 = SerialThread(port1, self.car1, None)
            self.thread1.start()
        if port2:
            self.thread2 = SerialThread(port2, self.car2, None)
            self.thread2.start()

        self.connect_btn.config(text="断开")
        self.status_var.set("已连接串口")

    def _disconnect(self):
        if self.thread1:
            self.thread1.stop()
            self.thread1 = None
        if self.thread2:
            self.thread2.stop()
            self.thread2 = None

        self.car1.connected = False
        self.car2.connected = False
        self.connect_btn.config(text="连接串口")
        self.status_var.set("未连接")

    def _toggle_sim(self):
        if self.sim_mode:
            self.simulator.stop()
            self.sim_mode = False
            self.sim_btn.config(text="启动模拟模式")
            self.status_var.set("未连接")
        else:
            if self.thread1 or self.thread2:
                self._disconnect()
            self.car1.clear_trail()
            self.car2.clear_trail()
            self.car1.packet_count = 0
            self.car2.packet_count = 0
            self.simulator.start()
            self.sim_mode = True
            self.sim_btn.config(text="停止模拟")
            self.status_var.set("模拟模式")

    def _apply_field_settings(self):
        try:
            Config.FIELD_WIDTH = float(self.width_var.get())
            Config.FIELD_HEIGHT = float(self.height_var.get())
            Config.SCALE = float(self.scale_var.get())
            Config.START_ZONE_X = float(self.start_x_var.get())
            Config.START_ZONE_Y = float(self.start_y_var.get())
            Config.START_ZONE_WIDTH = float(self.start_w_var.get())
            Config.START_ZONE_HEIGHT = float(self.start_h_var.get())
            Config.PARK_ZONE_X = float(self.park_x_var.get())
            Config.PARK_ZONE_Y = float(self.park_y_var.get())
            Config.PARK_ZONE_WIDTH = float(self.park_w_var.get())
            Config.PARK_ZONE_HEIGHT = float(self.park_h_var.get())
        except ValueError:
            messagebox.showerror("错误", "请输入有效数值!")

    def _set_origin_to_start(self):
        self.origin_x = Config.START_ZONE_X
        self.origin_y = Config.START_ZONE_Y

    def _add_cone(self):
        try:
            x = float(self.cone_x_var.get())
            y = float(self.cone_y_var.get())
            self.cones.append((x, y))
        except ValueError:
            messagebox.showerror("错误", "请输入有效坐标!")

    def _clear_cones(self):
        self.cones.clear()

    def _on_canvas_click(self, event):
        x, y = self._screen_to_world(event.x, event.y)
        self.cones.append((x, y))

    def _on_canvas_right_click(self, event):
        if not self.cones:
            return
        click_x, click_y = self._screen_to_world(event.x, event.y)
        min_dist = float("inf")
        min_idx = -1
        for i, (cx, cy) in enumerate(self.cones):
            dist = math.sqrt((cx - click_x)**2 + (cy - click_y)**2)
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx >= 0 and min_dist < Config.CONE_RADIUS * 2:
            del self.cones[min_idx]

    def _on_canvas_motion(self, event):
        mx, my = self._screen_to_world(event.x, event.y)
        self.mouse_label.config(text=f"鼠标: ({mx:.2f}, {my:.2f})")

    def _on_mouse_wheel(self, event):
        """滚轮缩放 - 以鼠标位置为中心"""
        # 获取鼠标位置的世界坐标 (缩放前)
        world_x, world_y = self._screen_to_world(event.x, event.y)

        # 计算缩放因子
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            factor = 1.2
        elif event.num == 5 or (hasattr(event, 'delta') and event.delta < 0):
            factor = 1 / 1.2
        else:
            return

        # 更新缩放
        new_scale = Config.SCALE * factor
        if 20 <= new_scale <= 800:  # 限制缩放范围
            Config.SCALE = new_scale
            self.scale_var.set(str(round(Config.SCALE, 1)))

            # 重新计算原点，使鼠标位置的世界坐标保持不变
            ox, oy = self._get_canvas_offset()
            # 新的屏幕坐标
            new_sx = ox + (world_x - self.origin_x) * Config.SCALE
            new_sy = oy - (world_y - self.origin_y) * Config.SCALE
            # 调整原点使鼠标位置对齐
            self.origin_x = world_x - (event.x - ox) / Config.SCALE
            self.origin_y = world_y + (event.y - oy) / Config.SCALE

    def _on_middle_press(self, event):
        """中键按下 - 开始拖动"""
        self._drag_start_x = event.x
        self._drag_start_y = event.y
        self._drag_origin_x = self.origin_x
        self._drag_origin_y = self.origin_y
        self.canvas.config(cursor="fleur")

    def _on_middle_drag(self, event):
        """中键拖动 - 平移视图"""
        dx = event.x - self._drag_start_x
        dy = event.y - self._drag_start_y

        self.origin_x = self._drag_origin_x - dx / Config.SCALE
        self.origin_y = self._drag_origin_y + dy / Config.SCALE

    def _on_middle_release(self, event):
        """中键释放"""
        self.canvas.config(cursor="")

    def _clear_trails(self):
        self.car1.clear_trail()
        self.car2.clear_trail()

    def _reset_origin(self):
        self.origin_x = 0.0
        self.origin_y = 0.0

    def _get_canvas_offset(self):
        """获取画布原点偏移 (左下角为场地原点)"""
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        margin = 50
        return margin, h - margin

    def _world_to_screen(self, x, y):
        """世界坐标转屏幕坐标 (原点在左下角)"""
        ox, oy = self._get_canvas_offset()
        sx = ox + (x - self.origin_x) * Config.SCALE
        sy = oy - (y - self.origin_y) * Config.SCALE
        return sx, sy

    def _screen_to_world(self, sx, sy):
        """屏幕坐标转世界坐标"""
        ox, oy = self._get_canvas_offset()
        x = (sx - ox) / Config.SCALE + self.origin_x
        y = (oy - sy) / Config.SCALE + self.origin_y
        return x, y

    def _update_loop(self):
        self._draw()
        self._update_status()
        self._record_data()
        self.root.after(Config.UPDATE_INTERVAL, self._update_loop)

    def _update_status(self):
        if self.car1.connected:
            x, y, yaw = self.car1.get_state()
            self.car1_label.config(text=f"车辆1: ({x:.2f}, {y:.2f}) Yaw: {math.degrees(yaw):.1f}° [{self.car1.packet_count}]")
        else:
            self.car1_label.config(text="车辆1: 未连接")

        if self.car2.connected:
            x, y, yaw = self.car2.get_state()
            self.car2_label.config(text=f"车辆2: ({x:.2f}, {y:.2f}) Yaw: {math.degrees(yaw):.1f}° [{self.car2.packet_count}]")
        else:
            self.car2_label.config(text="车辆2: 未连接")

        self.cone_label.config(text=f"锥桶: {len(self.cones)}")

        # 更新记录状态
        if self.recorder.recording:
            count = len(self.recorder.records)
            self.record_status.config(text=f"记录中... {count} 条数据", foreground="green")

    def _record_data(self):
        """记录数据"""
        if self.recorder.recording and self.car1.connected and self.car2.connected:
            state1 = self.car1.get_state()
            state2 = self.car2.get_state()
            self.recorder.record(state1, state2)

    def _toggle_record(self):
        """切换记录状态"""
        if self.recorder.recording:
            self.recorder.stop()
            self.record_btn.config(text="开始收集数据")
            self.record_status.config(text=f"已停止，共 {len(self.recorder.records)} 条数据", foreground="gray")
        else:
            if not (self.car1.connected or self.car2.connected):
                messagebox.showwarning("警告", "请先连接串口或启动模拟模式!")
                return
            self.recorder.start()
            self.record_btn.config(text="停止收集")
            self.record_status.config(text="记录中...", foreground="green")
            self.analysis_label.config(text="")

    def _stop_and_analyze(self):
        """停止记录并分析"""
        self.recorder.stop()
        self.record_btn.config(text="开始收集数据")

        stats = self.recorder.get_statistics()
        if not stats:
            self.analysis_label.config(text="没有足够的数据进行分析", foreground="red")
            return

        # 显示分析结果
        result = f"数据: {stats['count']}条, 时长: {stats['duration']:.1f}s\n"
        result += f"距离: 均值{stats['distance']['avg']:.3f}m\n"
        result += f"       范围{stats['distance']['min']:.3f}~{stats['distance']['max']:.3f}m\n"
        result += f"       标准差{stats['distance']['std']:.3f}m\n"
        result += f"角度: 均值{stats['yaw_diff']['avg']:.2f}°\n"
        result += f"       范围{stats['yaw_diff']['min']:.2f}~{stats['yaw_diff']['max']:.2f}°\n"
        result += f"       标准差{stats['yaw_diff']['std']:.2f}°"

        self.analysis_label.config(text=result, foreground="blue")
        self.record_status.config(text=f"分析完成，{stats['count']} 条数据")

    def _export_data(self):
        """导出数据"""
        if not self.recorder.records:
            messagebox.showwarning("警告", "没有数据可导出!")
            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON文件", "*.json")],
            initialfilename=f"dual_car_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        if filename:
            self.recorder.export(filename)
            messagebox.showinfo("成功", f"数据已导出: {filename}")

    def _draw(self):
        """绘制场地"""
        self.canvas.delete("all")

        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 50 or h < 50:
            return

        # 绘制场地边界
        self._draw_field()

        # 绘制发车区
        self._draw_zone(Config.START_ZONE_X, Config.START_ZONE_Y,
                       Config.START_ZONE_WIDTH, Config.START_ZONE_HEIGHT,
                       "#2ECC71", "发车区")

        # 绘制停车区
        self._draw_zone(Config.PARK_ZONE_X, Config.PARK_ZONE_Y,
                       Config.PARK_ZONE_WIDTH, Config.PARK_ZONE_HEIGHT,
                       "#9B59B6", "停车区")

        # 绘制锥桶
        for cx, cy in self.cones:
            self._draw_cone(cx, cy)

        # 绘制轨迹
        self._draw_trail(self.car1)
        self._draw_trail(self.car2)

        # 绘制车辆
        self._draw_car(self.car1)
        self._draw_car(self.car2)

        # 绘制图例
        self._draw_legend()

    def _draw_field(self):
        """绘制场地边界"""
        ox, oy = self._get_canvas_offset()

        # 场地矩形
        x1, y1 = self._world_to_screen(0, 0)
        x2, y2 = self._world_to_screen(Config.FIELD_WIDTH, Config.FIELD_HEIGHT)

        self.canvas.create_rectangle(x1, y2, x2, y1, outline="#333", width=3)

        # 刻度
        for i in range(0, int(Config.FIELD_WIDTH) + 1):
            sx, sy = self._world_to_screen(i, 0)
            self.canvas.create_line(sx, sy, sx, sy + 5, fill="#666", width=1)
            if i > 0:
                self.canvas.create_text(sx, sy + 15, text=f"{i}m", fill="#666", font=("Arial", 8))

        for i in range(0, int(Config.FIELD_HEIGHT) + 1):
            sx, sy = self._world_to_screen(0, i)
            self.canvas.create_line(sx - 5, sy, sx, sy, fill="#666", width=1)
            if i > 0:
                self.canvas.create_text(sx - 20, sy, text=f"{i}m", fill="#666", font=("Arial", 8))

    def _draw_zone(self, cx, cy, w, h, color, label):
        """绘制区域 (发车区/停车区)"""
        x1, y1 = self._world_to_screen(cx - w/2, cy - h/2)
        x2, y2 = self._world_to_screen(cx + w/2, cy + h/2)

        self.canvas.create_rectangle(x1, y2, x2, y1, fill=color, outline=color, width=2, stipple="gray50")
        self.canvas.create_text((x1+x2)/2, (y1+y2)/2, text=label, fill="white", font=("Arial", 9, "bold"))

    def _draw_cone(self, x, y):
        sx, sy = self._world_to_screen(x, y)
        r = Config.CONE_RADIUS * Config.SCALE
        self.canvas.create_oval(sx-r, sy-r, sx+r, sy+r, fill="#FF6B35", outline="#D63031", width=2)

    def _draw_trail(self, car):
        trail = car.get_trail_copy()
        if len(trail) < 2:
            return

        points = []
        for x, y in trail:
            sx, sy = self._world_to_screen(x, y)
            points.extend([sx, sy])

        if len(points) >= 4:
            self.canvas.create_line(points, fill=car.color, width=2, smooth=True)

    def _draw_car(self, car):
        x, y, yaw = car.get_state()
        sx, sy = self._world_to_screen(x, y)

        length = Config.CAR_LENGTH * Config.SCALE
        width = Config.CAR_WIDTH * Config.SCALE

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # 车身顶点
        half_l, half_w = length/2, width/2
        corners = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]

        pts = []
        for px, py in corners:
            rx = px * cos_yaw - py * sin_yaw
            ry = px * sin_yaw + py * cos_yaw
            pts.extend([sx + rx, sy - ry])

        self.canvas.create_polygon(pts, fill=car.color, outline="black", width=2)

        # 车头箭头
        arrow_len = length * 0.5
        ax = sx + arrow_len * cos_yaw
        ay = sy - arrow_len * sin_yaw
        self.canvas.create_line(sx, sy, ax, ay, fill="white", width=2, arrow=tk.LAST)

        # 状态点
        status_color = "#2ECC71" if car.connected else "#E74C3C"
        self.canvas.create_oval(sx-4, sy-4, sx+4, sy+4, fill=status_color, outline=status_color)

        # 右上方显示实时坐标
        yaw_deg = math.degrees(yaw)
        coord_text = f"({x:.2f}, {y:.2f}) {yaw_deg:.1f}°"
        # 计算文字位置 (车辆右上方)
        text_x = sx + length * 0.6
        text_y = sy - length * 0.8
        self.canvas.create_text(text_x, text_y, text=coord_text, anchor=tk.W,
                               fill=car.color, font=("Consolas", 9, "bold"))

    def _draw_legend(self):
        x, y = 20, 20

        self.canvas.create_rectangle(x, y, x+20, y+10, fill=self.car1.color)
        self.canvas.create_text(x+25, y+5, text=self.car1.name, anchor=tk.W, font=("Arial", 9))

        y += 18
        self.canvas.create_rectangle(x, y, x+20, y+10, fill=self.car2.color)
        self.canvas.create_text(x+25, y+5, text=self.car2.name, anchor=tk.W, font=("Arial", 9))

        y += 18
        self.canvas.create_oval(x+5, y, x+15, y+10, fill="#FF6B35", outline="#D63031")
        self.canvas.create_text(x+25, y+5, text="锥桶 (r=15cm)", anchor=tk.W, font=("Arial", 9))


def main():
    root = tk.Tk()
    app = DualCarViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
