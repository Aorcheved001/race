#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双车差距分析与AI调参工具

功能:
- 同时采集两辆车的实时数据
- 记录时间戳、位置、航向
- 计算两车之间的距离、角度差
- 分析数据统计特性
- 支持导出数据供AI调参

使用方法:
    python scripts/multi_car_analyzer.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import math
import time
from collections import deque
import json
import os
from datetime import datetime


# ==================== 配置 ====================
class Config:
    BAUDRATE = 115200
    TIMEOUT = 0.1
    SAMPLE_INTERVAL = 0.05  # 采样间隔 (秒)
    MAX_SAMPLES = 10000     # 最大样本数


# ==================== 数据记录类 ====================
class DataRecorder:
    """数据记录器"""

    def __init__(self):
        self.records = []  # [(timestamp, car1_data, car2_data, diff_data), ...]
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
        yaw_diff = car2_state[2] - car1_state[2]

        # 角度差归一化到 [-180, 180]
        while yaw_diff > 180:
            yaw_diff -= 360
        while yaw_diff < -180:
            yaw_diff += 360

        record = {
            'timestamp': round(timestamp, 3),
            'car1': {
                'x': round(car1_state[0], 4),
                'y': round(car1_state[1], 4),
                'yaw': round(math.degrees(car1_state[2]), 2)
            },
            'car2': {
                'x': round(car2_state[0], 4),
                'y': round(car2_state[1], 4),
                'yaw': round(math.degrees(car2_state[2]), 2)
            },
            'diff': {
                'dx': round(dx, 4),
                'dy': round(dy, 4),
                'distance': round(distance, 4),
                'yaw_diff': round(yaw_diff, 2)
            }
        }

        with self._lock:
            if len(self.records) < Config.MAX_SAMPLES:
                self.records.append(record)

    def get_statistics(self):
        """获取统计数据"""
        with self._lock:
            if len(self.records) < 2:
                return None

            distances = [r['diff']['distance'] for r in self.records]
            yaw_diffs = [abs(r['diff']['yaw_diff']) for r in self.records]

            return {
                'count': len(self.records),
                'duration': self.records[-1]['timestamp'],
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
        """计算标准差"""
        if len(data) < 2:
            return 0
        avg = sum(data) / len(data)
        variance = sum((x - avg) ** 2 for x in data) / len(data)
        return math.sqrt(variance)

    def export(self, filename):
        """导出数据"""
        with self._lock:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump({
                    'export_time': datetime.now().isoformat(),
                    'records': self.records
                }, f, indent=2, ensure_ascii=False)

    def export_for_ai(self, filename):
        """导出为AI调参格式"""
        with self._lock:
            if not self.records:
                return

            # 生成AI调参提示
            stats = self.get_statistics()
            prompt = f"""# 双车差距数据分析报告

## 数据概况
- 采样数量: {stats['count']} 条
- 记录时长: {stats['duration']:.2f} 秒

## 距离差距统计
- 最小值: {stats['distance']['min']:.4f} 米
- 最大值: {stats['distance']['max']:.4f} 米
- 平均值: {stats['distance']['avg']:.4f} 米
- 标准差: {stats['distance']['std']:.4f} 米

## 航向差距统计
- 最小值: {stats['yaw_diff']['min']:.2f}°
- 最大值: {stats['yaw_diff']['max']:.2f}°
- 平均值: {stats['yaw_diff']['avg']:.2f}°
- 标准差: {stats['yaw_diff']['std']:.2f}°

## 原始数据样本 (前50条)
```
timestamp, car1_x, car1_y, car1_yaw, car2_x, car2_y, car2_yaw, distance, yaw_diff
"""
            for r in self.records[:50]:
                prompt += f"{r['timestamp']}, {r['car1']['x']}, {r['car1']['y']}, {r['car1']['yaw']}, {r['car2']['x']}, {r['car2']['y']}, {r['car2']['yaw']}, {r['diff']['distance']}, {r['diff']['yaw_diff']}\n"

            prompt += "```\n\n## 调参建议请求\n请根据以上数据，分析两车的跟踪性能，并给出PID参数调整建议。"

            with open(filename, 'w', encoding='utf-8') as f:
                f.write(prompt)


# ==================== 车辆数据类 ====================
class CarData:
    """车辆数据 (线程安全)"""

    def __init__(self, name):
        self.name = name
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self.connected = False
        self.packet_count = 0

    def update(self, x, y, yaw_deg):
        with self._lock:
            self._x = x
            self._y = y
            self._yaw = math.radians(yaw_deg)
        self.packet_count += 1

    def get_state(self):
        with self._lock:
            return self._x, self._y, self._yaw


# ==================== 串口线程 ====================
class SerialThread(threading.Thread):
    def __init__(self, port, car):
        super().__init__(daemon=True)
        self.port = port
        self.car = car
        self.running = False
        self.serial = None

    def run(self):
        self.running = True
        try:
            self.serial = serial.Serial(self.port, Config.BAUDRATE, timeout=Config.TIMEOUT)
            self.car.connected = True
            print(f"串口 {self.port} 已连接")

            while self.running:
                try:
                    line = self.serial.readline().decode("utf-8", errors="ignore")
                    if line:
                        data = self._parse_line(line.strip())
                        if data:
                            self.car.update(*data)
                except serial.SerialException:
                    break
        except Exception as e:
            print(f"串口错误 {self.port}: {e}")
        finally:
            self.car.connected = False
            if self.serial and self.serial.is_open:
                self.serial.close()

    def stop(self):
        self.running = False

    def _parse_line(self, line):
        """解析数据行"""
        try:
            if line.startswith("pos:"):
                parts = line[4:].split(",")
            elif line.startswith("imu_att:"):
                parts = line[8:].split(",")
                if len(parts) >= 6:
                    return (0.0, 0.0, float(parts[5]))
                return None
            elif line.startswith("imu_yaw:"):
                parts = line[8:].split(",")
                if len(parts) >= 3:
                    return (0.0, 0.0, float(parts[2]))
                return None
            else:
                parts = line.split(",")

            if len(parts) >= 3:
                return (float(parts[0]), float(parts[1]), float(parts[2]))
        except (ValueError, IndexError):
            pass
        return None


# ==================== 主窗口 ====================
class MultiCarAnalyzer:
    def __init__(self, root):
        self.root = root
        self.root.title("双车差距分析与AI调参工具")
        self.root.geometry("900x700")

        # 车辆数据
        self.car1 = CarData("车辆1")
        self.car2 = CarData("车辆2")

        # 串口线程
        self.thread1 = None
        self.thread2 = None

        # 数据记录器
        self.recorder = DataRecorder()

        # 采样线程
        self.sampling = False
        self.sample_thread = None

        self._build_ui()
        self._scan_ports()
        self._update_loop()

    def _build_ui(self):
        # 主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # ===== 串口配置 =====
        port_frame = ttk.LabelFrame(main_frame, text="串口配置")
        port_frame.pack(fill=tk.X, pady=5)

        port_inner = ttk.Frame(port_frame)
        port_inner.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(port_inner, text="车辆1 (主车):").grid(row=0, column=0, sticky=tk.W)
        self.port1_var = tk.StringVar()
        self.port1_combo = ttk.Combobox(port_inner, textvariable=self.port1_var, width=15, state="readonly")
        self.port1_combo.grid(row=0, column=1, padx=5)

        ttk.Label(port_inner, text="车辆2 (从车):").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        self.port2_var = tk.StringVar()
        self.port2_combo = ttk.Combobox(port_inner, textvariable=self.port2_var, width=15, state="readonly")
        self.port2_combo.grid(row=0, column=3, padx=5)

        ttk.Button(port_inner, text="刷新", command=self._scan_ports).grid(row=0, column=4, padx=5)
        self.connect_btn = ttk.Button(port_inner, text="连接", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=5, padx=5)

        # ===== 控制面板 =====
        ctrl_frame = ttk.LabelFrame(main_frame, text="数据采集控制")
        ctrl_frame.pack(fill=tk.X, pady=5)

        ctrl_inner = ttk.Frame(ctrl_frame)
        ctrl_inner.pack(fill=tk.X, padx=5, pady=5)

        self.record_btn = ttk.Button(ctrl_inner, text="开始记录", command=self._toggle_record)
        self.record_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(ctrl_inner, text="清除数据", command=self._clear_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_inner, text="导出JSON", command=self._export_json).pack(side=tk.LEFT, padx=5)
        ttk.Button(ctrl_inner, text="导出AI调参格式", command=self._export_ai).pack(side=tk.LEFT, padx=5)

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(ctrl_inner, textvariable=self.status_var, foreground="gray").pack(side=tk.RIGHT, padx=5)

        # ===== 实时数据 =====
        realtime_frame = ttk.LabelFrame(main_frame, text="实时数据")
        realtime_frame.pack(fill=tk.X, pady=5)

        realtime_inner = ttk.Frame(realtime_frame)
        realtime_inner.pack(fill=tk.X, padx=5, pady=5)

        # 车辆1
        ttk.Label(realtime_inner, text="车辆1:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky=tk.W)
        self.car1_var = tk.StringVar(value="等待数据...")
        ttk.Label(realtime_inner, textvariable=self.car1_var, font=("Consolas", 10)).grid(row=0, column=1, sticky=tk.W, padx=10)

        # 车辆2
        ttk.Label(realtime_inner, text="车辆2:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky=tk.W)
        self.car2_var = tk.StringVar(value="等待数据...")
        ttk.Label(realtime_inner, textvariable=self.car2_var, font=("Consolas", 10)).grid(row=1, column=1, sticky=tk.W, padx=10)

        # 差距
        ttk.Label(realtime_inner, text="差距:", font=("Arial", 10, "bold"), foreground="red").grid(row=2, column=0, sticky=tk.W)
        self.diff_var = tk.StringVar(value="距离: -, 角度差: -")
        ttk.Label(realtime_inner, textvariable=self.diff_var, font=("Consolas", 10), foreground="red").grid(row=2, column=1, sticky=tk.W, padx=10)

        # ===== 统计数据 =====
        stats_frame = ttk.LabelFrame(main_frame, text="统计分析")
        stats_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.stats_text = tk.Text(stats_frame, height=15, font=("Consolas", 10))
        self.stats_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ===== AI调参区域 =====
        ai_frame = ttk.LabelFrame(main_frame, text="AI调参建议")
        ai_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.ai_text = tk.Text(ai_frame, height=10, font=("Consolas", 10))
        self.ai_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        ai_btn_frame = ttk.Frame(ai_frame)
        ai_btn_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(ai_btn_frame, text="生成调参报告", command=self._generate_ai_report).pack(side=tk.LEFT, padx=5)
        ttk.Button(ai_btn_frame, text="复制到剪贴板", command=self._copy_to_clipboard).pack(side=tk.LEFT, padx=5)

    def _scan_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port1_combo["values"] = ports
        self.port2_combo["values"] = ports

    def _toggle_connect(self):
        if self.thread1 and self.thread1.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port1 = self.port1_var.get()
        port2 = self.port2_var.get()

        if not port1 or not port2:
            messagebox.showwarning("警告", "请选择两个串口!")
            return

        self.thread1 = SerialThread(port1, self.car1)
        self.thread1.start()
        self.thread2 = SerialThread(port2, self.car2)
        self.thread2.start()

        self.connect_btn.config(text="断开")
        self.status_var.set("已连接")

    def _disconnect(self):
        if self.thread1:
            self.thread1.stop()
            self.thread1 = None
        if self.thread2:
            self.thread2.stop()
            self.thread2 = None

        self.car1.connected = False
        self.car2.connected = False
        self.connect_btn.config(text="连接")
        self.status_var.set("未连接")

    def _toggle_record(self):
        if self.recorder.recording:
            self.recorder.stop()
            self.record_btn.config(text="开始记录")
            self.status_var.set("已停止记录")
        else:
            if not (self.car1.connected and self.car2.connected):
                messagebox.showwarning("警告", "请先连接两个串口!")
                return
            self.recorder.start()
            self.record_btn.config(text="停止记录")
            self.status_var.set("正在记录...")

    def _clear_data(self):
        self.recorder.records.clear()
        self.stats_text.delete("1.0", tk.END)
        self.ai_text.delete("1.0", tk.END)

    def _export_json(self):
        if not self.recorder.records:
            messagebox.showwarning("警告", "没有数据可导出!")
            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON文件", "*.json")],
            initialfilename=f"multi_car_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        if filename:
            self.recorder.export(filename)
            messagebox.showinfo("成功", f"数据已导出到: {filename}")

    def _export_ai(self):
        if not self.recorder.records:
            messagebox.showwarning("警告", "没有数据可导出!")
            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".md",
            filetypes=[("Markdown文件", "*.md"), ("文本文件", "*.txt")],
            initialfilename=f"ai_tuning_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.md"
        )
        if filename:
            self.recorder.export_for_ai(filename)
            messagebox.showinfo("成功", f"AI调参报告已导出到: {filename}")

    def _generate_ai_report(self):
        stats = self.recorder.get_statistics()
        if not stats:
            self.ai_text.delete("1.0", tk.END)
            self.ai_text.insert(tk.END, "请先采集数据!")
            return

        report = f"""# 双车差距分析报告

## 数据概况
- 采样数量: {stats['count']} 条
- 记录时长: {stats['duration']:.2f} 秒

## 距离差距分析
| 指标 | 值 |
|------|-----|
| 最小值 | {stats['distance']['min']:.4f} 米 |
| 最大值 | {stats['distance']['max']:.4f} 米 |
| 平均值 | {stats['distance']['avg']:.4f} 米 |
| 标准差 | {stats['distance']['std']:.4f} 米 |

## 航向差距分析
| 指标 | 值 |
|------|-----|
| 最小值 | {stats['yaw_diff']['min']:.2f}° |
| 最大值 | {stats['yaw_diff']['max']:.2f}° |
| 平均值 | {stats['yaw_diff']['avg']:.2f}° |
| 标准差 | {stats['yaw_diff']['std']:.2f}° |

## AI调参建议提示词

请将以上数据提供给AI助手，并请求以下内容：

1. **距离跟踪性能分析**
   - 平均距离差距是否在可接受范围内？
   - 距离波动(标准差)是否过大？

2. **航向跟踪性能分析**
   - 平均航向差是否合理？
   - 是否存在系统性偏差？

3. **参数调整建议**
   - PID参数如何调整以减小差距？
   - 是否需要调整前馈控制？

4. **潜在问题诊断**
   - 两车之间是否存在通信延迟？
   - 是否存在传感器漂移？
"""
        self.ai_text.delete("1.0", tk.END)
        self.ai_text.insert(tk.END, report)

    def _copy_to_clipboard(self):
        text = self.ai_text.get("1.0", tk.END)
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo("成功", "已复制到剪贴板!")

    def _update_loop(self):
        self._update_display()
        self.root.after(100, self._update_loop)

    def _update_display(self):
        # 更新车辆1数据
        x1, y1, yaw1 = self.car1.get_state()
        status1 = "已连接" if self.car1.connected else "未连接"
        self.car1_var.set(f"X: {x1:.3f}  Y: {y1:.3f}  Yaw: {math.degrees(yaw1):.1f}°  [{self.car1.packet_count}]  {status1}")

        # 更新车辆2数据
        x2, y2, yaw2 = self.car2.get_state()
        status2 = "已连接" if self.car2.connected else "未连接"
        self.car2_var.set(f"X: {x2:.3f}  Y: {y2:.3f}  Yaw: {math.degrees(yaw2):.1f}°  [{self.car2.packet_count}]  {status2}")

        # 计算差距
        if self.car1.connected and self.car2.connected:
            dx = x2 - x1
            dy = y2 - y1
            distance = math.sqrt(dx*dx + dy*dy)
            yaw_diff = math.degrees(yaw2 - yaw1)
            while yaw_diff > 180:
                yaw_diff -= 360
            while yaw_diff < -180:
                yaw_diff += 360
            self.diff_var.set(f"距离: {distance:.4f}m  角度差: {yaw_diff:.2f}°")

            # 记录数据
            if self.recorder.recording:
                self.recorder.record((x1, y1, yaw1), (x2, y2, yaw2))
                self._update_stats()

    def _update_stats(self):
        stats = self.recorder.get_statistics()
        if stats:
            self.stats_text.delete("1.0", tk.END)
            self.stats_text.insert(tk.END, f"记录数: {stats['count']}  时长: {stats['duration']:.1f}s\n\n")
            self.stats_text.insert(tk.END, f"距离差距:\n")
            self.stats_text.insert(tk.END, f"  平均: {stats['distance']['avg']:.4f}m\n")
            self.stats_text.insert(tk.END, f"  范围: {stats['distance']['min']:.4f} ~ {stats['distance']['max']:.4f}m\n")
            self.stats_text.insert(tk.END, f"  标准差: {stats['distance']['std']:.4f}m\n\n")
            self.stats_text.insert(tk.END, f"航向差距:\n")
            self.stats_text.insert(tk.END, f"  平均: {stats['yaw_diff']['avg']:.2f}°\n")
            self.stats_text.insert(tk.END, f"  范围: {stats['yaw_diff']['min']:.2f}° ~ {stats['yaw_diff']['max']:.2f}°\n")
            self.stats_text.insert(tk.END, f"  标准差: {stats['yaw_diff']['std']:.2f}°\n")


def main():
    root = tk.Tk()
    app = MultiCarAnalyzer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
