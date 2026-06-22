#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Track Plotter - Wireless UART real-time trajectory plotting tool
Features: drag pan, scroll zoom, hover coordinates, auto-save, throttled redraw
"""

import sys
import os
import re
import threading
import time
import math
import queue
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import matplotlib
matplotlib.use('TkAgg')
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC', 'Arial Unicode MS', 'DejaVu Sans']
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

SERIAL_PORT = 'COM24'
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# Regex
RE_SAVE_OLD = re.compile(r'^S:(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.?\d*)')
RE_SAVE_NEW = re.compile(r'^S:(\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.?\d*),(\d+),(\d+),(\d+)')
RE_FOLLOW_IDX = re.compile(r'^F:(\d+),(-?\d+\.\d+),(-?\d+\.\d+)(?:,(-?\d+\.\d+))?')
RE_DEBUG = re.compile(r'^D:(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)(?:,(-?\d+\.\d+))?')
RE_POSITION = re.compile(r'\((-?\d+\.\d+),(-?\d+\.\d+)(?:,(-?\d+\.\d+))?(?:,(\d+))?\)')
RE_STATE = re.compile(r'STATE:(IDLE|DRIVE|CHANGE|YAWCR|FOLLOW|ARRIVE|ERROR)')
# R: regex accepts both new 8-field and old 10-field (pre-split) format.
# New:  R:x,y,yaw,vf,x_enu,y_enu,yaw_tn[,ant_raw]         (8 fields)
# Old:  R:x,y,yaw,vf,sats,fix,sample,x_enu,y_enu,yaw_tn   (10 fields)
# Optional trailing groups allow both formats to match.
RE_RTK = re.compile(r'^R:(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(\d+),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)(?:,(-?\d+\.?\d*))?(?:,(-?\d+\.?\d*))?(?:,(-?\d+\.?\d*))?')
RE_RTK_DIAG = re.compile(r'^R:(\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(\d+),(\d+),(\d+)')
RE_GNSS = re.compile(r'^G:(\d+),(\d+),(\d+),(\d+)')
RE_INS = re.compile(r'^I:(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)')
RE_INS_DIAG = re.compile(r'^I:(\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(\d+),(\d+),(\d+),(\d+)')
# yawrtk frame: yawrtk:seq,tick_ms,phase,yaw_gyro,mag_yaw_rel,mag_yaw_corr_rel,yaw_fused,rtk_yaw_ins,rtk_yaw_ref,rtk_valid,mag_x,mag_y,mag_z
RE_YAWRTK = re.compile(r'^yawrtk:(\d+),(\d+),(\w+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)')

# Throttle interval (seconds) - max redraw rate
DRAW_THROTTLE = 0.10  # ~10 FPS max, smoother for full matplotlib redraws
RTK_DISPLAY_OFFSET_DEG = 4.3
RTK_RECENT_FULL_POINTS = 220
RTK_HISTORY_MAX_DRAW = 900
INS_RECENT_FULL_POINTS = 240
INS_HISTORY_MAX_DRAW = 900


class SerialReader(threading.Thread):
    def __init__(self, port, baud, callback):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.callback = callback
        self.running = False
        self.ser = None

    def run(self):
        self.running = True
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=SERIAL_TIMEOUT)
            print(f"[Serial] 已连接 {self.port} @ {self.baud}")
            buf = b""
            while self.running:
                if self.ser.in_waiting > 0:
                    buf += self.ser.read(self.ser.in_waiting)
                    while b'\n' in buf:
                        line_bytes, buf = buf.split(b'\n', 1)
                        line_bytes = line_bytes.strip()
                        if line_bytes:
                            line = line_bytes.decode('ascii', errors='ignore').strip()
                            if line:
                                self.callback(line)
                else:
                    time.sleep(0.01)
        except serial.SerialException as e:
            print(f"[Serial] 连接失败: {e}")
            self.callback(f"__ERROR__:{e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


class TrackPlotter(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("轨迹调试上位机 - 无线串口")
        self.geometry("1200x800")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Subject mode: 1 or 3
        self.subject_mode = tk.IntVar(value=1)
        self.rtk_angle_offset_var = tk.DoubleVar(value=RTK_DISPLAY_OFFSET_DEG)
        
        # 科目三 state machine
        self.subject3_state = "IDLE"  # IDLE, DRIVE, CHANGE, YAW_CORRECT, FOLLOW, ARRIVE, ERROR
        self.park_yaw = 0.0
        self.yaw_offset = 0.0
        
        self.mode = "idle"
        self.save_points = []
        self.follow_pts = []
        self.change_pts = []  # CHANGE phase trajectory points
        self.debug_pts = []  # (idx, x, y, pp_target_deg, actual_deg)
        self.rtk_pts = []    # RTK data: [(x_ins, y_ins, yaw_ins, valid_flags, phase, x_enu, y_enu, yaw_tn, ant_raw), ...]
        self.ins_pts = []    # INS data: [(x, y, yaw_gyro_deg, yaw_mag_deg, yaw_ekf_deg), ...]
        self.yawrtk_data = []  # yawrtk data: [(seq, tick, phase, yaw_gyro, mag_yaw_rel, mag_yaw_corr_rel, yaw_fused, rtk_yaw_ins, rtk_yaw_ref, rtk_valid, mag_x, mag_y, mag_z), ...]
        self.save_diag = []   # [(index, tick_ms, q_drop, ring_count, pending4), ...]
        self.ins_diag = []    # [(seq, tick_ms, ring_count, pending4, q_drop, q_depth), ...]
        self.rtk_diag = []    # [(seq, tick_ms, q_drop, ring_count, pending4), ...]
        # Pre-rotated RTK cache for drawing (computed once per new point)
        self._rtk_rotated = []  # [(x_ins_r, y_ins_r, yaw_ins_rad, vf, phase), ...]
        # GNSS status (from G: messages, 1Hz)
        self._gnss_sats = 0
        self._gnss_fix_quality = 0  # GGA fix quality: 0=invalid 1=GPS 2=DGPS 4=RTKfixed 5=RTKfloat
        self._gnss_fix = 0
        self._gnss_sample_pct = 0
        self._rtk_warning = ""
        self._ins_max_pts = 2000  # INS数据点上限，防止内存无限增长
        # Throttle timestamps
        self._last_detail_time = 0
        self._last_draw_time = 0
        self._last_legend_time = 0
        self._draw_pending = False
        self._redraw_active = False
        self._mag_lut = None  # dict: {bin_idx: correction_deg}
        self._mag_lut_bin_size = 1  # degrees per bin
        self._load_mag_lut()
        # Py-side ENU→INS alignment state (computed from yaw_true_north)
        self._rtk_align_x0 = None     # alignment ENU x reference
        self._rtk_align_y0 = None     # alignment ENU y reference
        self._rtk_align_yaw0 = None   # alignment yaw_true_north (θ₀)
        self._rtk_py_aligned = False  # whether Py-side alignment is done
        self.save_count = 0
        self.serial_reader = None
        self.is_connected = False
        self._line_queue = queue.Queue(maxsize=3000)
        self._queue_poll_active = False

        # Plot state
        self._drag_start = None
        self._hover_annot = None
        self._sidebar_bg = '#eef1f4'

        self._create_widgets()
        self._setup_plot_interaction()

    def _setup_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use('clam')
        except tk.TclError:
            pass

        base_font = ('Segoe UI', 9)
        title_font = ('Segoe UI', 10, 'bold')
        self.option_add('*Font', base_font)
        style.configure('TFrame', background=self._sidebar_bg)
        style.configure('Sidebar.TFrame', background=self._sidebar_bg)
        style.configure('TLabelframe', background=self._sidebar_bg, padding=(8, 6))
        style.configure('TLabelframe.Label', font=title_font, foreground='#263238', background=self._sidebar_bg)
        style.configure('TLabel', background=self._sidebar_bg, foreground='#263238')
        style.configure('状态.TLabel', background='#ffffff', foreground='#263238', padding=(6, 3))
        style.configure('Primary.TButton', padding=(6, 5))
        style.configure('TButton', padding=(6, 4))
        style.configure('TRadiobutton', background=self._sidebar_bg, foreground='#263238')
        style.configure('TEntry', padding=3)

    @staticmethod
    def _pack_x(widget, padx=8, pady=4):
        widget.pack(fill=tk.X, padx=padx, pady=pady)
        return widget

    def _create_widgets(self):
        self._setup_style()
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Scrollable left sidebar
        left_width = 330
        left_container = ttk.Frame(main_frame, width=left_width, style='Sidebar.TFrame')
        left_container.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        left_container.pack_propagate(False)

        canvas = tk.Canvas(left_container, width=left_width, highlightthickness=0, bg=self._sidebar_bg)
        scrollbar = ttk.Scrollbar(left_container, orient=tk.VERTICAL, command=canvas.yview)
        ctrl = ttk.Frame(canvas, style='Sidebar.TFrame')
        ctrl.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.create_window((0, 0), window=ctrl, anchor='nw', width=left_width - 8)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Mouse wheel scrolling for the sidebar only.  Keep the binding global so
        # wheel events still work when the pointer is over child widgets.
        def _is_sidebar_widget(widget):
            while widget is not None:
                if widget == left_container:
                    return True
                widget = getattr(widget, 'master', None)
            return False

        def _on_sidebar_mousewheel(event):
            widget = self.winfo_containing(event.x_root, event.y_root)
            if not _is_sidebar_widget(widget):
                return None
            if getattr(event, 'num', None) == 4:
                delta = -3
            elif getattr(event, 'num', None) == 5:
                delta = 3
            else:
                delta = int(-event.delta / 120)
                if delta == 0:
                    delta = -1 if event.delta > 0 else 1
            canvas.yview_scroll(delta, 'units')
            return 'break'

        self.bind_all('<MouseWheel>', _on_sidebar_mousewheel, add='+')
        self.bind_all('<Button-4>', _on_sidebar_mousewheel, add='+')
        self.bind_all('<Button-5>', _on_sidebar_mousewheel, add='+')

        # Serial
        sf = ttk.LabelFrame(ctrl, text="串口")
        sf.pack(fill=tk.X, padx=8, pady=(0, 8))
        sf.columnconfigure(1, weight=1)
        ttk.Label(sf, text="端口").grid(row=0, column=0, sticky=tk.W, padx=(0, 6), pady=3)
        self.port_var = tk.StringVar(value=SERIAL_PORT)
        ttk.Entry(sf, textvariable=self.port_var, width=12).grid(row=0, column=1, sticky=tk.EW, pady=3)
        ttk.Label(sf, text="波特率").grid(row=1, column=0, sticky=tk.W, padx=(0, 6), pady=3)
        self.baud_var = tk.StringVar(value=str(SERIAL_BAUD))
        ttk.Entry(sf, textvariable=self.baud_var, width=12).grid(row=1, column=1, sticky=tk.EW, pady=3)
        btnf = ttk.Frame(sf); btnf.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(6, 2))
        self.connect_btn = ttk.Button(btnf, text="连接端口", command=self._toggle_conn)
        self.connect_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.status_lbl = ttk.Label(sf, text="未连接", foreground="gray", style='状态.TLabel')
        self.status_lbl.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=(4, 0))

        # Subject Selection
        sf_mode = ttk.LabelFrame(ctrl, text="任务选择")
        sf_mode.pack(fill=tk.X, padx=8, pady=8)
        ttk.Radiobutton(sf_mode, text="科目一", variable=self.subject_mode,
                        value=1, command=self._on_subject_change).pack(side=tk.LEFT, padx=(0, 14), pady=2)
        ttk.Radiobutton(sf_mode, text="科目三", variable=self.subject_mode,
                        value=3, command=self._on_subject_change).pack(side=tk.LEFT, pady=2)

        # Mode - 科目一 (Simple)
        self.mf_subj1 = ttk.LabelFrame(ctrl, text="科目一控制")
        self.mf_subj1.pack(fill=tk.X, padx=8, pady=8)
        self.btn_start_save = ttk.Button(self.mf_subj1, text="开始存点", command=self._start_save, state=tk.DISABLED)
        self._pack_x(self.btn_start_save)
        self.btn_stop_save = ttk.Button(self.mf_subj1, text="停止存点", command=self._stop_save, state=tk.DISABLED)
        self._pack_x(self.btn_stop_save)
        self.btn_start_follow = ttk.Button(self.mf_subj1, text="开始循迹", command=self._start_follow, state=tk.DISABLED)
        self._pack_x(self.btn_start_follow)
        self.btn_stop_follow = ttk.Button(self.mf_subj1, text="停止循迹", command=self._stop_follow, state=tk.DISABLED)
        self._pack_x(self.btn_stop_follow)
        self.btn_back = ttk.Button(self.mf_subj1, text="返回存点模式", command=self._back_to_save, state=tk.DISABLED)
        self._pack_x(self.btn_back)

        # Mode - 科目三 (Maze 任务选择)
        self.mf_subj3 = ttk.LabelFrame(ctrl, text="科目三状态机")
        self.mf_subj3.pack(fill=tk.X, padx=8, pady=8)
        self.btn_subj3_drive = ttk.Button(self.mf_subj3, text="开始第一段记录", 
                                          command=self._subj3_start_drive, state=tk.DISABLED)
        self._pack_x(self.btn_subj3_drive)
        self.btn_subj3_change = ttk.Button(self.mf_subj3, text="进入调头校准", 
                                           command=self._subj3_to_change, state=tk.DISABLED)
        self._pack_x(self.btn_subj3_change)
        self.btn_subj3_follow = ttk.Button(self.mf_subj3, text="开始循迹 (FOLLOW)", 
                                           command=self._subj3_start_follow, state=tk.DISABLED)
        self._pack_x(self.btn_subj3_follow)
        self.btn_subj3_end_follow = ttk.Button(self.mf_subj3, text="结束循迹并保存",
                                                command=self._subj3_end_follow, state=tk.DISABLED)
        self._pack_x(self.btn_subj3_end_follow)
        self.btn_subj3_reset = ttk.Button(self.mf_subj3, text="重置并丢弃数据", 
                                          command=self._subj3_reset, state=tk.DISABLED)
        self._pack_x(self.btn_subj3_reset)

        # 状态
        stf = ttk.LabelFrame(ctrl, text="状态")
        stf.pack(fill=tk.X, padx=8, pady=8)
        self.subject_lbl = ttk.Label(stf, text="科目: 1", font=('Arial', 10, 'bold'), foreground='blue')
        self._pack_x(self.subject_lbl, pady=2)
        self.mode_lbl = ttk.Label(stf, text="模式: 空闲", font=('Arial', 10, 'bold'))
        self._pack_x(self.mode_lbl, pady=2)
        self.subj3_state_lbl = ttk.Label(stf, text="阶段: IDLE", font=('Arial', 9), foreground='gray')
        self._pack_x(self.subj3_state_lbl, pady=2)
        self.pts_lbl = ttk.Label(stf, text="存点: 0")
        self._pack_x(self.pts_lbl, pady=2)
        self.flw_lbl = ttk.Label(stf, text="循迹点: 0")
        self._pack_x(self.flw_lbl, pady=2)
        self.yaw_offset_lbl = ttk.Label(stf, text="航向偏移: 0.0度")
        self._pack_x(self.yaw_offset_lbl, pady=2)
        self.rtk_status_lbl = ttk.Label(stf, text="RTK: ---", font=('Arial', 9), foreground='gray')
        self._pack_x(self.rtk_status_lbl, pady=2)
        self.ins_status_lbl = ttk.Label(stf, text="INS: ---", font=('Arial', 9), foreground='gray')
        self._pack_x(self.ins_status_lbl, pady=2)
        self.yawrtk_status_lbl = ttk.Label(stf, text="yawrtk: ---", font=('Arial', 9), foreground='gray')
        self._pack_x(self.yawrtk_status_lbl, pady=2)

        # RTK详细信息 Panel — multi-line, not truncated by sidebar width
        rtkf = ttk.LabelFrame(ctrl, text="RTK详细信息")
        rtkf.pack(fill=tk.X, padx=8, pady=8)
        self.rtk_detail_text = tk.Text(rtkf, height=6, width=34, font=('Consolas', 9),
                                       state=tk.DISABLED, wrap=tk.WORD,
                                       bg='#101820', fg='#8df0b5',
                                       relief=tk.FLAT, padx=4, pady=3)
        self.rtk_detail_text.pack(fill=tk.X, padx=4, pady=4)

        # RTK display calibration
        cal_f = ttk.LabelFrame(ctrl, text="RTK显示校准")
        cal_f.pack(fill=tk.X, padx=8, pady=8)
        cal_row = ttk.Frame(cal_f)
        cal_row.pack(fill=tk.X, padx=4, pady=(4, 2))
        ttk.Label(cal_row, text="角度").pack(side=tk.LEFT)
        self.rtk_angle_offset_lbl = ttk.Label(cal_row, text=f"{RTK_DISPLAY_OFFSET_DEG:+.1f}度", width=9, anchor=tk.E)
        self.rtk_angle_offset_lbl.pack(side=tk.RIGHT)
        self.rtk_angle_offset_scale = ttk.Scale(
            cal_f,
            from_=-15.0,
            to=15.0,
            variable=self.rtk_angle_offset_var,
            command=self._on_rtk_angle_offset_change)
        self.rtk_angle_offset_scale.pack(fill=tk.X, padx=4, pady=(0, 4))
        ttk.Button(cal_f, text="重置角度", command=self._reset_rtk_angle_offset).pack(
            fill=tk.X, padx=4, pady=(0, 4))
        
        # Now update subject UI after all widgets are created
        self._update_subject_ui()

        # Stats
        saf = ttk.LabelFrame(ctrl, text="信息")
        saf.pack(fill=tk.X, padx=8, pady=8)
        self.stats_text = tk.Text(saf, height=5, width=32, state=tk.DISABLED, wrap=tk.WORD,
                                  bg='#ffffff', fg='#263238', relief=tk.FLAT, padx=6, pady=5)
        self.stats_text.pack(padx=4, pady=4, fill=tk.X)

        # File
        ff = ttk.LabelFrame(ctrl, text="数据与视图")
        ff.pack(fill=tk.X, padx=8, pady=8)
        self._pack_x(ttk.Button(ff, text="加载最新数据", command=self._load_latest_data))
        self._pack_x(ttk.Button(ff, text="打开数据文件", command=self._open_data_file))
        self._pack_x(ttk.Button(ff, text="保存轨迹图片", command=self._save_plot_png))
        self._pack_x(ttk.Button(ff, text="导出存点数据", command=self._export_save))
        self._pack_x(ttk.Button(ff, text="导出循迹数据", command=self._export_follow))
        self._pack_x(ttk.Button(ff, text="重置视图", command=self._reset_view))

        # Plot
        pf2 = ttk.Frame(main_frame)
        pf2.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor='#f7f9fb')
        self.ax = self.fig.add_subplot(111)
        self._init_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=pf2)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _init_axes(self):
        self.ax.set_xlabel('X / 前向 (m)', fontsize=11)
        self.ax.set_ylabel('Y / 左向 (m)', fontsize=11)
        self.ax.set_title('车辆轨迹', fontsize=13, fontweight='bold')
        self.ax.grid(True, linestyle='--', alpha=0.4, color='#888888')
        self.ax.set_box_aspect(1)
        self.ax.set_facecolor('#fafafa')
        self.ax.axhline(0, color='#cccccc', linewidth=0.8, linestyle='-')
        self.ax.axvline(0, color='#cccccc', linewidth=0.8, linestyle='-')

    def _setup_plot_interaction(self):
        self.fig.canvas.mpl_connect('button_press_event', self._on_press)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_drag_mpl)
        self.fig.canvas.mpl_connect('button_release_event', self._on_release)
        self.fig.canvas.mpl_connect('scroll_event', self._on_scroll_mpl)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_hover)

    def _on_press(self, event):
        if event.inaxes == self.ax and event.button == 1:
            self._drag_start = (event.x, event.y, self.ax.get_xlim(), self.ax.get_ylim())

    def _on_drag_mpl(self, event):
        if self._drag_start is None:
            return
        if event.x is None or event.y is None:
            return
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        inv = self.ax.transData.inverted()
        x0, y0 = inv.transform((0, 0))
        x1, y1 = inv.transform((dx, dy))
        ddata_x = x1 - x0
        ddata_y = y1 - y0
        orig_xlim = self._drag_start[2]
        orig_ylim = self._drag_start[3]
        self.ax.set_xlim(orig_xlim[0] - ddata_x, orig_xlim[1] - ddata_x)
        self.ax.set_ylim(orig_ylim[0] - ddata_y, orig_ylim[1] - ddata_y)
        self.canvas.draw_idle()

    def _on_release(self, event):
        self._drag_start = None

    def _on_scroll_mpl(self, event):
        if event.inaxes != self.ax:
            return
        scale = 0.8 if event.button == 'up' else 1.25
        self._zoom_at(event.xdata, event.ydata, scale)

    def _zoom_at(self, cx, cy, scale):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        new_w = (xlim[1] - xlim[0]) * scale
        new_h = (ylim[1] - ylim[0]) * scale
        rx = (cx - xlim[0]) / (xlim[1] - xlim[0]) if xlim[1] != xlim[0] else 0.5
        ry = (cy - ylim[0]) / (ylim[1] - ylim[0]) if ylim[1] != ylim[0] else 0.5
        self.ax.set_xlim(cx - new_w * rx, cx + new_w * (1 - rx))
        self.ax.set_ylim(cy - new_h * ry, cy + new_h * (1 - ry))
        self.canvas.draw_idle()

    def _on_hover(self, event):
        """Hover tooltip with throttling to avoid excessive redraws."""
        # Throttle hover to max 10Hz
        now = time.time()
        if now - getattr(self, '_last_hover_time', 0) < 0.1:
            return
        self._last_hover_time = now

        if event.inaxes != self.ax:
            if self._hover_annot and self._hover_annot.get_visible():
                self._hover_annot.set_visible(False)
                self.canvas.draw_idle()
            return

        # Only check save_points and follow_pts (not RTK - too many)
        all_pts = []
        for p in self.save_points:
            all_pts.append((p[1], p[2], f"#{p[0]}"))
        for i, p in enumerate(self.follow_pts):
            label = f"F#{p[0]}" if p[0] > 0 else f"F{i+1}"
            all_pts.append((p[1], p[2], label))
        if not all_pts:
            return

        ex, ey = event.xdata, event.ydata
        if ex is None or ey is None:
            return

        # Quick distance check in data space first (much faster than pixel transform)
        nearest = None
        min_dist = 20
        for px, py, label in all_pts:
            pix = self.ax.transData.transform((px, py))
            dist = ((pix[0] - event.x) ** 2 + (pix[1] - event.y) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                nearest = (px, py, label)

        if nearest:
            px, py, label = nearest
            text = f"{label}\n({px:.3f}, {py:.3f})"
            if self._hover_annot is None:
                self._hover_annot = self.ax.annotate(
                    text, (px, py), textcoords="offset points",
                    xytext=(15, 15), fontsize=9, color='#333333',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffffcc', alpha=0.9, edgecolor='#999999'),
                    arrowprops=dict(arrowstyle='->', color='#666666'), zorder=100)
            else:
                self._hover_annot.xy = (px, py)
                self._hover_annot.set_text(text)
                self._hover_annot.set_visible(True)
            self.canvas.draw_idle()
        else:
            if self._hover_annot and self._hover_annot.get_visible():
                self._hover_annot.set_visible(False)
                self.canvas.draw_idle()

    def _reset_view(self):
        self._auto_fit()
        self.canvas.draw()

    def _auto_fit(self):
        xs, ys = [], []
        for p in self.save_points:
            xs.append(p[1]); ys.append(p[2])
        for p in self.follow_pts:
            xs.append(p[1]); ys.append(p[2])
        # Include displayed RTK points in auto-fit bounds
        for p in self._get_rtk_display_points():
            if p[3] & 1:
                xs.append(p[0]); ys.append(p[1])
        if xs and ys:
            pad = 0.5
            xmin, xmax = min(xs) - pad, max(xs) + pad
            ymin, ymax = min(ys) - pad, max(ys) + pad
            xr = xmax - xmin
            yr = ymax - ymin
            if xr > yr:
                mid = (ymin + ymax) / 2
                ymin = mid - xr / 2
                ymax = mid + xr / 2
            else:
                mid = (xmin + xmax) / 2
                xmin = mid - yr / 2
                xmax = mid + yr / 2
            self.ax.set_xlim(xmin, xmax)
            self.ax.set_ylim(ymin, ymax)
        else:
            self.ax.set_xlim(-2, 2)
            self.ax.set_ylim(-2, 2)

    # ---- Serial ----
    def _toggle_conn(self):
        if self.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("错误", "波特率必须是数字")
            return
        self.serial_reader = SerialReader(port, baud, self._enqueue_data)
        self.serial_reader.start()
        self._start_queue_poll()
        self.after(500, self._check_conn)

    def _check_conn(self):
        if self.serial_reader and self.serial_reader.running:
            if self.serial_reader.ser and self.serial_reader.ser.is_open:
                self.is_connected = True
                self.connect_btn.config(text="断开端口")
                self.status_lbl.config(text="已连接", foreground="green")
                self._enable_initial_btns()
                return
        self.is_connected = False
        self.status_lbl.config(text="连接失败", foreground="red")

    def _disconnect(self):
        if self.serial_reader:
            self.serial_reader.stop()
            self.serial_reader = None
        self._queue_poll_active = False
        self.is_connected = False
        self.connect_btn.config(text="连接端口")
        self.status_lbl.config(text="未连接", foreground="gray")
        self._disable_btns()

    def _disable_btns(self):
        self.btn_start_save.config(state=tk.DISABLED)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.DISABLED)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        # 科目三 buttons
        self.btn_subj3_drive.config(state=tk.DISABLED)
        self.btn_subj3_change.config(state=tk.DISABLED)
        self.btn_subj3_follow.config(state=tk.DISABLED)
        self.btn_subj3_end_follow.config(state=tk.DISABLED)
        self.btn_subj3_reset.config(state=tk.DISABLED)

    # ---- Subject Selection ----
    def _on_subject_change(self):
        """Update UI when subject mode changes"""
        self._update_subject_ui()
        self._reset_state()
        
    def _update_subject_ui(self):
        """Update UI display based on subject mode"""
        mode = self.subject_mode.get()
        if mode == 1:
            # 科目一: show simple controls, hide 科目三
            self.mf_subj1.pack(fill=tk.X, padx=5, pady=5)
            self.mf_subj3.pack_forget()
            self.subject_lbl.config(text="科目: 1", foreground='blue')
            self.subj3_state_lbl.pack_forget()
            self.yaw_offset_lbl.pack_forget()
        else:
            # 科目三: hide simple controls, show 科目三
            self.mf_subj1.pack_forget()
            self.mf_subj3.pack(fill=tk.X, padx=5, pady=5)
            self.subject_lbl.config(text="科目: 3", foreground='purple')
            self.subj3_state_lbl.pack(padx=5, pady=2, anchor=tk.W)
            self.yaw_offset_lbl.pack(padx=5, pady=2, anchor=tk.W)
        
        # Re-enable buttons based on connection state
        if self.is_connected:
            self._enable_initial_btns()
    
    def _enable_initial_btns(self):
        """Enable initial buttons after connection"""
        if self.subject_mode.get() == 1:
            self.btn_start_save.config(state=tk.NORMAL)
        else:
            self.btn_subj3_drive.config(state=tk.NORMAL)
    
    def _reset_state(self):
        """Reset all states"""
        self.mode = "idle"
        self.subject3_state = "IDLE"
        self.park_yaw = 0.0
        self.yaw_offset = 0.0
        self.save_points.clear()
        self.follow_pts.clear()
        self.change_pts.clear()
        self.debug_pts.clear()
        self.rtk_pts.clear()
        self.ins_pts.clear()
        self.yawrtk_data.clear()
        self.save_diag.clear()
        self.ins_diag.clear()
        self.rtk_diag.clear()
        self._rtk_rotated.clear()
        self._rtk_warning = ""
        self._update_subj3_state_display()
        self._clear_plot()
        
    # ---- 科目三 State Machine ----
    def _subj3_start_drive(self):
        """科目三: Start manual drive + recording"""
        self.subject3_state = "DRIVE"
        self.mode = "save"
        self.save_points.clear()
        self.follow_pts.clear()
        self.change_pts.clear()
        self.rtk_pts.clear()
        self.ins_pts.clear()
        self.yawrtk_data.clear()
        self.save_diag.clear()
        self.ins_diag.clear()
        self.rtk_diag.clear()
        self._rtk_rotated.clear()
        self._rtk_warning = ""
        self._rtk_warning = ""
        self._rtk_py_aligned = False
        self._rtk_align_x0 = None
        self._rtk_align_y0 = None
        self._rtk_align_yaw0 = None
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="模式: 存点中", foreground="blue")
        self._clear_plot()
        
    def _subj3_to_change(self):
        """科目三: Enter CHANGE phase - remote control to rotate car head"""
        # Record the last point's yaw as parking yaw
        if self.save_points:
            self.park_yaw = self.save_points[-1][3]  # (idx, x, y, yaw, speed_dir)
        
        self.subject3_state = "CHANGE"
        self.mode = "change"  # Real-time mode: receive and display coordinates
        self.change_pts.clear()
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="模式: 遥控调头", foreground="orange")
        self._auto_save_points_subj3("drive")
        # Draw saved trajectory as reference, then overlay change trajectory
        self._clear_plot()
        if self.save_points:
            self._draw_save_ref()
        
    def _subj3_start_follow(self):
        """科目三: Start reverse follow (from CHANGE phase)"""
        self.subject3_state = "FOLLOW"
        self.mode = "follow"
        self.follow_pts.clear()
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="模式: 循迹中", foreground="red")
        self._clear_plot()
        if self.save_points:
            self._draw_save_ref()
        # Yaw offset will be calculated from first car position yaw when data arrives
        # (in _process(), we update self.yaw_offset from first idx=0 point's yaw - park_yaw)
    
    def _subj3_end_follow(self):
        """科目三: End follow, save data, and go to IDLE"""
        # Analyze and save follow data
        if self.follow_pts:
            self._analyze_and_save_subj3()
        
        self.subject3_state = "IDLE"
        self.mode = "idle"
        self.park_yaw = 0.0
        self.yaw_offset = 0.0
        self.yaw_offset_lbl.config(text="航向偏移: 0.0度")
        self.save_points.clear()
        self.follow_pts.clear()
        self.change_pts.clear()
        self.save_diag.clear()
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="模式: 空闲", foreground="black")
        self._clear_plot()
            
    def _subj3_reset(self):
        """科目三: Reset to IDLE state (discard all data)"""
        # Reset discards all data - do NOT save

        self.subject3_state = "IDLE"
        self.mode = "idle"
        self.park_yaw = 0.0
        self.yaw_offset = 0.0
        self.yaw_offset_lbl.config(text="航向偏移: 0.0度")
        self.save_points.clear()
        self.follow_pts.clear()
        self.change_pts.clear()
        self.rtk_pts.clear()
        self.ins_pts.clear()
        self.yawrtk_data.clear()
        self.save_diag.clear()
        self.ins_diag.clear()
        self.rtk_diag.clear()
        self._rtk_rotated.clear()
        self._rtk_warning = ""
        self._rtk_warning = ""
        self._rtk_py_aligned = False
        self._rtk_align_x0 = None
        self._rtk_align_y0 = None
        self._rtk_align_yaw0 = None
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="模式: 空闲", foreground="black")
        self._clear_plot()
        
    def _update_subj3_btns(self):
        """Update 科目三 button states"""
        state = self.subject3_state
        
        # Reset all buttons
        self.btn_subj3_drive.config(state=tk.DISABLED)
        self.btn_subj3_change.config(state=tk.DISABLED)
        self.btn_subj3_follow.config(state=tk.DISABLED)
        self.btn_subj3_end_follow.config(state=tk.DISABLED)
        self.btn_subj3_reset.config(state=tk.DISABLED)
        
        if state == "IDLE":
            self.btn_subj3_drive.config(state=tk.NORMAL)
        elif state == "DRIVE":
            self.btn_subj3_change.config(state=tk.NORMAL)
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "CHANGE":
            self.btn_subj3_follow.config(state=tk.NORMAL)
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "FOLLOW":
            self.btn_subj3_end_follow.config(state=tk.NORMAL)
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "ARRIVE":
            self.btn_subj3_end_follow.config(state=tk.NORMAL)
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "ERROR":
            self.btn_subj3_reset.config(state=tk.NORMAL)
            
    def _update_subj3_state_display(self):
        """Update 科目三 state display"""
        state_colors = {
            "IDLE": "gray",
            "DRIVE": "blue",
            "CHANGE": "orange",
            "YAW_CORRECT": "purple",
            "FOLLOW": "red",
            "ARRIVE": "green",
            "ERROR": "red"
        }
        color = state_colors.get(self.subject3_state, "gray")
        self.subj3_state_lbl.config(text=f"阶段: {self.subject3_state}", foreground=color)

    # ---- Data ----
    def _enqueue_data(self, line):
        try:
            self._line_queue.put_nowait(line)
        except queue.Full:
            try:
                self._line_queue.get_nowait()
                self._line_queue.put_nowait(line)
            except queue.Empty:
                pass

    def _start_queue_poll(self):
        if self._queue_poll_active:
            return
        self._queue_poll_active = True
        self.after(20, self._process_line_queue)

    def _process_line_queue(self):
        if not self._queue_poll_active:
            return
        processed = 0
        while processed < 200:
            try:
                line = self._line_queue.get_nowait()
            except queue.Empty:
                break
            self._on_data(line)
            processed += 1
        self.after(20, self._process_line_queue)

    def _on_data(self, line):
        self.after(0, self._process, line)

    def _process(self, line):
        if line.startswith("__ERROR__"):
            self.status_lbl.config(text=f"错误: {line[9:]}", foreground="red")
            self.is_connected = False
            self.connect_btn.config(text="连接端口")
            return

        if line.startswith('S:'):
            matched_new = False
            for m in RE_SAVE_NEW.finditer(line):
                try:
                    matched_new = True
                    idx = int(m.group(1))
                    tick_ms = int(m.group(2))
                    x = float(m.group(3))
                    y = float(m.group(4))
                    yaw = float(m.group(5))
                    sd = float(m.group(6))
                    q_drop = int(m.group(7))
                    ring_count = int(m.group(8))
                    pending4 = int(m.group(9))
                    self.save_points.append((idx, x, y, yaw, sd))
                    self.save_diag.append((idx, tick_ms, q_drop, ring_count, pending4))
                    self.pts_lbl.config(text=f"存点: {len(self.save_points)} | tick={tick_ms}ms 丢包={q_drop}")
                    if self.mode == "save":
                        self._request_draw()
                except Exception as e:
                    print(f"[S] Parse error: {e} | raw: {line}")
            if matched_new:
                return
            for m in RE_SAVE_OLD.finditer(line):
                try:
                    idx = int(m.group(1))
                    x = float(m.group(2))
                    y = float(m.group(3))
                    yaw = float(m.group(4))
                    sd = float(m.group(5))
                    self.save_points.append((idx, x, y, yaw, sd))
                    self.pts_lbl.config(text=f"存点: {len(self.save_points)}")
                    if self.mode == "save":
                        self._request_draw()
                except:
                    pass

        if line.startswith('F:'):
            for m in RE_FOLLOW_IDX.finditer(line):
                try:
                    idx = int(m.group(1))
                    x = float(m.group(2))
                    y = float(m.group(3))
                    yaw = float(m.group(4)) if m.group(4) else 0.0
                    self.follow_pts.append((idx, x, y, yaw, idx))
                    self.flw_lbl.config(text=f"循迹点: {len(self.follow_pts)} (idx {idx})")
                    if self.mode == "follow":
                        self._request_draw()
                except:
                    pass

        # D:idx,x,y,pp_target,actual,yaw (steer debug)
        if self.mode == 'follow' and line.startswith('D:'):
            for m in RE_DEBUG.finditer(line):
                try:
                    idx = int(m.group(1))
                    x = float(m.group(2))
                    y = float(m.group(3))
                    pp_tgt = float(m.group(4))
                    actual = float(m.group(5))
                    yaw = float(m.group(6)) if m.group(6) else 0.0
                    self.follow_pts.append((idx, x, y, yaw, idx))
                    self.debug_pts.append((idx, x, y, pp_tgt, actual))
                    self.flw_lbl.config(text=f"循迹点: {len(self.follow_pts)} (idx {idx}) | 目标:{pp_tgt:.1f} 实际:{actual:.1f}")
                    self._request_draw()
                except:
                    pass

        if self.mode == 'follow' and '(' in line and ')' in line:
            for m in RE_POSITION.finditer(line):
                try:
                    x = float(m.group(1))
                    y = float(m.group(2))
                    yaw = float(m.group(3)) if m.group(3) else 0.0
                    target_idx = int(m.group(4)) if m.group(4) else 0
                    if abs(x) < 1e-6 and abs(y) < 1e-6:
                        continue
                    if self.follow_pts and x == self.follow_pts[-1][1] and y == self.follow_pts[-1][2]:
                        continue
                    # Calculate yaw offset from first car position
                    if not any(p[0] == 0 for p in self.follow_pts):
                        self.yaw_offset = yaw - self.park_yaw
                        self.yaw_offset_lbl.config(text=f"航向偏移: {np.degrees(self.yaw_offset):.1f}度")
                    self.follow_pts.append((0, x, y, yaw, target_idx))
                    if target_idx > 0:
                        self.flw_lbl.config(text=f"循迹点: {len(self.follow_pts)} | 目标: {target_idx}")
                    else:
                        self.flw_lbl.config(text=f"循迹点: {len(self.follow_pts)}")
                    self._request_draw()
                except:
                    pass

        # CHANGE mode: receive real-time position during remote control rotation
        if self.mode == 'change' and '(' in line and ')' in line:
            for m in RE_POSITION.finditer(line):
                try:
                    x = float(m.group(1))
                    y = float(m.group(2))
                    yaw = float(m.group(3)) if m.group(3) else 0.0
                    if abs(x) < 1e-6 and abs(y) < 1e-6:
                        continue
                    if self.change_pts and x == self.change_pts[-1][1] and y == self.change_pts[-1][2]:
                        continue
                    self.change_pts.append((0, x, y, yaw, 0))
                    self.flw_lbl.config(text=f"调头点: {len(self.change_pts)}")
                    self._request_draw()
                except:
                    pass

        if "Track信息:TotalPoints=" in line:
            try:
                self.pts_lbl.config(text=f"存点: {line.split('=')[1]}")
            except:
                pass

        # INS数据解析：兼容旧格式 I:x,y,yaw_gyro,yaw_mag,yaw_ekf
        # 新格式 I:seq,tick_ms,x,y,yaw_gyro,yaw_mag,yaw_ekf,ring_count,pending4,q_drop,q_depth
        if line.startswith('I:'):
            matched_diag = False
            for m in RE_INS_DIAG.finditer(line):
                try:
                    matched_diag = True
                    seq = int(m.group(1))
                    tick_ms = int(m.group(2))
                    ins_x = float(m.group(3))
                    ins_y = float(m.group(4))
                    yaw_gyro = float(m.group(5))
                    yaw_mag = float(m.group(6))
                    yaw_ekf = float(m.group(7))
                    ring_count = int(m.group(8))
                    pending4 = int(m.group(9))
                    q_drop = int(m.group(10))
                    q_depth = int(m.group(11))
                    if self.ins_pts and abs(ins_x - self.ins_pts[-1][0]) < 1e-4 and abs(ins_y - self.ins_pts[-1][1]) < 1e-4:
                        self.ins_pts[-1] = (ins_x, ins_y, yaw_gyro, yaw_mag, yaw_ekf)
                        if self.ins_diag:
                            self.ins_diag[-1] = (seq, tick_ms, ring_count, pending4, q_drop, q_depth)
                    else:
                        self.ins_pts.append((ins_x, ins_y, yaw_gyro, yaw_mag, yaw_ekf))
                        self.ins_diag.append((seq, tick_ms, ring_count, pending4, q_drop, q_depth))
                    if len(self.ins_pts) > self._ins_max_pts:
                        self.ins_pts = self.ins_pts[-self._ins_max_pts:]
                        self.ins_diag = self.ins_diag[-self._ins_max_pts:]
                    yaw_mag_lut = self._apply_mag_lut(yaw_mag)
                    lut_tag = f" lut={yaw_mag_lut:.1f}°" if self._mag_lut else ""
                    self.ins_status_lbl.config(
                        text=f"INS: seq={seq} tick={tick_ms}ms gyro={yaw_gyro:.1f}° mag={yaw_mag:.1f}° ekf={yaw_ekf:.1f}°{lut_tag} 丢包={q_drop} 积压={pending4}",
                        foreground='blue')
                    if self.mode in ("follow", "change", "save"):
                        self._request_draw()
                except Exception as e:
                    print(f"[INS] Parse error: {e} | raw: {line}")
            if matched_diag:
                return
            for m in RE_INS.finditer(line):
                try:
                    ins_x = float(m.group(1))
                    ins_y = float(m.group(2))
                    yaw_gyro = float(m.group(3))
                    yaw_mag = float(m.group(4))
                    yaw_ekf = float(m.group(5))
                    # 去重：与上一个点坐标相同则只更新yaw
                    if self.ins_pts and abs(ins_x - self.ins_pts[-1][0]) < 1e-4 and abs(ins_y - self.ins_pts[-1][1]) < 1e-4:
                        self.ins_pts[-1] = (ins_x, ins_y, yaw_gyro, yaw_mag, yaw_ekf)
                        continue
                    self.ins_pts.append((ins_x, ins_y, yaw_gyro, yaw_mag, yaw_ekf))
                    # 限制数据点数量
                    if len(self.ins_pts) > self._ins_max_pts:
                        self.ins_pts = self.ins_pts[-self._ins_max_pts:]
                    # 更新状态栏 - 显示三组yaw: 原始磁力计, 校准后磁力计, LUT修正后
                    yaw_mag_lut = self._apply_mag_lut(yaw_mag)
                    lut_tag = f" lut={yaw_mag_lut:.1f}\u00b0" if self._mag_lut else ""
                    self.ins_status_lbl.config(
                        text=f"INS: ({ins_x:.2f},{ins_y:.2f}) gyro={yaw_gyro:.1f}° mag={yaw_mag:.1f}° ekf={yaw_ekf:.1f}°{lut_tag}",
                        foreground='blue')
                    if self.mode in ("follow", "change", "save"):
                        self._request_draw()
                except Exception as e:
                    print(f"[INS] Parse error: {e} | raw: {line}")

        # yawrtk 数据解析：yawrtk:seq,tick_ms,phase,yaw_gyro,mag_yaw_rel,mag_yaw_corr_rel,yaw_fused,rtk_yaw_ins,rtk_yaw_ref,rtk_valid,mag_x,mag_y,mag_z
        if line.startswith('yawrtk:'):
            for m in RE_YAWRTK.finditer(line):
                try:
                    yr_seq = int(m.group(1))
                    yr_tick = int(m.group(2))
                    yr_phase = m.group(3)
                    yr_yaw_gyro = float(m.group(4))
                    yr_mag_yaw_rel = float(m.group(5))
                    yr_mag_yaw_corr_rel = float(m.group(6))
                    yr_yaw_fused = float(m.group(7))
                    yr_rtk_yaw_ins = float(m.group(8))
                    yr_rtk_yaw_ref = float(m.group(9))
                    yr_rtk_valid = int(m.group(10))
                    yr_mag_x = float(m.group(11))
                    yr_mag_y = float(m.group(12))
                    yr_mag_z = float(m.group(13))
                    # 存储到 yawrtk 数据列表
                    self.yawrtk_data.append((yr_seq, yr_tick, yr_phase, yr_yaw_gyro,
                                             yr_mag_yaw_rel, yr_mag_yaw_corr_rel, yr_yaw_fused,
                                             yr_rtk_yaw_ins, yr_rtk_yaw_ref, yr_rtk_valid,
                                             yr_mag_x, yr_mag_y, yr_mag_z))
                    # 限制数据点数量
                    if len(self.yawrtk_data) > 5000:
                        self.yawrtk_data = self.yawrtk_data[-5000:]
                    # 更新状态栏 - 显示 yawrtk 信息
                    self.yawrtk_status_lbl.config(
                        text=f"yawrtk: seq={yr_seq} phase={yr_phase} gyro={yr_yaw_gyro:.1f}\u00b0 mag={yr_mag_yaw_rel:.1f}\u00b0 corr={yr_mag_yaw_corr_rel:.1f}\u00b0 fused={yr_yaw_fused:.1f}\u00b0 rtk={yr_rtk_yaw_ins:.1f}\u00b0 v={yr_rtk_valid}",
                        foreground='purple')
                    # 更新科目三阶段
                    phase_map = {'IDLE': 'IDLE', 'DRIVE': 'DRIVE', 'CHANG': 'CHANGE',
                                 'YAWCR': 'YAW_CORRECT', 'FOLLO': 'FOLLOW', 'ARRIV': 'ARRIVE', 'ERROR': 'ERROR'}
                    if yr_phase in phase_map:
                        self.subject3_state = phase_map[yr_phase]
                except Exception as e:
                    print(f"[YAWRTK] Parse error: {e} | raw: {line}")

        # RTK位置数据解析（快，约20Hz）：兼容新旧格式
        # New: R:x,y,yaw,vf,x_enu,y_enu,yaw_tn[,ant_raw]
        # Old: R:x,y,yaw,vf,sats,fix,sample,x_enu,y_enu,yaw_tn
        if line.startswith('R:'):
            matched_diag = False
            for m in RE_RTK_DIAG.finditer(line):
                try:
                    matched_diag = True
                    rtk_seq = int(m.group(1))
                    rtk_tick = int(m.group(2))
                    rtk_x = float(m.group(3))
                    rtk_y = float(m.group(4))
                    rtk_yaw = float(m.group(5))
                    rtk_valid = int(m.group(6))
                    rtk_x_enu = float(m.group(7))
                    rtk_y_enu = float(m.group(8))
                    rtk_yaw_tn = float(m.group(9))
                    rtk_ant_raw = float(m.group(10))
                    q_drop = int(m.group(11))
                    ring_count = int(m.group(12))
                    pending4 = int(m.group(13))
                    while rtk_yaw > 180: rtk_yaw -= 360
                    while rtk_yaw < -180: rtk_yaw += 360
                    while rtk_yaw_tn > 180: rtk_yaw_tn -= 360
                    while rtk_yaw_tn < -180: rtk_yaw_tn += 360
                    if self.subject_mode.get() == 3:
                        phase = self.subject3_state
                    else:
                        phase = self.mode.upper() if self.mode in ("save", "follow", "change") else "IDLE"
                    rtk_new = (rtk_x, rtk_y, rtk_yaw, rtk_valid, phase, rtk_x_enu, rtk_y_enu, rtk_yaw_tn, rtk_ant_raw)
                    rtk_diag_new = (rtk_seq, rtk_tick, q_drop, ring_count, pending4)
                    is_new_pos = True
                    if self.rtk_pts:
                        dx_enu = rtk_x_enu - self.rtk_pts[-1][5]
                        dy_enu = rtk_y_enu - self.rtk_pts[-1][6]
                        if (dx_enu*dx_enu + dy_enu*dy_enu) < 0.0001:
                            is_new_pos = False
                            self.rtk_pts[-1] = rtk_new
                            if self.rtk_diag:
                                self.rtk_diag[-1] = rtk_diag_new
                    if is_new_pos:
                        self.rtk_pts.append(rtk_new)
                        self.rtk_diag.append(rtk_diag_new)
                    if not self._rtk_py_aligned and (rtk_valid & 7) == 7:
                        self._rtk_align_x0 = rtk_x_enu
                        self._rtk_align_y0 = rtk_y_enu
                        self._rtk_align_yaw0 = rtk_yaw_tn
                        self._rtk_py_aligned = True
                        print(f"[RTK] Py-side aligned: x0={rtk_x_enu:.3f}, y0={rtk_y_enu:.3f}, yaw0(vehicle)={rtk_yaw_tn:.1f}°, ant_raw={rtk_ant_raw:.1f}°")
                    pos_valid = (rtk_valid & 1) != 0
                    yaw_valid = (rtk_valid & 2) != 0
                    aligned = (rtk_valid & 4) != 0
                    status_parts = ['POS' if pos_valid else '---',
                                   'YAW' if yaw_valid else '---',
                                   'ALN' if aligned else '---']
                    self.rtk_status_lbl.config(
                        text=f"RTK: {'|'.join(status_parts)} seq={rtk_seq} tick={rtk_tick}ms 丢包={q_drop}",
                        foreground='green' if aligned else ('orange' if pos_valid else 'gray'))
                    now = time.time()
                    if now - self._last_detail_time > 0.25:
                        self._last_detail_time = now
                        lines = [
                            f"位置: INS({rtk_x:.2f},{rtk_y:.2f})" if aligned else f"位置: ENU({rtk_x_enu:.3f},{rtk_y_enu:.3f})",
                            f"航向: {rtk_yaw_tn:7.1f} deg",
                            f"天线: {rtk_ant_raw:7.1f} deg",
                            f"诊断: seq={rtk_seq} tick={rtk_tick}ms drop={q_drop}",
                            f"积压: ring={ring_count} pending4={pending4}",
                            f"GNSS: Sats={self._gnss_sats} Fix={self._gnss_fix_quality} Org={self._gnss_sample_pct}%",
                        ]
                        self._update_rtk_detail('\n'.join(lines))
                    if self._rtk_py_aligned and (rtk_valid & 1) and is_new_pos:
                        self._add_rtk_rotated(rtk_x, rtk_y, rtk_yaw, rtk_valid, phase)
                        if self.mode in ("follow", "change", "save"):
                            self._request_draw()
                except Exception as e:
                    print(f"[RTK] Parse error: {e} | raw: {line}")
            if matched_diag:
                return
            for m in RE_RTK.finditer(line):
                try:
                    rtk_x = float(m.group(1))
                    rtk_y = float(m.group(2))
                    rtk_yaw = float(m.group(3))
                    rtk_valid = int(m.group(4))
                    f5 = m.group(5)
                    f6 = m.group(6) if m.group(6) else '0'
                    f7 = m.group(7) if m.group(7) else '0'
                    f8 = m.group(8) if m.group(8) else '0'
                    # Detect format: if f5 has no dot, it's old 10-field (sats=int)
                    if '.' not in f5:
                        # Old format: f5=sats, f6=fix, f7=sample, f8=x_enu, f9=y_enu, f10=yaw_tn
                        rtk_x_enu = float(f8)
                        rtk_y_enu = float(m.group(9) or '0')
                        rtk_yaw_tn = float(m.group(10) or '0')
                        rtk_ant_raw = 0.0
                    else:
                        # New format: f5=x_enu, f6=y_enu, f7=yaw_tn[, f8=ant_raw]
                        rtk_x_enu = float(f5)
                        rtk_y_enu = float(f6)
                        rtk_yaw_tn = float(f7)
                        rtk_ant_raw = float(f8)
                    # 归一化 yaw 到 [-180, 180]（与 INS 角度一致）
                    while rtk_yaw > 180: rtk_yaw -= 360
                    while rtk_yaw < -180: rtk_yaw += 360
                    while rtk_yaw_tn > 180: rtk_yaw_tn -= 360
                    while rtk_yaw_tn < -180: rtk_yaw_tn += 360
                    # Determine current phase from subject3_state or mode
                    if self.subject_mode.get() == 3:
                        phase = self.subject3_state
                    else:
                        phase = self.mode.upper() if self.mode in ("save", "follow", "change") else "IDLE"
                    # 去重: ENU位置变化<1cm视为未移动，只更新yaw，不新增点
                    rtk_new = (rtk_x, rtk_y, rtk_yaw, rtk_valid, phase, rtk_x_enu, rtk_y_enu, rtk_yaw_tn, rtk_ant_raw)
                    is_new_pos = True
                    if self.rtk_pts:
                        dx_enu = rtk_x_enu - self.rtk_pts[-1][5]
                        dy_enu = rtk_y_enu - self.rtk_pts[-1][6]
                        if (dx_enu*dx_enu + dy_enu*dy_enu) < 0.0001:  # <1cm
                            is_new_pos = False
                            self.rtk_pts[-1] = rtk_new  # update yaw
                    if is_new_pos:
                        self.rtk_pts.append(rtk_new)
                    # Py-side ENU→INS alignment: 等 C 端对齐完成（bit2=is_aligned）后再记录
                    # 避免在 yaw 不稳定时错误对齐，也避免反复重新对齐
                    if not self._rtk_py_aligned and (rtk_valid & 7) == 7:  # POS+YAW+ALN 全置位
                        self._rtk_align_x0 = rtk_x_enu
                        self._rtk_align_y0 = rtk_y_enu
                        self._rtk_align_yaw0 = rtk_yaw_tn  # θ₀ in degrees (C sends yaw_tn_deg)
                        self._rtk_py_aligned = True
                        print(f"[RTK] Py-side aligned: x0={rtk_x_enu:.3f}, y0={rtk_y_enu:.3f}, yaw0(vehicle)={rtk_yaw_tn:.1f}°, ant_raw={rtk_ant_raw:.1f}°")
                    # Update RTK status label + detail panel
                    pos_valid = (rtk_valid & 1) != 0
                    yaw_valid = (rtk_valid & 2) != 0
                    aligned = (rtk_valid & 4) != 0
                    status_parts = ['POS' if pos_valid else '---',
                                   'YAW' if yaw_valid else '---',
                                   'ALN' if aligned else '---']
                    self.rtk_status_lbl.config(
                        text=f"RTK: {'|'.join(status_parts)} Sats:{self._gnss_sats} FixQ:{self._gnss_fix_quality} Org:{self._gnss_sample_pct}%",
                        foreground='green' if aligned else ('orange' if pos_valid else 'gray'))
                    # RTK详细信息 panel — throttle to 4Hz (Text widget is expensive)
                    now = time.time()
                    if now - self._last_detail_time > 0.25:
                        self._last_detail_time = now
                        lines = [
                            f"Pos  : INS({rtk_x:.2f},{rtk_y:.2f})" if aligned else f"Pos  : ENU({rtk_x_enu:.3f},{rtk_y_enu:.3f}) (C not aligned)",
                            f"Yaw  : {rtk_yaw_tn:7.1f} deg  (vehicle heading)",
                            f"Ant  : {rtk_ant_raw:7.1f} deg  (raw MAIN>AUX)",
                            f"Flags: POS={'Y' if pos_valid else 'N'} YAW={'Y' if yaw_valid else 'N'} ALN={'Y' if aligned else 'N'}",
                            f"GNSS : Sats={self._gnss_sats}  Fix={self._gnss_fix_quality}({'RTK' if self._gnss_fix_quality==4 else 'FLT' if self._gnss_fix_quality==5 else 'GPS' if self._gnss_fix_quality==1 else 'DGPS' if self._gnss_fix_quality==2 else '?'})  Org={self._gnss_sample_pct}%",
                            f"Cache: {len(self._rtk_rotated)} pts",
                        ]
                        self._update_rtk_detail('\n'.join(lines))
                    # Pre-compute rotation for drawing cache (only when position actually changed)
                    if self._rtk_py_aligned and (rtk_valid & 1) and is_new_pos:
                        self._add_rtk_rotated(rtk_x, rtk_y, rtk_yaw, rtk_valid, phase)
                        if self.mode in ("follow", "change", "save"):
                            self._request_draw()
                except Exception as e:
                    print(f"[RTK] Parse error: {e} | raw: {line}")

        # GNSS状态数据解析（慢，1Hz）：G:sats,fix_quality,fix_state,sample_pct
        if line.startswith('G:'):
            for m in RE_GNSS.finditer(line):
                try:
                    self._gnss_sats = int(m.group(1))
                    self._gnss_fix_quality = int(m.group(2))  # GGA fix quality
                    self._gnss_fix = int(m.group(3)) if m.group(3) else 0
                    self._gnss_sample_pct = int(m.group(4)) if m.group(4) else 0
                    if self._gnss_fix_quality and self._gnss_fix_quality not in (4, 5):
                        self._rtk_warning = f"Live FixQ={self._gnss_fix_quality}: waiting for RTK Float/Fixed"
                    elif self._rtk_warning.startswith("Live FixQ="):
                        self._rtk_warning = ""
                except Exception as e:
                    print(f"[GNSS] Parse error: {e} | raw: {line}")

    def _update_rtk_detail(self, text):
        """Update the multi-line RTK detail panel (non-blocking, text widget)."""
        try:
            self.rtk_detail_text.config(state=tk.NORMAL)
            self.rtk_detail_text.delete('1.0', tk.END)
            self.rtk_detail_text.insert('1.0', text)
            self.rtk_detail_text.config(state=tk.DISABLED)
        except Exception:
            pass

    def _request_draw(self, force=False):
        """Throttled draw request - schedules a redraw if not too frequent"""
        if self._draw_pending:
            return

        now = time.time()
        self._draw_pending = True
        if force or now - self._last_draw_time >= DRAW_THROTTLE:
            self._last_draw_time = now
            self.after_idle(self._flush_draw)
        else:
            delay_ms = int((DRAW_THROTTLE - (now - self._last_draw_time)) * 1000) + 10
            self.after(delay_ms, self._flush_draw)

    def _flush_draw(self):
        if self._redraw_active:
            return
        self._draw_pending = False
        self._last_draw_time = time.time()
        self._redraw_active = True
        try:
            self._do_redraw()
        finally:
            self._redraw_active = False

    def _do_redraw(self):
        """Full redraw of current data - optimized for performance."""
        # Save current view limits before clearing
        if getattr(self, '_view_initialized', False):
            try:
                old_xlim = self.ax.get_xlim()
                old_ylim = self.ax.get_ylim()
            except:
                self._view_initialized = False

        self.ax.clear()
        self._init_axes()
        self._hover_annot = None

        if self.mode == "save":
            self._draw_save_content()
        elif self.mode == "follow":
            self._draw_follow_content()
        elif self.mode == "change":
            self._draw_change_content()
        else:
            # idle: show whatever we have
            if self.save_points:
                self._draw_save_content()
            elif self.follow_pts:
                self._draw_follow_content()

        # Auto-fit only on first draw after mode switch, then preserve user pan/zoom
        if not getattr(self, '_view_initialized', False):
            self._auto_fit()
            self._view_initialized = True
        else:
            # Restore user's view position (preserve pan/zoom)
            self.ax.set_xlim(old_xlim)
            self.ax.set_ylim(old_ylim)

        # Legend construction is surprisingly expensive in TkAgg. Hide it while
        # streaming; show it again for offline inspection or small loaded traces.
        show_legend = (not self.is_connected) or (len(self.rtk_pts) + len(self.ins_pts) < 80)
        handles, labels = self.ax.get_legend_handles_labels()
        if handles and show_legend:
            seen = set()
            unique_h, unique_l = [], []
            for h, l in zip(handles, labels):
                if l not in seen:
                    seen.add(l)
                    unique_h.append(h)
                    unique_l.append(l)
            self.ax.legend(unique_h, unique_l, loc='upper left', fontsize=7,
                          framealpha=0.86, edgecolor='#cccccc', fancybox=True)

        self.canvas.draw_idle()

    def _draw_save_content(self):
        if not self.save_points:
            self._draw_rtk_overlay()
            self._draw_ins_overlay()
            return
        xs = [p[1] for p in self.save_points]
        ys = [p[2] for p in self.save_points]
        n = len(self.save_points)
        self.ax.set_title(f'存点车辆轨迹 ({n}点)', fontsize=13, fontweight='bold')
        self.ax.plot(xs, ys, '-', color='#1976D2', linewidth=1.8, alpha=0.72, zorder=2, label='INS轨迹')
        # Only scatter last 100 points to reduce overhead (was 200)
        scatter_n = min(n, 24)
        self.ax.scatter(xs[-scatter_n:], ys[-scatter_n:], c='#E53935', s=18, zorder=5, alpha=0.72, edgecolors='white', linewidths=0.4)
        # Sparse annotation: max 6 labels
        step = max(1, n // 6)
        for i, (idx, x, y, _, _) in enumerate(self.save_points):
            if i not in (0, n - 1) and (i % step == 0):
                self.ax.annotate(str(idx), (x, y), textcoords="offset points",
                                 xytext=(5, 5), fontsize=7, color='#D32F2F', alpha=0.7)
        if n >= 1:
            self.ax.scatter([xs[0]], [ys[0]], c='#4CAF50', s=120, marker='^', zorder=7, edgecolors='white', linewidths=1, label='起点')
        if n >= 2:
            self.ax.scatter([xs[-1]], [ys[-1]], c='#9C27B0', s=120, marker='v', zorder=7, edgecolors='white', linewidths=1, label='终点')
        # Draw car model at the last point
        last_yaw = self.save_points[-1][3]
        self._draw_car_model(xs[-1], ys[-1], last_yaw, color='#2196F3')
        # Draw RTK overlay if available
        self._draw_rtk_overlay()
        self._draw_ins_overlay()

    def _draw_follow_content(self):
        if not self.follow_pts:
            return
        # Save reference
        if self.save_points:
            sx = [p[1] for p in self.save_points]
            sy = [p[2] for p in self.save_points]
            self.ax.plot(sx, sy, '--', color='#BDBDBD', linewidth=1, alpha=0.5, label='存点参考', zorder=1)
            self.ax.scatter(sx, sy, c='#E0E0E0', s=15, alpha=0.4, zorder=1)

        # Separate follow data into two categories:
        #   idx=0: car's current position (from INS)
        #   idx>0: target point from stored trajectory
        car_pts = [(p[1], p[2]) for p in self.follow_pts if p[0] == 0]
        tgt_pts = [(p[1], p[2], p[0]) for p in self.follow_pts if p[0] > 0]

        n = len(self.follow_pts)
        self.ax.set_title(f'循迹车辆轨迹 ({n}点, 车辆:{len(car_pts)}, 目标:{len(tgt_pts)})', fontsize=13, fontweight='bold')

        # Draw car path: connect only car current positions (idx=0)
        if car_pts:
            cx = [p[0] for p in car_pts]
            cy = [p[1] for p in car_pts]
            self.ax.plot(cx, cy, '-', color='#1565C0', linewidth=2.5, label='车辆轨迹', zorder=3)
            self.ax.scatter([cx[0]], [cy[0]], c='#4CAF50', s=120, marker='^', zorder=7, edgecolors='white', linewidths=1, label='起点')
            self.ax.scatter([cx[-1]], [cy[-1]], c='#FF5722', s=120, marker='o', zorder=7, edgecolors='white', linewidths=1, label='当前位置')
            # Draw car model at the last car position
            last_car_yaw = [p[3] for p in self.follow_pts if p[0] == 0][-1]
            self._draw_car_model(cx[-1], cy[-1], last_car_yaw, color='#1565C0')

        # Draw target points: connect only stored trajectory targets (idx>0)
        if tgt_pts:
            IDX_JUMP_THRESHOLD = 5
            segments = []
            seg_x, seg_y = [tgt_pts[0][0]], [tgt_pts[0][1]]
            for i in range(1, len(tgt_pts)):
                prev_idx = tgt_pts[i-1][2]
                curr_idx = tgt_pts[i][2]
                if abs(curr_idx - prev_idx) > IDX_JUMP_THRESHOLD:
                    segments.append((seg_x[:], seg_y[:]))
                    seg_x, seg_y = [tgt_pts[i][0]], [tgt_pts[i][1]]
                else:
                    seg_x.append(tgt_pts[i][0])
                    seg_y.append(tgt_pts[i][1])
            segments.append((seg_x, seg_y))
            
            for i, (seg_x, seg_y) in enumerate(segments):
                label = '目标轨迹' if i == 0 else None
                self.ax.plot(seg_x, seg_y, '-', color='#FF9800', linewidth=1.5, alpha=0.7, label=label, zorder=2)
            tx = [p[0] for p in tgt_pts]
            ty = [p[1] for p in tgt_pts]
            scatter_n = min(len(tx), 60)
            self.ax.scatter(tx[-scatter_n:], ty[-scatter_n:], c='#FF9800', s=20, alpha=0.6, zorder=4)

        # Draw RTK overlay for INS vs RTK comparison
        self._draw_rtk_overlay()
        self._draw_ins_overlay()

    # RTK phase colors
    RTK_PHASE_COLORS = {
        'DRIVE':  '#FFC107',   # amber - 遥控行驶
        'CHANGE': '#FF9800',   # orange - 调头阶段
        'FOLLOW': '#4CAF50',   # green - 循迹阶段
        'ARRIVE': '#9C27B0',   # purple - 到达
        'IDLE':   '#9E9E9E',   # gray - 空闲
        'ERROR':  '#F44336',   # red - 错误
    }

    @staticmethod
    def _smooth_pts(pts, window=3):
        """Simple moving-average smooth on (x,y) only, keep yaw/vf/phase intact."""
        if len(pts) < window:
            return pts
        half = window // 2
        out = []
        for i in range(len(pts)):
            s = max(0, i - half)
            e = min(len(pts), i + half + 1)
            ax = sum(p[0] for p in pts[s:e]) / (e - s)
            ay = sum(p[1] for p in pts[s:e]) / (e - s)
            out.append((ax, ay, pts[i][2], pts[i][3], pts[i][4]))
        return out

    @staticmethod
    def _thin_history_keep_recent(pts, recent_count, max_history_draw):
        """Keep the whole path visible while limiting artist point count."""
        n = len(pts)
        if n <= recent_count + max_history_draw:
            return pts
        history = pts[:n - recent_count]
        recent = pts[n - recent_count:]
        step = max(1, math.ceil(len(history) / max_history_draw))
        thinned = history[::step]
        if thinned and recent and thinned[-1] != recent[0]:
            thinned.append(recent[0])
        return thinned + recent

    def _add_rtk_rotated(self, x_ins_c, y_ins_c, yaw_ins_deg, vf, phase):
        """Append to draw cache using C-side already-rotated INS coordinates.
        C side rtk.c already computed x_ins/y_ins via ENU→INS rotation.
        No Python-side re-rotation needed; yaw is local INS yaw, not true-north yaw.
        """
        yaw_r_rad = 0.0
        if vf & 2:
            yaw_r_deg = yaw_ins_deg
            while yaw_r_deg > 180: yaw_r_deg -= 360
            while yaw_r_deg < -180: yaw_r_deg += 360
            yaw_r_rad = math.radians(yaw_r_deg)
        self._rtk_rotated.append((x_ins_c, y_ins_c, yaw_r_rad, vf, phase))

    def _get_rtk_display_points(self):
        """Return RTK draw points after UI-only antenna angle compensation."""
        offset_deg = float(self.rtk_angle_offset_var.get())
        if abs(offset_deg) < 1e-6:
            return self._rtk_rotated

        angle = math.radians(offset_deg)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        out = []
        for x, y, yaw, vf, phase in self._rtk_rotated:
            out.append((
                cos_a * x - sin_a * y,
                sin_a * x + cos_a * y,
                yaw + angle,
                vf,
                phase))
        return out

    def _on_rtk_angle_offset_change(self, _value=None):
        offset = float(self.rtk_angle_offset_var.get())
        self.rtk_angle_offset_lbl.config(text=f"{offset:+.1f}度")
        self._request_draw(force=True)

    def _reset_rtk_angle_offset(self):
        self.rtk_angle_offset_var.set(0.0)
        self._on_rtk_angle_offset_change()

    def _draw_rtk_overlay(self):
        """Draw RTK trajectory overlay using pre-computed rotation cache.

        The cache _rtk_rotated is built incrementally as RTK points arrive,
        so we never recompute rotation on redraw.
        """
        rotated_pts = self._get_rtk_display_points()

        if self._rtk_warning:
            self.ax.text(0.02, 0.98, self._rtk_warning,
                         transform=self.ax.transAxes, fontsize=9, color='#B71C1C',
                         va='top', ha='left',
                         bbox=dict(boxstyle='round,pad=0.35', facecolor='#FFEBEE',
                                   alpha=0.92, edgecolor='#EF9A9A'))

        if not rotated_pts:
            if self.rtk_pts:
                last = self.rtk_pts[-1]
                pos_ok = (last[3] & 1) != 0
                yaw_ok = (last[3] & 2) != 0
                aln_ok = (last[3] & 4) != 0
                py_aln = 'Y' if self._rtk_py_aligned else 'N'
                diag = f"RTK: POS={'Y' if pos_ok else 'N'} YAW={'Y' if yaw_ok else 'N'} ALN={'Y' if aln_ok else 'N'} Py:{py_aln} ({len(self.rtk_pts)} pts)"
                self.ax.text(0.02, 0.02, diag,
                            transform=self.ax.transAxes, fontsize=9, color='gray',
                            bbox=dict(boxstyle='round', facecolor='#ffffcc', alpha=0.8))
            return

        # Keep the full route visible, but thin old history so real-time redraws
        # do not stutter as the RTK buffer grows.
        display_pts = self._thin_history_keep_recent(
            rotated_pts,
            RTK_RECENT_FULL_POINTS,
            RTK_HISTORY_MAX_DRAW)

        all_x = [p[0] for p in display_pts]
        all_y = [p[1] for p in display_pts]
        self.ax.plot(all_x, all_y, '-', color='#66BB6A', linewidth=1.2, alpha=0.4,
                     zorder=2)

        # Phase-colored segments
        segments = []
        cur_phase = display_pts[0][4]
        cur_seg = [display_pts[0]]
        for pt in display_pts[1:]:
            if pt[4] == cur_phase:
                cur_seg.append(pt)
            else:
                segments.append((cur_phase, cur_seg))
                cur_phase = pt[4]
                cur_seg = [cur_seg[-1], pt]
        segments.append((cur_phase, cur_seg))

        drawn_phases = set()
        for phase, seg_pts in segments:
            if len(seg_pts) < 2:
                continue
            xs = [p[0] for p in seg_pts]
            ys = [p[1] for p in seg_pts]
            color = self.RTK_PHASE_COLORS.get(phase, '#9E9E9E')
            label = f'RTK-{phase}' if phase not in drawn_phases else None
            drawn_phases.add(phase)
            self.ax.plot(xs, ys, '-', color=color, linewidth=2.0, alpha=0.85,
                         zorder=3, label=label)

        # Latest RTK diamond
        last = display_pts[-1]
        color = self.RTK_PHASE_COLORS.get(last[4], '#9E9E9E')
        self.ax.scatter([last[0]], [last[1]], c=color, s=120, marker='D',
                       zorder=8, edgecolors='white', linewidths=1.5, label='RTK位置')

        # RTK heading arrow
        if last[3] & 2:
            arrow_len = 0.5
            adx = arrow_len * math.cos(last[2])
            ady = arrow_len * math.sin(last[2])
            self.ax.annotate('', xy=(last[0] + adx, last[1] + ady),
                            xytext=(last[0], last[1]),
                            arrowprops=dict(arrowstyle='->', color='#2E7D32', lw=2.5),
                            zorder=9)

    def _draw_ins_overlay(self):
        """Draw INS trajectory overlay (from I: data) on the plot.
        
        INS data comes directly in INS coordinates (x, y from Kalman filter),
        so no rotation is needed. Draws as a thin cyan line with current position marker.
        """
        if not self.ins_pts:
            return

        # Limit drawing to last N points for performance
        MAX_INS_DRAW = 400
        pts = self._thin_history_keep_recent(
            self.ins_pts,
            INS_RECENT_FULL_POINTS,
            INS_HISTORY_MAX_DRAW)

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        # Draw INS trajectory as thin cyan line
        self.ax.plot(xs, ys, '-', color='#00BCD4', linewidth=1.0, alpha=0.5,
                     zorder=2, label='INS轨迹')

        # Current INS position - circle marker
        last = pts[-1]
        self.ax.scatter([last[0]], [last[1]], c='#00BCD4', s=80, marker='o',
                       zorder=7, edgecolors='white', linewidths=1.0, label='INS位置')

        # INS heading arrow (using EKF yaw)
        arrow_len = 0.4
        yaw_rad = math.radians(-last[4])  # serial yaw is CW-positive; plot uses CCW-positive
        adx = arrow_len * math.cos(yaw_rad)
        ady = arrow_len * math.sin(yaw_rad)
        self.ax.annotate('', xy=(last[0] + adx, last[1] + ady),
                        xytext=(last[0], last[1]),
                        arrowprops=dict(arrowstyle='->', color='#00838F', lw=2.0),
                        zorder=8)

        # LUT-corrected mag heading arrow (if LUT loaded)
        if self._mag_lut is not None:
            yaw_mag_lut = self._apply_mag_lut(last[3])  # last[3] = yaw_mag_deg
            yaw_lut_rad = math.radians(-yaw_mag_lut)
            adx2 = arrow_len * math.cos(yaw_lut_rad)
            ady2 = arrow_len * math.sin(yaw_lut_rad)
            self.ax.annotate('', xy=(last[0] + adx2, last[1] + ady2),
                            xytext=(last[0], last[1]),
                            arrowprops=dict(arrowstyle='->', color='#FF6F00', lw=1.5, linestyle='--'),
                            zorder=8)
            # Raw mag heading arrow (thin, for comparison)
            yaw_mag_rad = math.radians(-last[3])
            adx3 = arrow_len * 0.8 * math.cos(yaw_mag_rad)
            ady3 = arrow_len * 0.8 * math.sin(yaw_mag_rad)
            self.ax.annotate('', xy=(last[0] + adx3, last[1] + ady3),
                            xytext=(last[0], last[1]),
                            arrowprops=dict(arrowstyle='->', color='#9C27B0', lw=1.0, linestyle=':'),
                            zorder=8)

    def _load_mag_lut(self):
        """Load magnetometer yaw LUT from track_data/mag_yaw_lut.py if available"""
        lut_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'track_data', 'mag_yaw_lut.py')
        if os.path.exists(lut_path):
            try:
                import importlib.util
                spec = importlib.util.spec_from_file_location("mag_yaw_lut_mod", lut_path)
                lut_mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(lut_mod)
                self._mag_lut = lut_mod.MAG_YAW_LUT
                if hasattr(lut_mod, 'MAG_YAW_LUT_BIN_SIZE'):
                    self._mag_lut_bin_size = lut_mod.MAG_YAW_LUT_BIN_SIZE
                print(f"[LUT] Loaded {len(self._mag_lut)} entries from {lut_path}")
            except Exception as e:
                print(f"[LUT] Failed to load: {e}")
                self._mag_lut = None

    def _apply_mag_lut(self, mag_yaw_deg):
        """Apply LUT correction to magnetometer yaw"""
        if self._mag_lut is None:
            return mag_yaw_deg
        bin_idx = int(mag_yaw_deg % 360 / self._mag_lut_bin_size) % len(self._mag_lut)
        correction = self._mag_lut.get(bin_idx, 0.0)
        return mag_yaw_deg - correction

    def _start_save(self):
        self.mode = "save"
        self.save_points.clear()
        self.follow_pts.clear()
        self.rtk_pts.clear()
        self.ins_pts.clear()
        self.yawrtk_data.clear()
        self.save_diag.clear()
        self.ins_diag.clear()
        self.rtk_diag.clear()
        self._rtk_rotated.clear()
        self._rtk_py_aligned = False
        self._rtk_align_x0 = None
        self._rtk_align_y0 = None
        self._rtk_align_yaw0 = None
        self.btn_start_save.config(state=tk.DISABLED)
        self.btn_stop_save.config(state=tk.NORMAL)
        self.btn_start_follow.config(state=tk.DISABLED)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="模式: 存点中", foreground="blue")
        self.pts_lbl.config(text="存点: 0")
        self._clear_plot()

    def _stop_save(self):
        self.mode = "idle"
        self.save_count += 1
        self.btn_start_save.config(state=tk.NORMAL)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.NORMAL)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="模式: 空闲", foreground="black")
        if self.save_points:
            self._auto_save_points()

    def _start_follow(self):
        self.mode = "follow"
        self.follow_pts.clear()
        self.rtk_pts.clear()
        self.ins_pts.clear()
        self.yawrtk_data.clear()
        self.ins_diag.clear()
        self.rtk_diag.clear()
        self._rtk_rotated.clear()
        self._rtk_py_aligned = False
        self._rtk_align_x0 = None
        self._rtk_align_y0 = None
        self._rtk_align_yaw0 = None
        self.btn_start_save.config(state=tk.DISABLED)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.DISABLED)
        self.btn_stop_follow.config(state=tk.NORMAL)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="模式: 循迹中", foreground="red")
        self.flw_lbl.config(text="循迹点: 0")
        self._clear_plot()
        if self.save_points:
            self._draw_save_ref()

    def _stop_follow(self):
        self.mode = "idle"
        self.btn_start_save.config(state=tk.NORMAL)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.NORMAL)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="模式: 空闲", foreground="black")
        if self.follow_pts:
            self._auto_save_follow()

    def _back_to_save(self):
        self.mode = "idle"
        self.btn_start_save.config(state=tk.NORMAL)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.NORMAL)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="模式: 空闲", foreground="black")
        if self.follow_pts:
            self._auto_save_follow()

    def _clear_plot(self):
        self.ax.clear()
        self._init_axes()
        self._hover_annot = None
        self._last_draw_time = 0
        self._draw_pending = False
        self._view_initialized = False  # Reset so next redraw will auto-fit
        self.canvas.draw()

    def _draw_save_ref(self):
        if not self.save_points:
            return
        sx = [p[1] for p in self.save_points]
        sy = [p[2] for p in self.save_points]
        self.ax.plot(sx, sy, '--', color='#BDBDBD', linewidth=1, alpha=0.5, label='存点参考', zorder=1)
        self.ax.scatter(sx, sy, c='#E0E0E0', s=15, alpha=0.4, zorder=1)
        # Mark start and end of saved trajectory
        self.ax.scatter([sx[0]], [sy[0]], c='green', s=60, marker='o', zorder=3, label='起点')
        self.ax.scatter([sx[-1]], [sy[-1]], c='red', s=60, marker='s', zorder=3, label='终点')
        self.canvas.draw_idle()

    def _draw_car_model(self, x, y, yaw, color='#FF3300', alpha=0.9, size=0.12):
        """Draw a car model at (x, y) rotated by yaw, with yaw label at upper-right.

        Args:
            x, y: position in data coordinates
            yaw: heading angle in radians (0 = +X direction, CCW positive)
            color: fill color of the car body
            alpha: transparency
            size: overall length of the car in data units
        """
        # Car body vertices (top-down, nose points to +X)
        # Shape: arrow-like car silhouette
        hw = size * 0.35   # half width of body
        bl = size * 0.45   # body half-length (rear)
        nl = size * 0.55   # nose length (front)
        # Rear-left, rear-right, front-right, nose, front-left
        body = np.array([
            [-bl, -hw],
            [-bl,  hw],
            [ nl * 0.4,  hw * 0.85],
            [ nl,  0],
            [ nl * 0.4, -hw * 0.85],
        ])
        # Rotation matrix
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        rotated = (R @ body.T).T + np.array([x, y])

        # Draw filled car body
        car_poly = self.ax.fill(rotated[:, 0], rotated[:, 1],
                                color=color, alpha=alpha, zorder=10)[0]

        # Draw a heading line from center to nose
        nose_x, nose_y = rotated[3]
        self.ax.plot([x, nose_x], [y, nose_y], color='white', linewidth=1.5, alpha=0.8, zorder=11)

        # Small wheel marks (rear axle)
        wheel_len = size * 0.2
        for sign in [-1, 1]:
            wx = x - bl * 0.6 * c
            wy = y - bl * 0.6 * s
            ox = sign * hw * 0.7 * (-s)
            oy = sign * hw * 0.7 * c
            self.ax.plot([wx + ox - wheel_len * 0.5 * c, wx + ox + wheel_len * 0.5 * c],
                         [wy + oy - wheel_len * 0.5 * s, wy + oy + wheel_len * 0.5 * s],
                         color='#333333', linewidth=3, alpha=0.7, zorder=11, solid_capstyle='round')

        # Yaw label at upper-right of the car (offset in data coords)
        yaw_deg = np.degrees(yaw)
        label = f"yaw={yaw_deg:.1f}°"
        # Place text offset to upper-right in screen-consistent direction
        offset_data = size * 0.9
        tx = x + offset_data * (c - s) * 0.5
        ty = y + offset_data * (s + c) * 0.5
        self.ax.annotate(label, (tx, ty),
                         fontsize=8, fontweight='bold', color='#222222',
                         bbox=dict(boxstyle='round,pad=0.25', facecolor='#FFFFCC',
                                   alpha=0.85, edgecolor='#999999', linewidth=0.8),
                         ha='center', va='center', zorder=12)

    def _draw_change_content(self):
        """Draw CHANGE phase: saved trajectory + real-time remote control trajectory"""
        # Draw saved reference trajectory
        if self.save_points:
            sx = [p[1] for p in self.save_points]
            sy = [p[2] for p in self.save_points]
            self.ax.plot(sx, sy, '--', color='#BDBDBD', linewidth=1, alpha=0.5, label='Save ref', zorder=1)
            self.ax.scatter(sx, sy, c='#E0E0E0', s=15, alpha=0.4, zorder=1)
            # Mark start and end
            self.ax.scatter([sx[0]], [sy[0]], c='green', s=60, marker='o', zorder=3, label='Start')
            self.ax.scatter([sx[-1]], [sy[-1]], c='red', s=60, marker='s', zorder=3, label='End')
        # Draw real-time change trajectory
        if self.change_pts:
            cx = [p[1] for p in self.change_pts]
            cy = [p[2] for p in self.change_pts]
            self.ax.plot(cx, cy, '-', color='#FF6600', linewidth=2, alpha=0.8, label='RC Change', zorder=2)
            # Draw car model at the last point
            last_yaw = self.change_pts[-1][3]
            self._draw_car_model(cx[-1], cy[-1], last_yaw, color='#FF6600')
        # Draw RTK overlay
        self._draw_rtk_overlay()
        self._draw_ins_overlay()

    # ---- Offline data loading ----
    def _load_latest_data(self):
        data_dir = self._get_save_dir()
        candidates = [
            os.path.join(data_dir, name) for name in os.listdir(data_dir)
            if name.endswith('.txt') and '_rtk' not in name and '_ins' not in name
        ]
        if not candidates:
            messagebox.showwarning("警告", "没有找到轨迹数据文件")
            return
        latest = max(candidates, key=os.path.getmtime)
        self._load_data_bundle(latest)

    def _open_data_file(self):
        fp = filedialog.askopenfilename(
            initialdir=self._get_save_dir(),
            filetypes=[("Track data", "*.txt"), ("All files", "*.*")]
        )
        if fp:
            self._load_data_bundle(fp)

    def _load_data_bundle(self, main_fp):
        base, ext = os.path.splitext(main_fp)
        if base.endswith('_rtk') or base.endswith('_ins'):
            base = base.rsplit('_', 1)[0]
            main_fp = base + ext
        rtk_fp = base + '_rtk' + ext
        ins_fp = base + '_ins' + ext

        self._reset_state()
        self.subject_mode.set(3 if 'subj3_' in os.path.basename(main_fp) else 1)
        self._update_subject_ui()
        self.mode = 'save'
        self.subject3_state = 'DRIVE' if 'drive' in os.path.basename(main_fp) else 'FOLLOW'

        loaded_main = self._load_main_track_file(main_fp)
        loaded_rtk = self._load_rtk_file(rtk_fp) if os.path.exists(rtk_fp) else 0
        loaded_ins = self._load_ins_file(ins_fp) if os.path.exists(ins_fp) else 0

        if self.follow_pts and not self.save_points:
            self.mode = 'follow'
        else:
            self.mode = 'save'

        self._rebuild_rtk_draw_cache()
        self._view_initialized = False
        self._do_redraw()
        self.mode_lbl.config(text=f"模式: 离线 {self.mode}", foreground="#455A64")
        self.pts_lbl.config(text=f"存点: {len(self.save_points)}")
        self.flw_lbl.config(text=f"循迹点: {len(self.follow_pts)}")
        self.ins_status_lbl.config(text=f"INS: {loaded_ins}点", foreground='blue' if loaded_ins else 'gray')
        self.rtk_status_lbl.config(text=f"RTK: {loaded_rtk}点", foreground='green' if loaded_rtk else 'gray')
        self._update_info(
            f"已加载: {os.path.basename(main_fp)}\n"
            f"主轨迹: {loaded_main} pts\nRTK: {loaded_rtk} 点\nINS: {loaded_ins}点"
        )

    def _load_main_track_file(self, fp):
        count = 0
        if not os.path.exists(fp):
            return count
        with open(fp, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                try:
                    if len(parts) >= 5:
                        idx = int(float(parts[0]))
                        x = float(parts[1])
                        y = float(parts[2])
                        yaw = float(parts[3])
                        fifth = float(parts[4])
                        if len(parts) >= 9:
                            self.save_diag.append((idx, int(float(parts[5])), int(float(parts[6])), int(float(parts[7])), int(float(parts[8]))))
                        if idx == 0 or 'follow' in os.path.basename(fp):
                            target_idx = int(fifth)
                            self.follow_pts.append((idx, x, y, yaw, target_idx))
                        else:
                            self.save_points.append((idx, x, y, yaw, fifth))
                        count += 1
                except ValueError:
                    continue
        return count

    def _load_ins_file(self, fp):
        count = 0
        with open(fp, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) < 5:
                    continue
                try:
                    self.ins_pts.append(tuple(float(parts[i]) for i in range(5)))
                    if len(parts) >= 11:
                        self.ins_diag.append((int(float(parts[5])), int(float(parts[6])), int(float(parts[7])),
                                              int(float(parts[8])), int(float(parts[9])), int(float(parts[10]))))
                    count += 1
                except ValueError:
                    continue
        if len(self.ins_pts) > self._ins_max_pts:
            self.ins_pts = self.ins_pts[-self._ins_max_pts:]
        return count

    def _load_rtk_file(self, fp):
        count = 0
        align_re = re.compile(r'x0=([-0-9.]+).*y0=([-0-9.]+).*yaw0=([-0-9.]+)')
        fix_re = re.compile(r'fix=(\d+)')
        header_fix = None
        legacy_has_px_columns = False
        with open(fp, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                if line.startswith('# Format:') and 'px_ins' in line and 'ant_raw' not in line:
                    legacy_has_px_columns = True
                    continue
                if line.startswith('# GNSS status:'):
                    m = fix_re.search(line)
                    if m:
                        header_fix = int(m.group(1))
                    continue
                if line.startswith('# Py-side alignment:'):
                    m = align_re.search(line)
                    if m:
                        self._rtk_align_x0 = float(m.group(1))
                        self._rtk_align_y0 = float(m.group(2))
                        self._rtk_align_yaw0 = float(m.group(3))
                        self._rtk_py_aligned = True
                    continue
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) < 8:
                    continue
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    yaw = float(parts[2])
                    vf = int(float(parts[3]))
                    ph = parts[4]
                    x_enu = float(parts[5])
                    y_enu = float(parts[6])
                    yaw_tn = float(parts[7])
                    ant_raw = 0.0 if legacy_has_px_columns else (float(parts[8]) if len(parts) > 8 else 0.0)
                    self.rtk_pts.append((x, y, yaw, vf, ph, x_enu, y_enu, yaw_tn, ant_raw))
                    if len(parts) >= 16:
                        self.rtk_diag.append((int(float(parts[11])), int(float(parts[12])), int(float(parts[13])),
                                              int(float(parts[14])), int(float(parts[15]))))
                    count += 1
                except ValueError:
                    continue
        self._update_rtk_quality_warning(header_fix)
        return count

    def _update_rtk_quality_warning(self, header_fix=None):
        warnings = []
        if header_fix is not None and header_fix not in (4, 5):
            warnings.append(f"RTK file FixQ={header_fix}: single/DGPS data, not RTK truth")

        if len(self.rtk_pts) >= 3:
            enu_steps = []
            for prev, cur in zip(self.rtk_pts, self.rtk_pts[1:]):
                dx = cur[5] - prev[5]
                dy = cur[6] - prev[6]
                enu_steps.append(math.hypot(dx, dy))
            median_step = float(np.median(enu_steps)) if enu_steps else 0.0
            if median_step > 0.25:
                warnings.append(f"RTK ENU step median={median_step:.2f}m: quantized/low quality")

        self._rtk_warning = "\n".join(warnings)

    def _rebuild_rtk_draw_cache(self):
        self._rtk_rotated.clear()
        for x, y, yaw, vf, ph, *_ in self.rtk_pts:
            if vf & 1:
                self._add_rtk_rotated(x, y, yaw, vf, ph)

    def _save_plot_png(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"track_plot_{ts}.png")
        self.fig.savefig(fp, dpi=160, bbox_inches='tight')
        self._update_info(f"已保存轨迹图片:\n{fp}")
        print(f"[Plot] Saved {fp}")
        return fp

    def _save_window_screenshot(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"track_plotter_ui_{ts}.png")
        self.update_idletasks()
        self.update()
        try:
            from PIL import ImageGrab
            x = self.winfo_rootx()
            y = self.winfo_rooty()
            w = self.winfo_width()
            h = self.winfo_height()
            ImageGrab.grab((x, y, x + w, y + h)).save(fp)
            self._update_info(f"已保存窗口截图:\n{fp}")
            print(f"[UI] Saved {fp}")
            return fp
        except Exception as e:
            print(f"[UI] Screenshot failed: {e}")
            return self._save_plot_png()


    # ---- File saving ----
    def _get_save_dir(self):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "track_data")
        os.makedirs(d, exist_ok=True)
        return d

    def _save_rtk_data(self, base_filepath, phase=""):
        """Save RTK data to a separate file alongside the main trajectory file.
        
        Args:
            base_filepath: the main trajectory file path, RTK file will be _rtk suffixed
            phase: optional phase label for the header
        """
        if not self.rtk_pts:
            return
        # Derive RTK file path: same name with _rtk suffix before extension
        base, ext = os.path.splitext(base_filepath)
        rtk_fp = f"{base}_rtk{ext}"
        try:
            with open(rtk_fp, 'w', encoding='utf-8') as f:
                f.write(f"# RTK trajectory data{f' - {phase}' if phase else ''}\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Total RTK points: {len(self.rtk_pts)}\n")
                f.write(f"# Format: x_ins_c,y_ins_c,yaw_ins,valid_flags,phase,x_enu,y_enu,yaw_tn,ant_raw,px_ins,py_ins,seq,tick_ms,q_drop,ring_count,pending4\n")
                f.write(f"# valid_flags: bit0=pos_valid, bit1=yaw_valid, bit2=is_aligned\n")
                f.write(f"# x_enu/y_enu: ENU coordinates (East/North from origin)\n")
                f.write(f"# yaw_tn: corrected vehicle true north heading (deg)\n")
                f.write(f"# ant_raw: raw GN43RFA antenna_direction (deg), for offset calibration\n")
                f.write(f"# px_ins/py_ins: Py-side ENU→INS rotated coordinates\n")
                f.write(f"# GNSS status: sats={self._gnss_sats}, fix={self._gnss_fix_quality}, origin_sample={self._gnss_sample_pct}%\n")
                if self._rtk_py_aligned:
                    f.write(f"# Py-side alignment: x0={self._rtk_align_x0:.4f}, y0={self._rtk_align_y0:.4f}, yaw0={self._rtk_align_yaw0:.4f} deg\n")
                f.write(f"#----------------------------------------\n")
                # _rtk_align_yaw0 is in degrees (from C: yaw_tn*57.29578), convert to radians for sin/cos
                yaw0_rad = math.radians(self._rtk_align_yaw0) if self._rtk_py_aligned else 0.0
                for i, item in enumerate(self.rtk_pts):
                    # Unpack: (x, y, yaw, vf, ph, x_enu, y_enu, yaw_tn, ant_raw) or 8-tuple for old data
                    x, y, yaw, vf, ph, x_enu, y_enu, yaw_tn = item[0], item[1], item[2], item[3], item[4], item[5], item[6], item[7]
                    ant_raw = item[8] if len(item) > 8 else 0.0
                    # Compute Py-side rotated coordinates
                    if self._rtk_py_aligned and (vf & 1):
                        dx = x_enu - self._rtk_align_x0
                        dy = y_enu - self._rtk_align_y0
                        sin_t = math.sin(yaw0_rad)
                        cos_t = math.cos(yaw0_rad)
                        px = sin_t * dx + cos_t * dy      # d·f: forward
                        py = -cos_t * dx + sin_t * dy     # d·l: left, matches INS y
                    else:
                        px, py = 0.0, 0.0
                    seq, tick_ms, q_drop, ring_count, pending4 = self.rtk_diag[i] if i < len(self.rtk_diag) else (0, 0, 0, 0, 0)
                    f.write(f"{x:.4f},{y:.4f},{yaw:.4f},{vf},{ph},{x_enu:.4f},{y_enu:.4f},{yaw_tn:.4f},{ant_raw:.1f},{px:.4f},{py:.4f},{seq},{tick_ms},{q_drop},{ring_count},{pending4}\n")
            print(f"[RTK] Saved {len(self.rtk_pts)} points to {os.path.basename(rtk_fp)}")
        except Exception as e:
            print(f"[RTK] 保存失败: {e}")

    def _save_ins_data(self, base_filepath, phase=""):
        """Save INS data to a separate file alongside the main trajectory file.
        
        Args:
            base_filepath: the main trajectory file path, INS file will be _ins suffixed
            phase: optional phase label for the header
        """
        if not self.ins_pts:
            return
        base, ext = os.path.splitext(base_filepath)
        ins_fp = f"{base}_ins{ext}"
        try:
            with open(ins_fp, 'w', encoding='utf-8') as f:
                f.write(f"# INS trajectory data{f' - {phase}' if phase else ''}\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Total INS points: {len(self.ins_pts)}\n")
                f.write(f"# Format: x,y,yaw_gyro_deg,yaw_mag_deg,yaw_ekf_deg,seq,tick_ms,ring_count,pending4,q_drop,q_depth\n")
                f.write(f"# yaw_gyro: cumulative gyro integration yaw (deg)\n")
                f.write(f"# yaw_mag: pure magnetometer yaw (deg)\n")
                f.write(f"# yaw_ekf: EKF fused yaw (deg)\n")
                f.write(f"#----------------------------------------\n")
                for i, (x, y, yaw_gyro, yaw_mag, yaw_ekf) in enumerate(self.ins_pts):
                    seq, tick_ms, ring_count, pending4, q_drop, q_depth = self.ins_diag[i] if i < len(self.ins_diag) else (0, 0, 0, 0, 0, 0)
                    f.write(f"{x:.4f},{y:.4f},{yaw_gyro:.2f},{yaw_mag:.2f},{yaw_ekf:.2f},{seq},{tick_ms},{ring_count},{pending4},{q_drop},{q_depth}\n")
            print(f"[INS] Saved {len(self.ins_pts)} points to {os.path.basename(ins_fp)}")
        except Exception as e:
            print(f"[INS] 保存失败: {e}")

    def _auto_save_points(self):
        if not self.save_points:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"save_{self.save_count}_{ts}.txt")
        try:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Save point data\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Session: {self.save_count}\n")
                f.write(f"# Total points: {len(self.save_points)}\n")
                f.write(f"# Format: index,x,y,yaw(rad),speed_dir(1=fwd,0=rev),tick_ms,q_drop,ring_count,pending4\n")
                f.write(f"#----------------------------------------\n")
                for i, (idx, x, y, yaw, sd) in enumerate(self.save_points):
                    _, tick_ms, q_drop, ring_count, pending4 = self.save_diag[i] if i < len(self.save_diag) else (idx, 0, 0, 0, 0)
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f},{sd:.0f},{tick_ms},{q_drop},{ring_count},{pending4}\n")
            self._save_rtk_data(fp, "SAVE")
            self._save_ins_data(fp, "SAVE")
            self._update_info(f"已保存: {os.path.basename(fp)}\n点数: {len(self.save_points)}")
        except Exception as e:
            messagebox.showerror("错误", f"保存失败: {e}")

    def _auto_save_follow(self):
        if not self.follow_pts:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"follow_{self.save_count}_{ts}.txt")
        try:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Follow trajectory data\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Session: {self.save_count}\n")
                f.write(f"# Total points: {len(self.follow_pts)}\n")
                f.write(f"# Format: idx,x,y,yaw,target_idx\n")
                f.write(f"#----------------------------------------\n")
                for pt in self.follow_pts:
                    target_idx = pt[4] if len(pt) > 4 else 0
                    f.write(f"{pt[0]},{pt[1]:.4f},{pt[2]:.4f},{pt[3]:.4f},{target_idx}\n")
            self._save_rtk_data(fp, "FOLLOW")
            self._save_ins_data(fp, "FOLLOW")
            self._update_info(f"已保存: {os.path.basename(fp)}\n点数: {len(self.follow_pts)}")
        except Exception as e:
            messagebox.showerror("错误", f"保存失败: {e}")

    def _auto_save_points_subj3(self, phase):
        """科目三 specific save function - DRIVE phase"""
        if not self.save_points:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"subj3_{phase}_{ts}.txt")
        try:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# 科目三 trajectory data - {phase.upper()} phase\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 科目: 3\n")
                f.write(f"# Phase: {phase.upper()}\n")
                f.write(f"# Total points: {len(self.save_points)}\n")
                f.write(f"# Park yaw: {self.park_yaw:.4f} rad ({np.degrees(self.park_yaw):.1f} deg)\n")
                f.write(f"# Format: index,x,y,yaw(rad),speed_dir(1=fwd,0=rev),tick_ms,q_drop,ring_count,pending4\n")
                f.write(f"#----------------------------------------\n")
                for i, (idx, x, y, yaw, sd) in enumerate(self.save_points):
                    _, tick_ms, q_drop, ring_count, pending4 = self.save_diag[i] if i < len(self.save_diag) else (idx, 0, 0, 0, 0)
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f},{sd:.0f},{tick_ms},{q_drop},{ring_count},{pending4}\n")
            self._save_rtk_data(fp, phase.upper())
            self._save_ins_data(fp, phase.upper())
            self._update_info(f"已保存: {os.path.basename(fp)}\n点数: {len(self.save_points)}")
        except Exception as e:
            messagebox.showerror("错误", f"保存失败: {e}")
            
    def _auto_save_follow_subj3(self):
        """科目三 specific save function - FOLLOW phase"""
        if not self.follow_pts:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"subj3_follow_{ts}.txt")
        try:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# 科目三 trajectory data - FOLLOW phase\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 科目: 3\n")
                f.write(f"# Phase: FOLLOW (reverse)\n")
                f.write(f"# Total points: {len(self.follow_pts)}\n")
                f.write(f"# Park yaw: {self.park_yaw:.4f} rad ({np.degrees(self.park_yaw):.1f} deg)\n")
                f.write(f"# Yaw offset: {self.yaw_offset:.4f} rad ({np.degrees(self.yaw_offset):.1f} deg)\n")
                f.write(f"# Format: idx,x,y,yaw,target_idx\n")
                f.write(f"#----------------------------------------\n")
                for pt in self.follow_pts:
                    target_idx = pt[4] if len(pt) > 4 else 0
                    f.write(f"{pt[0]},{pt[1]:.4f},{pt[2]:.4f},{pt[3]:.4f},{target_idx}\n")
            self._save_rtk_data(fp, "FOLLOW")
            self._save_ins_data(fp, "FOLLOW")
        except Exception as e:
            messagebox.showerror("错误", f"保存失败: {e}")

    def _analyze_and_save_subj3(self):
        """Analyze follow trajectory accuracy and save data"""
        if not self.follow_pts or not self.save_points:
            self._auto_save_follow_subj3()
            return

        # Extract car positions and target points
        car_pts = [(p[1], p[2]) for p in self.follow_pts if p[0] == 0]
        if not car_pts:
            self._auto_save_follow_subj3()
            return

        # Save endpoint vs car start position
        save_end = (self.save_points[-1][1], self.save_points[-1][2])
        save_start = (self.save_points[0][1], self.save_points[0][2])
        car_start = car_pts[0]
        car_end = car_pts[-1]

        # Calculate distances
        start_gap = np.sqrt((car_start[0] - save_end[0])**2 + (car_start[1] - save_end[1])**2)
        end_gap = np.sqrt((car_end[0] - save_start[0])**2 + (car_end[1] - save_start[1])**2)

        # Calculate path length
        save_path = sum(np.sqrt((self.save_points[i][1] - self.save_points[i-1][1])**2 +
                                (self.save_points[i][2] - self.save_points[i-1][2])**2)
                        for i in range(1, len(self.save_points)))
        car_path = sum(np.sqrt((car_pts[i][0] - car_pts[i-1][0])**2 +
                               (car_pts[i][1] - car_pts[i-1][1])**2)
                       for i in range(1, len(car_pts)))

        # Calculate lateral deviation for each car point (signed: positive = outside of curve)
        lat_devs = []
        for cx, cy in car_pts:
            min_dist = float('inf')
            best_proj = None
            best_seg_idx = 0
            for i in range(len(self.save_points) - 1):
                x1, y1 = self.save_points[i][1], self.save_points[i][2]
                x2, y2 = self.save_points[i+1][1], self.save_points[i+1][2]
                dx, dy = x2 - x1, y2 - y1
                seg_len_sq = dx * dx + dy * dy
                if seg_len_sq < 1e-12:
                    dist = np.sqrt((cx - x1)**2 + (cy - y1)**2)
                    proj = (x1, y1)
                else:
                    t = max(0, min(1, ((cx - x1) * dx + (cy - y1) * dy) / seg_len_sq))
                    px, py = x1 + t * dx, y1 + t * dy
                    dist = np.sqrt((cx - px)**2 + (cy - py)**2)
                    proj = (px, py)
                if dist < min_dist:
                    min_dist = dist
                    best_proj = proj
                    best_seg_idx = i
            # Determine sign: positive = car is to the left of path direction (outside of curve)
            if best_proj is not None:
                seg_dx = self.save_points[best_seg_idx+1][1] - self.save_points[best_seg_idx][1]
                seg_dy = self.save_points[best_seg_idx+1][2] - self.save_points[best_seg_idx][2]
                # Cross product: positive = car is to the left (outside of right turn)
                cross = seg_dx * (cy - best_proj[1]) - seg_dy * (cx - best_proj[0])
                signed_dist = min_dist if cross > 0 else -min_dist
            else:
                signed_dist = 0.0
            lat_devs.append(signed_dist)

        max_lat_dev = max(abs(d) for d in lat_devs) if lat_devs else 0.0
        mean_lat_dev = np.mean(lat_devs) if lat_devs else 0.0
        # Positive mean = car tends to be outside (corner cutting)
        outside_pct = sum(1 for d in lat_devs if d > 0.05) / len(lat_devs) * 100 if lat_devs else 0

        # Detect corners in save path (heading change > threshold)
        corner_threshold_deg = 15.0
        corners = []
        for i in range(1, len(self.save_points) - 1):
            dx1 = self.save_points[i][1] - self.save_points[i-1][1]
            dy1 = self.save_points[i][2] - self.save_points[i-1][2]
            dx2 = self.save_points[i+1][1] - self.save_points[i][1]
            dy2 = self.save_points[i+1][2] - self.save_points[i][2]
            a1 = math.atan2(dy1, dx1)
            a2 = math.atan2(dy2, dx2)
            da = math.degrees(a2 - a1)
            if da > 180: da -= 360
            if da < -180: da += 360
            if abs(da) > corner_threshold_deg:
                corners.append((i, da, self.save_points[i][1], self.save_points[i][2]))

        # Progress percentage
        progress = (1 - end_gap / save_path) * 100 if save_path > 0 else 0

        # Build analysis report
        report = (
            f"=== 科目三 Analysis ===\n"
            f"Save path: {save_path:.2f}m ({len(self.save_points)} pts)\n"
            f"Car path: {car_path:.2f}m ({len(car_pts)} pts)\n"
            f"Start gap: {start_gap:.3f}m\n"
            f"End gap: {end_gap:.3f}m\n"
            f"Max lat dev: {max_lat_dev:.3f}m\n"
            f"Mean lat dev: {mean_lat_dev:+.3f}m ({'outside' if mean_lat_dev > 0 else 'inside'})\n"
            f"Outside ratio: {outside_pct:.0f}%\n"
            f"Corners: {len(corners)}\n"
            f"Progress: {progress:.0f}%\n"
            f"Yaw offset: {np.degrees(self.yaw_offset):.1f} deg"
        )
        self._update_info(report)
        print(report)

        # Save with analysis header
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"subj3_follow_{ts}.txt")
        try:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# 科目三 trajectory data - FOLLOW phase\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# 科目: 3\n")
                f.write(f"# Phase: FOLLOW (reverse)\n")
                f.write(f"# Total points: {len(self.follow_pts)}\n")
                f.write(f"# Park yaw: {self.park_yaw:.4f} rad ({np.degrees(self.park_yaw):.1f} deg)\n")
                f.write(f"# Yaw offset: {self.yaw_offset:.4f} rad ({np.degrees(self.yaw_offset):.1f} deg)\n")
                f.write(f"# Save path: {save_path:.2f}m ({len(self.save_points)} pts)\n")
                f.write(f"# Car path: {car_path:.2f}m ({len(car_pts)} pts)\n")
                f.write(f"# Start gap: {start_gap:.3f}m\n")
                f.write(f"# End gap: {end_gap:.3f}m\n")
                f.write(f"# Max lateral deviation: {max_lat_dev:.3f}m\n")
                f.write(f"# Mean lateral deviation: {mean_lat_dev:+.3f}m\n")
                f.write(f"# Outside ratio: {outside_pct:.0f}%\n")
                f.write(f"# Corners detected: {len(corners)}\n")
                for ci, (idx, da, cx, cy) in enumerate(corners):
                    f.write(f"#   Corner {ci+1}: idx={idx} angle={da:+.1f}deg pos=({cx:.2f},{cy:.2f})\n")
                f.write(f"# Progress: {progress:.0f}%\n")
                f.write(f"# Format: idx,x,y,yaw,target_idx\n")
                f.write(f"#----------------------------------------\n")
                for pt in self.follow_pts:
                    target_idx = pt[4] if len(pt) > 4 else 0
                    f.write(f"{pt[0]},{pt[1]:.4f},{pt[2]:.4f},{pt[3]:.4f},{target_idx}\n")
            self._save_rtk_data(fp, "FOLLOW")
            self._save_ins_data(fp, "FOLLOW")
        except Exception as e:
            messagebox.showerror("错误", f"保存失败: {e}")

    def _export_save(self):
        if not self.save_points:
            messagebox.showwarning("警告", "没有存点数据")
            return
        fp = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text", "*.txt")],
                                          initialfile=f"save_{self.save_count}.txt")
        if fp:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Save point data - Session {self.save_count}\n")
                f.write(f"# Total: {len(self.save_points)}\n")
                f.write(f"# Format: index,x,y,yaw,speed_dir,tick_ms,q_drop,ring_count,pending4\n")
                for i, (idx, x, y, yaw, sd) in enumerate(self.save_points):
                    _, tick_ms, q_drop, ring_count, pending4 = self.save_diag[i] if i < len(self.save_diag) else (idx, 0, 0, 0, 0)
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f},{sd:.0f},{tick_ms},{q_drop},{ring_count},{pending4}\n")
            messagebox.showinfo("完成", f"已导出: {fp}")

    def _export_follow(self):
        if not self.follow_pts:
            messagebox.showwarning("警告", "没有循迹数据")
            return
        fp = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text", "*.txt")],
                                          initialfile=f"follow_{self.save_count}.txt")
        if fp:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Follow trajectory - Session {self.save_count}\n")
                f.write(f"# Total: {len(self.follow_pts)}\n")
                f.write(f"# Format: idx,x,y,yaw,target_idx\n")
                for pt in self.follow_pts:
                    target_idx = pt[4] if len(pt) > 4 else 0
                    f.write(f"{pt[0]},{pt[1]:.4f},{pt[2]:.4f},{pt[3]:.4f},{target_idx}\n")
            messagebox.showinfo("完成", f"已导出: {fp}")

    def _update_info(self, text):
        self.stats_text.config(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, text)
        self.stats_text.config(state=tk.DISABLED)

    def _on_close(self):
        if self.serial_reader:
            self.serial_reader.stop()
        self.destroy()


if __name__ == "__main__":
    app = TrackPlotter()
    if "--load-latest" in sys.argv:
        app._load_latest_data()
    if "--save-plot" in sys.argv:
        app.update_idletasks()
        app.update()
        app._save_plot_png()
    if "--save-window-shot" in sys.argv:
        app.update_idletasks()
        app.update()
        app._save_window_screenshot()
    if "--exit-after-save" in sys.argv:
        app.destroy()
    else:
        app.mainloop()

