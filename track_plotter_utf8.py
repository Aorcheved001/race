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
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import matplotlib
matplotlib.use('TkAgg')
# Use default fonts to avoid encoding issues
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

SERIAL_PORT = 'COM24'
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# Regex
RE_SAVE = re.compile(r'S:(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.?\d*)')
RE_FOLLOW_IDX = re.compile(r'F:(\d+),(-?\d+\.\d+),(-?\d+\.\d+)(?:,(-?\d+\.\d+))?')
RE_DEBUG = re.compile(r'D:(\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)(?:,(-?\d+\.\d+))?')
RE_POSITION = re.compile(r'\((-?\d+\.\d+),(-?\d+\.\d+)(?:,(-?\d+\.\d+))?\)')

# Throttle interval (seconds) - max redraw rate
DRAW_THROTTLE = 0.2  # 5 FPS max


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
            print(f"[Serial] Connected {self.port} @ {self.baud}")
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
            print(f"[Serial] Failed: {e}")
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
        self.title("Track Plotter - Wireless UART")
        self.geometry("1200x800")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Subject mode: 1 or 3
        self.subject_mode = tk.IntVar(value=1)
        
        # Subject 3 state machine
        self.subject3_state = "IDLE"  # IDLE, DRIVE, PARK, ADJUST, FOLLOW, ARRIVE, ERROR
        self.park_yaw = 0.0
        self.yaw_offset = 0.0
        
        self.mode = "idle"
        self.save_points = []
        self.follow_pts = []
        self.debug_pts = []  # (idx, x, y, pp_target_deg, actual_deg)
        self.save_count = 0
        self.serial_reader = None
        self.is_connected = False

        # Plot state
        self._drag_start = None
        self._hover_annot = None
        self._last_draw_time = 0
        self._draw_pending = False

        self._create_widgets()
        self._setup_plot_interaction()

    def _create_widgets(self):
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        ctrl = ttk.LabelFrame(main_frame, text="Control", width=250)
        ctrl.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        ctrl.pack_propagate(False)

        # Serial
        sf = ttk.LabelFrame(ctrl, text="Serial")
        sf.pack(fill=tk.X, padx=5, pady=5)
        pf = ttk.Frame(sf); pf.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(pf, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=SERIAL_PORT)
        ttk.Entry(pf, textvariable=self.port_var, width=10).pack(side=tk.LEFT, padx=5)
        bf = ttk.Frame(sf); bf.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(bf, text="Baud:").pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value=str(SERIAL_BAUD))
        ttk.Entry(bf, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=5)
        btnf = ttk.Frame(sf); btnf.pack(fill=tk.X, padx=5, pady=5)
        self.connect_btn = ttk.Button(btnf, text="Connect", command=self._toggle_conn)
        self.connect_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.status_lbl = ttk.Label(sf, text="Disconnected", foreground="gray")
        self.status_lbl.pack(padx=5, pady=2)

        # Subject Selection
        sf_mode = ttk.LabelFrame(ctrl, text="Subject Selection")
        sf_mode.pack(fill=tk.X, padx=5, pady=5)
        ttk.Radiobutton(sf_mode, text="Subject 1 (Simple Mode)", variable=self.subject_mode, 
                        value=1, command=self._on_subject_change).pack(anchor=tk.W, padx=5, pady=2)
        ttk.Radiobutton(sf_mode, text="Subject 3 (Maze Task)", variable=self.subject_mode, 
                        value=3, command=self._on_subject_change).pack(anchor=tk.W, padx=5, pady=2)

        # Mode - Subject 1 (Simple)
        self.mf_subj1 = ttk.LabelFrame(ctrl, text="Subject 1 Control")
        self.mf_subj1.pack(fill=tk.X, padx=5, pady=5)
        self.btn_start_save = ttk.Button(self.mf_subj1, text="Start Save", command=self._start_save, state=tk.DISABLED)
        self.btn_start_save.pack(fill=tk.X, padx=5, pady=2)
        self.btn_stop_save = ttk.Button(self.mf_subj1, text="Stop Save", command=self._stop_save, state=tk.DISABLED)
        self.btn_stop_save.pack(fill=tk.X, padx=5, pady=2)
        self.btn_start_follow = ttk.Button(self.mf_subj1, text="Start Follow", command=self._start_follow, state=tk.DISABLED)
        self.btn_start_follow.pack(fill=tk.X, padx=5, pady=2)
        self.btn_stop_follow = ttk.Button(self.mf_subj1, text="Stop Follow", command=self._stop_follow, state=tk.DISABLED)
        self.btn_stop_follow.pack(fill=tk.X, padx=5, pady=2)
        self.btn_back = ttk.Button(self.mf_subj1, text="Back to Save Mode", command=self._back_to_save, state=tk.DISABLED)
        self.btn_back.pack(fill=tk.X, padx=5, pady=2)

        # Mode - Subject 3 (Maze Task)
        self.mf_subj3 = ttk.LabelFrame(ctrl, text="Subject 3 State Machine")
        self.mf_subj3.pack(fill=tk.X, padx=5, pady=5)
        self.btn_subj3_drive = ttk.Button(self.mf_subj3, text="Start Drive (DRIVE)", 
                                          command=self._subj3_start_drive, state=tk.DISABLED)
        self.btn_subj3_drive.pack(fill=tk.X, padx=5, pady=2)
        self.btn_subj3_park = ttk.Button(self.mf_subj3, text="Arrive Park (PARK)", 
                                         command=self._subj3_to_park, state=tk.DISABLED)
        self.btn_subj3_park.pack(fill=tk.X, padx=5, pady=2)
        self.btn_subj3_adjust = ttk.Button(self.mf_subj3, text="Confirm Adjust (ADJUST)", 
                                           command=self._subj3_adjust, state=tk.DISABLED)
        self.btn_subj3_adjust.pack(fill=tk.X, padx=5, pady=2)
        self.btn_subj3_follow = ttk.Button(self.mf_subj3, text="Start Follow (FOLLOW)", 
                                           command=self._subj3_start_follow, state=tk.DISABLED)
        self.btn_subj3_follow.pack(fill=tk.X, padx=5, pady=2)
        self.btn_subj3_reset = ttk.Button(self.mf_subj3, text="Reset (IDLE)", 
                                          command=self._subj3_reset, state=tk.DISABLED)
        self.btn_subj3_reset.pack(fill=tk.X, padx=5, pady=2)

        # Status
        stf = ttk.LabelFrame(ctrl, text="Status")
        stf.pack(fill=tk.X, padx=5, pady=5)
        self.subject_lbl = ttk.Label(stf, text="Subject: 1", font=('Arial', 10, 'bold'), foreground='blue')
        self.subject_lbl.pack(padx=5, pady=2, anchor=tk.W)
        self.mode_lbl = ttk.Label(stf, text="Mode: Idle", font=('Arial', 10, 'bold'))
        self.mode_lbl.pack(padx=5, pady=2, anchor=tk.W)
        self.subj3_state_lbl = ttk.Label(stf, text="State: IDLE", font=('Arial', 9), foreground='gray')
        self.subj3_state_lbl.pack(padx=5, pady=2, anchor=tk.W)
        self.pts_lbl = ttk.Label(stf, text="Save points: 0")
        self.pts_lbl.pack(padx=5, pady=2, anchor=tk.W)
        self.flw_lbl = ttk.Label(stf, text="Follow points: 0")
        self.flw_lbl.pack(padx=5, pady=2, anchor=tk.W)
        self.yaw_offset_lbl = ttk.Label(stf, text="Yaw Offset: 0.0 deg")
        self.yaw_offset_lbl.pack(padx=5, pady=2, anchor=tk.W)
        
        # Now update subject UI after all widgets are created
        self._update_subject_ui()

        # Stats
        saf = ttk.LabelFrame(ctrl, text="Info")
        saf.pack(fill=tk.X, padx=5, pady=5)
        self.stats_text = tk.Text(saf, height=6, width=30, state=tk.DISABLED)
        self.stats_text.pack(padx=5, pady=5, fill=tk.X)

        # File
        ff = ttk.LabelFrame(ctrl, text="File")
        ff.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(ff, text="Export Save Data", command=self._export_save).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(ff, text="Export Follow Data", command=self._export_follow).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(ff, text="Reset View", command=self._reset_view).pack(fill=tk.X, padx=5, pady=2)

        # Plot
        pf2 = ttk.LabelFrame(main_frame, text="Trajectory")
        pf2.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor='#f8f8f8')
        self.ax = self.fig.add_subplot(111)
        self._init_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=pf2)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _init_axes(self):
        self.ax.set_xlabel('X (m)', fontsize=11)
        self.ax.set_ylabel('Y (m)', fontsize=11)
        self.ax.set_title('Trajectory', fontsize=13, fontweight='bold')
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
        if event.inaxes != self.ax:
            if self._hover_annot:
                self._hover_annot.set_visible(False)
                self.canvas.draw_idle()
            return
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
        min_dist = 20
        nearest = None
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
            messagebox.showerror("Error", "Baud must be a number")
            return
        self.serial_reader = SerialReader(port, baud, self._on_data)
        self.serial_reader.start()
        self.after(500, self._check_conn)

    def _check_conn(self):
        if self.serial_reader and self.serial_reader.running:
            if self.serial_reader.ser and self.serial_reader.ser.is_open:
                self.is_connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_lbl.config(text="Connected", foreground="green")
                self._enable_initial_btns()
                return
        self.is_connected = False
        self.status_lbl.config(text="Failed", foreground="red")

    def _disconnect(self):
        if self.serial_reader:
            self.serial_reader.stop()
            self.serial_reader = None
        self.is_connected = False
        self.connect_btn.config(text="Connect")
        self.status_lbl.config(text="Disconnected", foreground="gray")
        self._disable_btns()

    def _disable_btns(self):
        self.btn_start_save.config(state=tk.DISABLED)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.DISABLED)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        # Subject 3 buttons
        self.btn_subj3_drive.config(state=tk.DISABLED)
        self.btn_subj3_park.config(state=tk.DISABLED)
        self.btn_subj3_adjust.config(state=tk.DISABLED)
        self.btn_subj3_follow.config(state=tk.DISABLED)
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
            # Subject 1: show simple controls, hide Subject 3
            self.mf_subj1.pack(fill=tk.X, padx=5, pady=5)
            self.mf_subj3.pack_forget()
            self.subject_lbl.config(text="Subject: 1", foreground='blue')
            self.subj3_state_lbl.pack_forget()
            self.yaw_offset_lbl.pack_forget()
        else:
            # Subject 3: hide simple controls, show Subject 3
            self.mf_subj1.pack_forget()
            self.mf_subj3.pack(fill=tk.X, padx=5, pady=5)
            self.subject_lbl.config(text="Subject: 3", foreground='purple')
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
        self.debug_pts.clear()
        self._update_subj3_state_display()
        self._clear_plot()
        
    # ---- Subject 3 State Machine ----
    def _subj3_start_drive(self):
        """Subject 3: Start manual drive + recording"""
        self.subject3_state = "DRIVE"
        self.mode = "save"
        self.save_points.clear()
        self.follow_pts.clear()
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="Mode: Saving", foreground="blue")
        self._clear_plot()
        
    def _subj3_to_park(self):
        """Subject 3: Arrive at parking area, record parking yaw"""
        # Record the last point's yaw as parking yaw
        if self.save_points:
            self.park_yaw = self.save_points[-1][3]  # (idx, x, y, yaw, speed_dir)
        
        self.subject3_state = "PARK"
        self.mode = "idle"
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="Mode: Idle", foreground="black")
        self._auto_save_points_subj3("drive")
        
    def _subj3_adjust(self):
        """Subject 3: Confirm head adjustment, calculate yaw offset"""
        # Get current yaw (last follow point or save point's yaw)
        current_yaw = 0.0
        if self.follow_pts:
            current_yaw = self.follow_pts[-1][3]
        elif self.save_points:
            current_yaw = self.save_points[-1][3]
        
        self.yaw_offset = current_yaw - self.park_yaw
        self.yaw_offset_lbl.config(text=f"Yaw Offset: {np.degrees(self.yaw_offset):.1f} deg")
        
        self.subject3_state = "ADJUST"
        self._update_subj3_btns()
        self._update_subj3_state_display()
        
    def _subj3_start_follow(self):
        """Subject 3: Start reverse follow"""
        self.subject3_state = "FOLLOW"
        self.mode = "follow"
        self.follow_pts.clear()
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="Mode: Following", foreground="red")
        self._clear_plot()
        if self.save_points:
            self._draw_save_ref()
            
    def _subj3_reset(self):
        """Subject 3: Reset to IDLE state"""
        # Save follow data
        if self.subject3_state == "FOLLOW" and self.follow_pts:
            self._auto_save_follow_subj3()
        
        self.subject3_state = "IDLE"
        self.mode = "idle"
        self.park_yaw = 0.0
        self.yaw_offset = 0.0
        self.yaw_offset_lbl.config(text="Yaw Offset: 0.0 deg")
        self.save_points.clear()
        self.follow_pts.clear()
        self._update_subj3_btns()
        self._update_subj3_state_display()
        self.mode_lbl.config(text="Mode: Idle", foreground="black")
        self._clear_plot()
        
    def _update_subj3_btns(self):
        """Update Subject 3 button states"""
        state = self.subject3_state
        
        # Reset all buttons
        self.btn_subj3_drive.config(state=tk.DISABLED)
        self.btn_subj3_park.config(state=tk.DISABLED)
        self.btn_subj3_adjust.config(state=tk.DISABLED)
        self.btn_subj3_follow.config(state=tk.DISABLED)
        self.btn_subj3_reset.config(state=tk.DISABLED)
        
        if state == "IDLE":
            self.btn_subj3_drive.config(state=tk.NORMAL)
        elif state == "DRIVE":
            self.btn_subj3_park.config(state=tk.NORMAL)
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "PARK":
            self.btn_subj3_adjust.config(state=tk.NORMAL)
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "ADJUST":
            self.btn_subj3_follow.config(state=tk.NORMAL)
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "FOLLOW":
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "ARRIVE":
            self.btn_subj3_reset.config(state=tk.NORMAL)
        elif state == "ERROR":
            self.btn_subj3_reset.config(state=tk.NORMAL)
            
    def _update_subj3_state_display(self):
        """Update Subject 3 state display"""
        state_colors = {
            "IDLE": "gray",
            "DRIVE": "blue",
            "PARK": "orange",
            "ADJUST": "purple",
            "FOLLOW": "red",
            "ARRIVE": "green",
            "ERROR": "red"
        }
        color = state_colors.get(self.subject3_state, "gray")
        self.subj3_state_lbl.config(text=f"State: {self.subject3_state}", foreground=color)

    # ---- Data ----
    def _on_data(self, line):
        self.after(0, self._process, line)

    def _process(self, line):
        if line.startswith("__ERROR__"):
            self.status_lbl.config(text=f"Error: {line[9:]}", foreground="red")
            self.is_connected = False
            self.connect_btn.config(text="Connect")
            return

        if 'S:' in line:
            for m in RE_SAVE.finditer(line):
                try:
                    idx = int(m.group(1))
                    x = float(m.group(2))
                    y = float(m.group(3))
                    yaw = float(m.group(4))
                    sd = float(m.group(5))
                    self.save_points.append((idx, x, y, yaw, sd))
                    self.pts_lbl.config(text=f"Save points: {len(self.save_points)}")
                    if self.mode == "save":
                        self._request_draw()
                except:
                    pass

        if 'F:' in line:
            for m in RE_FOLLOW_IDX.finditer(line):
                try:
                    idx = int(m.group(1))
                    x = float(m.group(2))
                    y = float(m.group(3))
                    yaw = float(m.group(4)) if m.group(4) else 0.0
                    self.follow_pts.append((idx, x, y, yaw))
                    self.flw_lbl.config(text=f"Follow: {len(self.follow_pts)} (idx {idx})")
                    if self.mode == "follow":
                        self._request_draw()
                except:
                    pass

        # D:idx,x,y,pp_target,actual,yaw (steer debug)
        if 'D:' in line:
            for m in RE_DEBUG.finditer(line):
                try:
                    idx = int(m.group(1))
                    x = float(m.group(2))
                    y = float(m.group(3))
                    pp_tgt = float(m.group(4))
                    actual = float(m.group(5))
                    yaw = float(m.group(6)) if m.group(6) else 0.0
                    self.follow_pts.append((idx, x, y, yaw))
                    self.debug_pts.append((idx, x, y, pp_tgt, actual))
                    self.flw_lbl.config(text=f"Follow: {len(self.follow_pts)} (idx {idx}) | PP:{pp_tgt:.1f} Act:{actual:.1f}")
                    self._request_draw()
                except:
                    pass

        if self.mode == 'follow' and '(' in line and ')' in line:
            for m in RE_POSITION.finditer(line):
                try:
                    x = float(m.group(1))
                    y = float(m.group(2))
                    yaw = float(m.group(3)) if m.group(3) else 0.0
                    if abs(x) < 1e-6 and abs(y) < 1e-6:
                        continue
                    if self.follow_pts and x == self.follow_pts[-1][1] and y == self.follow_pts[-1][2]:
                        continue
                    self.follow_pts.append((0, x, y, yaw))
                    self.flw_lbl.config(text=f"Follow: {len(self.follow_pts)}")
                    self._request_draw()
                except:
                    pass

        if "TrackInfo:TotalPoints=" in line:
            try:
                self.pts_lbl.config(text=f"Save points: {line.split('=')[1]}")
            except:
                pass

    def _request_draw(self):
        """Throttled draw request - schedules a redraw if not too frequent"""
        now = time.time()
        if now - self._last_draw_time >= DRAW_THROTTLE:
            self._last_draw_time = now
            self._do_redraw()
        elif not self._draw_pending:
            self._draw_pending = True
            delay_ms = int((DRAW_THROTTLE - (now - self._last_draw_time)) * 1000) + 10
            self.after(delay_ms, self._flush_draw)

    def _flush_draw(self):
        self._draw_pending = False
        self._last_draw_time = time.time()
        self._do_redraw()

    def _do_redraw(self):
        """Full redraw of current data"""
        self.ax.clear()
        self._init_axes()
        self._hover_annot = None

        if self.mode == "save":
            self._draw_save_content()
        elif self.mode == "follow":
            self._draw_follow_content()
        else:
            # idle: show whatever we have
            if self.save_points:
                self._draw_save_content()
            elif self.follow_pts:
                self._draw_follow_content()

        self._auto_fit()
        self.canvas.draw()

    def _draw_save_content(self):
        if not self.save_points:
            return
        xs = [p[1] for p in self.save_points]
        ys = [p[2] for p in self.save_points]
        n = len(self.save_points)
        self.ax.set_title(f'Save Trajectory ({n} points)', fontsize=13, fontweight='bold')
        self.ax.plot(xs, ys, '-', color='#2196F3', linewidth=1.5, alpha=0.6, zorder=2)
        self.ax.scatter(xs, ys, c='#E53935', s=40, zorder=5, edgecolors='white', linewidths=0.5)
        step = max(1, n // 15)
        for i, (idx, x, y, _, _) in enumerate(self.save_points):
            if i % step == 0 or i == n - 1:
                self.ax.annotate(str(idx), (x, y), textcoords="offset points",
                                 xytext=(6, 6), fontsize=7, color='#B71C1C', alpha=0.8)
        if n >= 1:
            self.ax.scatter([xs[0]], [ys[0]], c='#4CAF50', s=120, marker='^', zorder=7, edgecolors='white', linewidths=1)
            self.ax.annotate('Start', (xs[0], ys[0]), textcoords="offset points", xytext=(12, 8), fontsize=10, color='#2E7D32', fontweight='bold')
        if n >= 2:
            self.ax.scatter([xs[-1]], [ys[-1]], c='#9C27B0', s=120, marker='v', zorder=7, edgecolors='white', linewidths=1)
            self.ax.annotate('End', (xs[-1], ys[-1]), textcoords="offset points", xytext=(12, -12), fontsize=10, color='#6A1B9A', fontweight='bold')

    def _draw_follow_content(self):
        if not self.follow_pts:
            return
        # Save reference
        if self.save_points:
            sx = [p[1] for p in self.save_points]
            sy = [p[2] for p in self.save_points]
            self.ax.plot(sx, sy, '--', color='#BDBDBD', linewidth=1, alpha=0.5, label='Save ref', zorder=1)
            self.ax.scatter(sx, sy, c='#E0E0E0', s=15, alpha=0.4, zorder=1)

        # Separate follow data into two categories:
        #   idx=0: car's current position (from INS)
        #   idx>0: target point from stored trajectory
        car_pts = [(p[1], p[2]) for p in self.follow_pts if p[0] == 0]
        tgt_pts = [(p[1], p[2], p[0]) for p in self.follow_pts if p[0] > 0]

        n = len(self.follow_pts)
        self.ax.set_title(f'Follow Trajectory ({n} pts, car:{len(car_pts)}, tgt:{len(tgt_pts)})', fontsize=13, fontweight='bold')

        # Draw car path: connect only car current positions (idx=0)
        if car_pts:
            cx = [p[0] for p in car_pts]
            cy = [p[1] for p in car_pts]
            self.ax.plot(cx, cy, '-', color='#1565C0', linewidth=2.5, label='Car path', zorder=3)
            self.ax.scatter([cx[0]], [cy[0]], c='#4CAF50', s=120, marker='^', zorder=7, edgecolors='white', linewidths=1)
            self.ax.scatter([cx[-1]], [cy[-1]], c='#FF5722', s=120, marker='o', zorder=7, edgecolors='white', linewidths=1)
            self.ax.annotate('Current', (cx[-1], cy[-1]), textcoords="offset points", xytext=(12, 8), fontsize=10, color='#BF360C', fontweight='bold')

        # Draw target points: connect only stored trajectory targets (idx>0)
        # Break line at big index jumps (direction switch causes target to jump)
        if tgt_pts:
            IDX_JUMP_THRESHOLD = 5  # break line if target index jumps more than this
            # Split into segments where consecutive indices are close
            segments = []
            seg_x, seg_y = [tgt_pts[0][0]], [tgt_pts[0][1]]
            for i in range(1, len(tgt_pts)):
                prev_idx = tgt_pts[i-1][2]
                curr_idx = tgt_pts[i][2]
                if abs(curr_idx - prev_idx) > IDX_JUMP_THRESHOLD:
                    # Big jump - save current segment and start new one
                    segments.append((seg_x[:], seg_y[:]))
                    seg_x, seg_y = [tgt_pts[i][0]], [tgt_pts[i][1]]
                else:
                    seg_x.append(tgt_pts[i][0])
                    seg_y.append(tgt_pts[i][1])
            segments.append((seg_x, seg_y))
            
            for i, (seg_x, seg_y) in enumerate(segments):
                label = 'Target path' if i == 0 else None
                self.ax.plot(seg_x, seg_y, '-', color='#FF9800', linewidth=1.5, alpha=0.7, label=label, zorder=2)
            # Scatter all target points
            tx = [p[0] for p in tgt_pts]
            ty = [p[1] for p in tgt_pts]
            self.ax.scatter(tx, ty, c='#FF9800', s=20, alpha=0.6, zorder=4)

        self.ax.legend(loc='upper right', fontsize=9, framealpha=0.9)

    # ---- Mode buttons ----
    def _start_save(self):
        self.mode = "save"
        self.save_points.clear()
        self.follow_pts.clear()
        self.btn_start_save.config(state=tk.DISABLED)
        self.btn_stop_save.config(state=tk.NORMAL)
        self.btn_start_follow.config(state=tk.DISABLED)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="Mode: Saving", foreground="blue")
        self.pts_lbl.config(text="Save points: 0")
        self._clear_plot()

    def _stop_save(self):
        self.mode = "idle"
        self.save_count += 1
        self.btn_start_save.config(state=tk.NORMAL)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.NORMAL)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="Mode: Idle", foreground="black")
        if self.save_points:
            self._auto_save_points()

    def _start_follow(self):
        self.mode = "follow"
        self.follow_pts.clear()
        self.btn_start_save.config(state=tk.DISABLED)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.DISABLED)
        self.btn_stop_follow.config(state=tk.NORMAL)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="Mode: Following", foreground="red")
        self.flw_lbl.config(text="Follow: 0")
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
        self.mode_lbl.config(text="Mode: Idle", foreground="black")
        if self.follow_pts:
            self._auto_save_follow()

    def _back_to_save(self):
        self.mode = "idle"
        self.btn_start_save.config(state=tk.NORMAL)
        self.btn_stop_save.config(state=tk.DISABLED)
        self.btn_start_follow.config(state=tk.NORMAL)
        self.btn_stop_follow.config(state=tk.DISABLED)
        self.btn_back.config(state=tk.DISABLED)
        self.mode_lbl.config(text="Mode: Idle", foreground="black")
        if self.follow_pts:
            self._auto_save_follow()

    def _clear_plot(self):
        self.ax.clear()
        self._init_axes()
        self._hover_annot = None
        self._last_draw_time = 0
        self._draw_pending = False
        self.canvas.draw()

    def _draw_save_ref(self):
        if not self.save_points:
            return
        sx = [p[1] for p in self.save_points]
        sy = [p[2] for p in self.save_points]
        self.ax.plot(sx, sy, '--', color='#BDBDBD', linewidth=1, alpha=0.5, label='Save ref', zorder=1)
        self.ax.scatter(sx, sy, c='#E0E0E0', s=15, alpha=0.4, zorder=1)
        self.ax.legend(loc='upper right', fontsize=9, framealpha=0.9)
        self.canvas.draw()

    # ---- File saving ----
    def _get_save_dir(self):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "track_data")
        os.makedirs(d, exist_ok=True)
        return d

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
                f.write(f"# Format: index,x,y,yaw(rad),speed_dir(1=fwd,0=rev)\n")
                f.write(f"#----------------------------------------\n")
                for idx, x, y, yaw, sd in self.save_points:
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f},{sd:.0f}\n")
            self._update_info(f"Saved: {os.path.basename(fp)}\nPoints: {len(self.save_points)}")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {e}")

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
                f.write(f"# Format: idx,x,y,yaw\n")
                f.write(f"#----------------------------------------\n")
                for idx, x, y, yaw in self.follow_pts:
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f}\n")
            self._update_info(f"Saved: {os.path.basename(fp)}\nPoints: {len(self.follow_pts)}")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {e}")

    def _auto_save_points_subj3(self, phase):
        """Subject 3 specific save function - DRIVE phase"""
        if not self.save_points:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"subj3_{phase}_{ts}.txt")
        try:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Subject 3 trajectory data - {phase.upper()} phase\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Subject: 3\n")
                f.write(f"# Phase: {phase.upper()}\n")
                f.write(f"# Total points: {len(self.save_points)}\n")
                f.write(f"# Park yaw: {self.park_yaw:.4f} rad ({np.degrees(self.park_yaw):.1f}��)\n")
                f.write(f"# Format: index,x,y,yaw(rad),speed_dir(1=fwd,0=rev)\n")
                f.write(f"#----------------------------------------\n")
                for idx, x, y, yaw, sd in self.save_points:
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f},{sd:.0f}\n")
            self._update_info(f"Saved: {os.path.basename(fp)}\nPoints: {len(self.save_points)}")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {e}")
            
    def _auto_save_follow_subj3(self):
        """Subject 3 specific save function - FOLLOW phase"""
        if not self.follow_pts:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fp = os.path.join(self._get_save_dir(), f"subj3_follow_{ts}.txt")
        try:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Subject 3 trajectory data - FOLLOW phase\n")
                f.write(f"# Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"# Subject: 3\n")
                f.write(f"# Phase: FOLLOW (reverse)\n")
                f.write(f"# Total points: {len(self.follow_pts)}\n")
                f.write(f"# Park yaw: {self.park_yaw:.4f} rad ({np.degrees(self.park_yaw):.1f}��)\n")
                f.write(f"# Yaw offset: {self.yaw_offset:.4f} rad ({np.degrees(self.yaw_offset):.1f}��)\n")
                f.write(f"# Format: idx,x,y,yaw\n")
                f.write(f"#----------------------------------------\n")
                for idx, x, y, yaw in self.follow_pts:
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f}\n")
            self._update_info(f"Saved: {os.path.basename(fp)}\nPoints: {len(self.follow_pts)}")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {e}")

    def _export_save(self):
        if not self.save_points:
            messagebox.showwarning("Warning", "No save data")
            return
        fp = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text", "*.txt")],
                                          initialfile=f"save_{self.save_count}.txt")
        if fp:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Save point data - Session {self.save_count}\n")
                f.write(f"# Total: {len(self.save_points)}\n")
                f.write(f"# Format: index,x,y,yaw,speed_dir\n")
                for idx, x, y, yaw, sd in self.save_points:
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f},{sd:.0f}\n")
            messagebox.showinfo("OK", f"Exported: {fp}")

    def _export_follow(self):
        if not self.follow_pts:
            messagebox.showwarning("Warning", "No follow data")
            return
        fp = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text", "*.txt")],
                                          initialfile=f"follow_{self.save_count}.txt")
        if fp:
            with open(fp, 'w', encoding='utf-8') as f:
                f.write(f"# Follow trajectory - Session {self.save_count}\n")
                f.write(f"# Total: {len(self.follow_pts)}\n")
                f.write(f"# Format: idx,x,y,yaw\n")
                for idx, x, y, yaw in self.follow_pts:
                    f.write(f"{idx},{x:.4f},{y:.4f},{yaw:.4f}\n")
            messagebox.showinfo("OK", f"Exported: {fp}")

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
    app.mainloop()
