#!/usr/bin/env python
# -*- coding: gbk -*-
"""
Real-time gyroyaw data receiver + first-layer Kalman filter analysis.

Receives: gyroyaw:seq,z_bias_deg,rp_bias_deg,yaw_model_deg,roll_deg,pitch_deg
Analyzes: drift rate, roll/pitch noise, yaw model vs gyro, steady-state Kalman gain,
          and recommends Q/R tuning for the 6-axis attitude Kalman filter.
"""

import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[WARN] matplotlib not available, running in text-only mode")


# ── Regex ──────────────────────────────────────────────────────────────────────
RE_GYROYAW = re.compile(
    r"gyroyaw:\s*"
    r"(\d+)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)\s*,\s*"
    r"([-+]?\d+(?:\.\d*)?)"
)

# ── Kalman steady-state analysis ──────────────────────────────────────────────

def kalman_steady_state_gain(Q, R):
    """Compute steady-state Kalman gain for scalar KF: x = x + T*u, P = P + Q, K = P/(P+R)."""
    # Steady state: P = P + Q - P^2/(P+R) => P^2 - Q*P - Q*R = 0
    # P_ss = (Q + sqrt(Q^2 + 4*Q*R)) / 2
    P_ss = (Q + np.sqrt(Q**2 + 4*Q*R)) / 2.0
    K_ss = P_ss / (P_ss + R)
    return P_ss, K_ss


def analyze_kalman_params(Q, R, T, acc_noise_deg, gyro_drift_deg_per_s):
    """
    Analyze first-layer Kalman filter parameters and provide tuning advice.

    The 6-axis Kalman filter has 3 decoupled scalar KFs:
    - Xk[0] = roll,  measurement = atan2(acc_y, acc_z)
    - Xk[1] = pitch, measurement = -atan2(acc_x, sqrt(acc_y^2+acc_z^2))
    - Xk[2] = yaw,   NO measurement (K[2]=0, pure integration)

    For roll/pitch:
    - Q represents gyro integration noise (process noise per step)
    - R represents accelerometer measurement noise variance
    - Steady-state gain K_ss = P_ss/(P_ss+R) determines trust balance

    For yaw:
    - K[2] = 0 always, so Q[2] and R[2] don't affect output
    - yaw is pure gyro integration, drift depends on gyro bias
    """
    P_ss, K_ss = kalman_steady_state_gain(Q, R)

    # Effective time constant (approximate)
    # K_ss ≈ Q/R for small Q/R, and the filter responds with time constant ~ R/Q * T
    tau = (R / Q) * T if Q > 0 else float('inf')

    # Noise reduction factor: output noise ≈ sqrt(K_ss * R) for measurement noise
    # and output tracks gyro with lag ~ tau
    output_acc_noise = np.sqrt(K_ss * R) * 180 / np.pi  # rough estimate in degrees

    results = {
        'Q': Q,
        'R': R,
        'T': T,
        'P_ss': P_ss,
        'K_ss': K_ss,
        'tau_s': tau,
        'acc_trust_pct': K_ss * 100,
        'gyro_trust_pct': (1 - K_ss) * 100,
    }
    return results


def recommend_params(roll_std_deg, pitch_std_deg, yaw_drift_deg_per_min,
                     T=0.004, current_Q=0.001, current_R=0.1):
    """
    Recommend Q and R based on observed data characteristics.

    Strategy:
    - R should match accelerometer noise variance (in rad^2)
    - Q should match gyro drift noise per step (in rad^2)
    - For roll/pitch: want K_ss ~ 0.05-0.15 (trust gyro 85-95%, use acc for drift correction)
    - If roll/pitch are too noisy → increase R (trust acc less)
    - If roll/pitch drift → decrease R or increase Q (let acc correct more)
    """
    # Convert observed noise to radians
    roll_std_rad = np.radians(roll_std_deg)
    pitch_std_rad = np.radians(pitch_std_deg)
    avg_attitude_std_rad = (roll_std_rad + pitch_std_rad) / 2.0

    # R should be ~ observed accelerometer noise variance
    # The accelerometer measurement noise in the KF is the variance of atan2(acc_y, acc_z)
    # For small angles, this is approximately (acc_noise / g)^2
    # We observe the output noise, which is filtered, so raw acc noise is higher
    # Rough: R ≈ (output_std / K_ss)^2, but K_ss depends on R...
    # Simpler: set R = observed_variance / K_target, iterate

    # Target: K_ss ≈ 0.1 (10% acc trust, 90% gyro trust)
    K_target = 0.10

    # From K_ss = P_ss/(P_ss+R) and P_ss ≈ Q*R/(R-Q) for small Q:
    # K_ss ≈ Q/(Q+R) for small Q
    # So R ≈ Q*(1-K_target)/K_target

    # Q: process noise per step. For gyro with bias drift:
    # yaw_drift_deg_per_min → bias rate in rad/s
    yaw_drift_rad_per_s = np.radians(yaw_drift_deg_per_min) / 60.0
    # Q should be ~ (gyro_noise_per_step)^2
    # gyro noise per step = gyro_noise_density * sqrt(T) + bias_drift * T
    # For IMU963RA: typical gyro noise ~0.01 deg/s/sqrt(Hz)
    gyro_noise_density = np.radians(0.01)  # rad/s/sqrt(Hz)
    gyro_noise_per_step = gyro_noise_density * np.sqrt(T) + yaw_drift_rad_per_s * T
    Q_recommended = gyro_noise_per_step ** 2

    # R: from observed attitude noise
    # If we see roll_std_deg noise in the output, and K_ss ≈ 0.1,
    # then raw measurement noise ≈ output_noise / sqrt(K_ss)
    R_from_observation = (avg_attitude_std_rad / np.sqrt(K_target)) ** 2

    # Also compute R from K_target and Q
    R_from_target = Q_recommended * (1 - K_target) / K_target

    # Use the larger R (more conservative)
    R_recommended = max(R_from_observation, R_from_target)

    # Clamp to reasonable range
    Q_recommended = np.clip(Q_recommended, 1e-5, 0.1)
    R_recommended = np.clip(R_recommended, 0.01, 10.0)

    return {
        'Q_recommended': Q_recommended,
        'R_recommended': R_recommended,
        'K_target': K_target,
        'Q_current': current_Q,
        'R_current': current_R,
    }


# ── Real-time plotter ─────────────────────────────────────────────────────────

class GyroYawAnalyzer:
    """Real-time analyzer for gyroyaw data."""

    def __init__(self, port, baud, max_points=3000):
        self.port = port
        self.baud = baud
        self.max_points = max_points

        # Data buffers
        self.t = []
        self.seq = []
        self.z_bias = []
        self.rp_bias = []
        self.yaw_model = []
        self.roll = []
        self.pitch = []

        # Analysis results
        self.analysis_done = False
        self.recommendation = None

        # Serial
        self.ser = None
        self.t0 = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        self.t0 = time.time()
        print(f"[PORT] {self.port} @ {self.baud}")
        print(f"[RECV] gyroyaw:seq,z_bias_deg,rp_bias_deg,yaw_model_deg,roll_deg,pitch_deg")
        print(f"[INFO] Waiting for data... (Ctrl+C to stop and analyze)\n")

    def read_line(self):
        if self.ser is None:
            return None
        try:
            line = self.ser.readline().decode("ascii", errors="ignore").strip()
            return line if line else None
        except Exception:
            return None

    def parse_line(self, line):
        m = RE_GYROYAW.search(line)
        if not m:
            return None
        now = time.time() - self.t0
        return {
            't': now,
            'seq': int(m.group(1)),
            'z_bias': float(m.group(2)),
            'rp_bias': float(m.group(3)),
            'yaw_model': float(m.group(4)),
            'roll': float(m.group(5)),
            'pitch': float(m.group(6)),
        }

    def add_data(self, d):
        self.t.append(d['t'])
        self.seq.append(d['seq'])
        self.z_bias.append(d['z_bias'])
        self.rp_bias.append(d['rp_bias'])
        self.yaw_model.append(d['yaw_model'])
        self.roll.append(d['roll'])
        self.pitch.append(d['pitch'])

        # Trim to max_points
        if len(self.t) > self.max_points:
            excess = len(self.t) - self.max_points
            self.t = self.t[excess:]
            self.seq = self.seq[excess:]
            self.z_bias = self.z_bias[excess:]
            self.rp_bias = self.rp_bias[excess:]
            self.yaw_model = self.yaw_model[excess:]
            self.roll = self.roll[excess:]
            self.pitch = self.pitch[excess:]

    def run_text_mode(self, duration_s=None):
        """Run in text-only mode, print analysis every N samples."""
        count = 0
        last_analysis = 0
        try:
            while True:
                if duration_s and (time.time() - self.t0) > duration_s:
                    break
                line = self.read_line()
                if line is None:
                    continue
                d = self.parse_line(line)
                if d is None:
                    continue
                self.add_data(d)
                count += 1

                # Print every 50 samples
                if count % 50 == 0:
                    print(
                        f"[{count:5d}] z={d['z_bias']:+8.2f}° rp={d['rp_bias']:+8.2f}° "
                        f"model={d['yaw_model']:+8.2f}° "
                        f"roll={d['roll']:+7.2f}° pitch={d['pitch']:+7.2f}°",
                        flush=True,
                    )

                # Analyze every 500 samples
                if count - last_analysis >= 500 and count >= 500:
                    self._print_analysis()
                    last_analysis = count

        except KeyboardInterrupt:
            pass
        finally:
            self._print_final_analysis()

    def run_plot_mode(self, duration_s=None):
        """Run with real-time matplotlib plot."""
        if not HAS_MPL:
            print("[ERROR] matplotlib not available, use text mode")
            return

        fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
        fig.suptitle("First-Layer Kalman Filter Analysis", fontsize=14)

        lines = {}
        labels = {
            0: ('z_bias_yaw (°)', 'Z-axis gyro yaw'),
            1: ('rp_bias_yaw (°)', 'RP-compensated yaw'),
            2: ('yaw_model (°)', 'Vehicle model yaw'),
            3: ('roll (°)', 'Roll'),
            4: ('pitch (°)', 'Pitch'),
        }
        data_keys = ['z_bias', 'rp_bias', 'yaw_model', 'roll', 'pitch']

        for i, ax in enumerate(axes):
            lines[i], = ax.plot([], [], 'b-', linewidth=0.8)
            ax.set_ylabel(labels[i][0])
            ax.set_title(labels[i][1], fontsize=10)
            ax.grid(True, alpha=0.3)

        axes[-1].set_xlabel('Time (s)')

        # Add text annotation for analysis
        analysis_text = fig.text(0.02, 0.01, '', fontsize=9, family='monospace',
                                  verticalalignment='bottom')

        count = [0]
        last_analysis = [0]

        def init():
            for line in lines.values():
                line.set_data([], [])
            return list(lines.values()) + [analysis_text]

        def update(frame):
            # Read all available data
            for _ in range(20):  # Read up to 20 lines per frame
                line = self.read_line()
                if line is None:
                    break
                d = self.parse_line(line)
                if d is None:
                    continue
                self.add_data(d)
                count[0] += 1

            if not self.t:
                return list(lines.values()) + [analysis_text]

            t_arr = np.array(self.t)
            for i, key in enumerate(data_keys):
                data_arr = np.array(getattr(self, key))
                lines[i].set_data(t_arr, data_arr)
                axes[i].relim()
                axes[i].autoscale_view()

            # Update analysis text every 500 samples
            if count[0] - last_analysis[0] >= 500 and count[0] >= 500:
                rec = self._compute_recommendation()
                if rec:
                    txt = (
                        f"Current: Q={rec['Q_current']:.4f} R={rec['R_current']:.3f} "
                        f"K_ss={kalman_steady_state_gain(rec['Q_current'], rec['R_current'])[1]:.3f} | "
                        f"Recommend: Q={rec['Q_recommended']:.5f} R={rec['R_recommended']:.3f} "
                        f"K_target={rec['K_target']:.2f}"
                    )
                    analysis_text.set_text(txt)
                last_analysis[0] = count[0]

            return list(lines.values()) + [analysis_text]

        ani = FuncAnimation(fig, update, init_func=init, interval=100, blit=False, cache_frame_data=False)
        plt.tight_layout(rect=[0, 0.05, 1, 0.95])
        plt.show()

    def _compute_analysis(self):
        """Compute statistics from collected data."""
        if len(self.t) < 100:
            return None

        t_arr = np.array(self.t)
        z_arr = np.array(self.z_bias)
        rp_arr = np.array(self.rp_bias)
        model_arr = np.array(self.yaw_model)
        roll_arr = np.array(self.roll)
        pitch_arr = np.array(self.pitch)

        dt = np.median(np.diff(t_arr)) if len(t_arr) > 1 else 0.004

        # Roll/pitch statistics
        roll_mean = np.mean(roll_arr)
        roll_std = np.std(roll_arr)
        pitch_mean = np.mean(pitch_arr)
        pitch_std = np.std(pitch_arr)

        # Yaw drift rate (linear fit)
        if len(t_arr) > 10:
            z_fit = np.polyfit(t_arr, z_arr, 1)
            rp_fit = np.polyfit(t_arr, rp_arr, 1)
            z_drift_deg_per_min = z_fit[0] * 60.0
            rp_drift_deg_per_min = rp_fit[0] * 60.0
        else:
            z_drift_deg_per_min = 0.0
            rp_drift_deg_per_min = 0.0

        # Difference between z_bias and rp_bias
        diff_arr = rp_arr - z_arr
        diff_mean = np.mean(diff_arr)
        diff_std = np.std(diff_arr)

        # Yaw model vs gyro comparison
        # Normalize angle differences to [-180, 180]
        model_vs_z = model_arr - z_arr
        model_vs_z = (model_vs_z + 180) % 360 - 180
        model_vs_z_mean = np.mean(model_vs_z)
        model_vs_z_std = np.std(model_vs_z)

        model_vs_rp = model_arr - rp_arr
        model_vs_rp = (model_vs_rp + 180) % 360 - 180
        model_vs_rp_mean = np.mean(model_vs_rp)
        model_vs_rp_std = np.std(model_vs_rp)

        # Current Kalman parameters
        Q_current = 0.001
        R_current = 0.1
        T_current = 0.004

        # Current steady-state analysis
        current_analysis = analyze_kalman_params(Q_current, R_current, T_current,
                                                  roll_std, z_drift_deg_per_min)

        return {
            'n_samples': len(t_arr),
            'duration_s': t_arr[-1] - t_arr[0],
            'dt_median': dt,
            'roll_mean': roll_mean,
            'roll_std': roll_std,
            'pitch_mean': pitch_mean,
            'pitch_std': pitch_std,
            'z_drift_deg_per_min': z_drift_deg_per_min,
            'rp_drift_deg_per_min': rp_drift_deg_per_min,
            'diff_mean': diff_mean,
            'diff_std': diff_std,
            'model_vs_z_mean': model_vs_z_mean,
            'model_vs_z_std': model_vs_z_std,
            'model_vs_rp_mean': model_vs_rp_mean,
            'model_vs_rp_std': model_vs_rp_std,
            'current_kalman': current_analysis,
        }

    def _compute_recommendation(self):
        """Compute parameter recommendation."""
        a = self._compute_analysis()
        if a is None:
            return None

        rec = recommend_params(
            roll_std_deg=a['roll_std'],
            pitch_std_deg=a['pitch_std'],
            yaw_drift_deg_per_min=abs(a['z_drift_deg_per_min']),
            T=a['dt_median'],
            current_Q=0.001,
            current_R=0.1,
        )
        self.recommendation = rec
        return rec

    def _print_analysis(self):
        """Print intermediate analysis."""
        a = self._compute_analysis()
        if a is None:
            return

        print("\n" + "="*70)
        print(f"  INTERMEDIATE ANALYSIS ({a['n_samples']} samples, {a['duration_s']:.1f}s)")
        print("="*70)
        print(f"  Roll:  mean={a['roll_mean']:+.3f}°  std={a['roll_std']:.4f}°")
        print(f"  Pitch: mean={a['pitch_mean']:+.3f}°  std={a['pitch_std']:.4f}°")
        print(f"  Z-yaw drift:   {a['z_drift_deg_per_min']:+.4f} °/min")
        print(f"  RP-yaw drift:  {a['rp_drift_deg_per_min']:+.4f} °/min")
        print(f"  Z vs RP diff:  mean={a['diff_mean']:+.3f}°  std={a['diff_std']:.4f}°")
        print(f"  Model vs Z:    mean={a['model_vs_z_mean']:+.3f}°  std={a['model_vs_z_std']:.4f}°")
        print(f"  Model vs RP:   mean={a['model_vs_rp_mean']:+.3f}°  std={a['model_vs_rp_std']:.4f}°")

        ck = a['current_kalman']
        print(f"\n  Current Kalman: Q={ck['Q']:.4f}  R={ck['R']:.3f}")
        print(f"    Steady-state gain K_ss={ck['K_ss']:.4f} "
              f"(acc trust {ck['acc_trust_pct']:.1f}%, gyro trust {ck['gyro_trust_pct']:.1f}%)")
        print(f"    Time constant τ={ck['tau_s']:.2f}s")
        print("="*70 + "\n")

    def _print_final_analysis(self):
        """Print final analysis with parameter recommendations."""
        a = self._compute_analysis()
        if a is None:
            print("[WARN] Not enough data for analysis")
            return

        rec = self._compute_recommendation()

        print("\n" + "#"*70)
        print("#  FINAL ANALYSIS - First-Layer Kalman Filter Tuning")
        print("#"*70)

        print(f"\n## Data Summary")
        print(f"  Samples: {a['n_samples']}, Duration: {a['duration_s']:.1f}s")
        print(f"  Median dt: {a['dt_median']*1000:.1f}ms (expected 4ms)")

        print(f"\n## Roll/Pitch (6-axis Kalman output)")
        print(f"  Roll:  mean={a['roll_mean']:+.3f}°  std={a['roll_std']:.4f}°")
        print(f"  Pitch: mean={a['pitch_mean']:+.3f}°  std={a['pitch_std']:.4f}°")

        print(f"\n## Yaw Drift (pure gyro integration)")
        print(f"  Z-axis yaw drift:   {a['z_drift_deg_per_min']:+.4f} °/min "
              f"({a['z_drift_deg_per_min']/60:.6f} °/s)")
        print(f"  RP-comp yaw drift:  {a['rp_drift_deg_per_min']:+.4f} °/min "
              f"({a['rp_drift_deg_per_min']/60:.6f} °/s)")
        print(f"  Z vs RP difference: mean={a['diff_mean']:+.3f}°  std={a['diff_std']:.4f}°")

        print(f"\n## Yaw Model vs Gyro")
        print(f"  Model vs Z-yaw:  mean={a['model_vs_z_mean']:+.3f}°  std={a['model_vs_z_std']:.4f}°")
        print(f"  Model vs RP-yaw: mean={a['model_vs_rp_mean']:+.3f}°  std={a['model_vs_rp_std']:.4f}°")

        ck = a['current_kalman']
        print(f"\n## Current Kalman Parameters")
        print(f"  Q = {ck['Q']:.4f}  (process noise per step)")
        print(f"  R = {ck['R']:.3f}  (measurement noise)")
        print(f"  T = {ck['T']:.4f}s  (sample period)")
        print(f"  Steady-state gain K_ss = {ck['K_ss']:.4f}")
        print(f"    → Acc trust: {ck['acc_trust_pct']:.1f}%")
        print(f"    → Gyro trust: {ck['gyro_trust_pct']:.1f}%")
        print(f"  Time constant τ ≈ {ck['tau_s']:.2f}s")

        if rec:
            print(f"\n## Recommended Parameters")
            print(f"  Q = {rec['Q_recommended']:.5f}  (was {rec['Q_current']:.4f})")
            print(f"  R = {rec['R_recommended']:.3f}  (was {rec['R_current']:.3f})")

            new_P_ss, new_K_ss = kalman_steady_state_gain(rec['Q_recommended'], rec['R_recommended'])
            print(f"  New K_ss = {new_K_ss:.4f}  (was {ck['K_ss']:.4f})")
            print(f"    → Acc trust: {new_K_ss*100:.1f}%  (was {ck['acc_trust_pct']:.1f}%)")
            print(f"    → Gyro trust: {(1-new_K_ss)*100:.1f}%  (was {ck['gyro_trust_pct']:.1f}%)")

            # C code changes
            print(f"\n## C Code Changes (Ins.c Init_config)")
            print(f"  s_config.kalman_6axis_q = {rec['Q_recommended']:.5f}f;   // was 0.001f")
            print(f"  s_config.kalman_6axis_r = {rec['R_recommended']:.3f}f;    // was 0.1f")

        print(f"\n## Interpretation")
        print(f"  - Roll/Pitch: The 6-axis Kalman fuses gyro (prediction) with")
        print(f"    accelerometer (measurement). K_ss determines the balance.")
        print(f"  - If roll/pitch are too noisy → increase R (trust acc less)")
        print(f"  - If roll/pitch drift slowly → decrease R (trust acc more)")
        print(f"  - Yaw: K[2]=0 always, so Q[2]/R[2] don't affect output.")
        print(f"    Yaw is pure gyro integration; drift is corrected by the")
        print(f"    second-layer yaw EKF (mag fusion) or bias calibration.")
        print(f"  - The Z vs RP yaw difference ({a['diff_mean']:+.3f}°) shows the")
        print(f"    effect of roll/pitch compensation on yaw rate.")

        print("#"*70 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Real-time gyroyaw analyzer for first-layer Kalman tuning"
    )
    parser.add_argument("--port", default="COM24", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--plot", action="store_true", help="Enable real-time plot")
    parser.add_argument("--duration", type=float, default=None, help="Duration in seconds")
    args = parser.parse_args()

    analyzer = GyroYawAnalyzer(args.port, args.baud)
    analyzer.connect()

    if args.plot and HAS_MPL:
        analyzer.run_plot_mode(args.duration)
    else:
        analyzer.run_text_mode(args.duration)


if __name__ == "__main__":
    main()
