# -*- coding: utf-8 -*-
import re
import sys

logfile = r'd:\race\save\4.10\new_ins\dynamic_p2_rotation.txt'
if len(sys.argv) > 1:
    logfile = sys.argv[1]

data_gyro = []
data_mag = []
data_ekf = []

with open(logfile, 'r') as f:
    for line in f:
        m = re.search(r'imu_yaw:([-\d.]+),([-\d.]+),([-\d.]+)', line)
        if m:
            data_gyro.append(float(m.group(1)))
            data_mag.append(float(m.group(2)))
            data_ekf.append(float(m.group(3)))

n = len(data_ekf)
print(f"Total samples: {n}")
print(f"Duration: {n / 25.0:.2f}s ({n} samples @ 25Hz)")
print()

# Find rotation point: detect where yaw_ekf changes rapidly
# Use sliding window to find max rate of change
max_rate = 0
max_rate_idx = 0
window = 10
for i in range(window, n - window):
    rate = abs(data_ekf[i + window] - data_ekf[i - window]) / (2 * window / 25.0)
    if rate > max_rate:
        max_rate = rate
        max_rate_idx = i

# Define phases
# Phase A (static before): first 20% or until rapid change starts
pre_samples = min(n // 5, max(0, max_rate_idx - 100))
# Phase B (rotation + settling): from pre_samples to end
post_start = pre_samples

pre_avg = sum(data_ekf[:pre_samples]) / max(pre_samples, 1)
post_avg_last_10s = sum(data_ekf[-250:]) / min(250, n)  # last 10 seconds

rotation_deg = post_avg_last_10s - pre_avg

print("=" * 55)
print("  DYNAMIC CONVERGENCE ANALYSIS")
print("=" * 55)

print(f"\n--- Phase A: Static (before rotation) ---")
print(f"  Samples     : {pre_samples}")
print(f"  Duration    : {pre_samples / 25.0:.1f}s")
print(f"  Avg yaw_ekf : {pre_avg:.2f} deg")

print(f"\n--- Rotation Event ---")
print(f"  Detected at sample #{max_rate_idx} ({max_rate_idx/25.0:.1f}s)")
print(f"  Max rate of change : {max_rate:.1f} deg/s")

print(f"\n--- Phase B: Post-rotation settling ---")
print(f"  Last 10s avg yaw_ekf : {post_avg_last_10s:.2f} deg")
print(f"  Total angle change   : {rotation_deg:+.2f} deg")

# Analyze convergence: after rotation, when does it settle?
# Find the point where ekf enters ??1deg of final value and stays there
final_val = post_avg_last_10s
settled = False
settle_time = None
consecutive_stable = 0
required_stable = 50  # 2 seconds of consecutive stability

for i in range(max(pre_samples + 50, max_rate_idx + 20), n):
    if abs(data_ekf[i] - final_val) < 1.0:
        consecutive_stable += 1
        if not settled and consecutive_stable >= required_stable:
            settle_time = i - required_stable
            settle_time_s = settle_time / 25.0
            settled = True
    else:
        consecutive_stable = 0

if settle_time is not None:
    rotation_end_approx = max_rate_idx / 25.0
    convergence_time = settle_time_s - rotation_end_approx
    print(f"\n--- Convergence Metrics ---")
    print(f"  Settled at sample #{settle_time} ({settle_time_s:.1f}s)")
    print(f"  Convergence time (from peak motion): {convergence_time:.1f}s")
else:
    print(f"\n--- Convergence ---")
    print(f"  Did NOT fully settle within capture window")

# Overshoot analysis
# Find min/max after rotation starts
post_data = data_ekf[max(pre_samples, max_rate_idx - 30):]
if len(post_data) > 10:
    post_min = min(post_data)
    post_max = max(post_data)
    final_range = post_max - post_min
    # overshoot relative to final value direction
    if rotation_deg > 0:
        overshoot = max(post_max - final_val, 0)
    else:
        overshoot = max(final_val - post_min, 0)

    print(f"\n--- Post-rotation Range ---")
    print(f"  Min yaw_ekf  : {post_min:.2f} deg")
    print(f"  Max yaw_ekf  : {post_max:.2f} deg")
    print(f"  Range        : {final_range:.2f} deg")
    print(f"  Overshoot    : {overshoot:.2f} deg")

# Drift during post-rotation static period (last 30s)
if n > 750:
    drift_start = sum(data_ekf[-750:-250]) / 500
    drift_end = sum(data_ekf[-250:]) / 250
    drift_rate = (drift_end - drift_start) / 0.333  # 20 seconds in minutes
    print(f"\n--- Post-settling Drift (last 20s of static) ---")
    print(f"  First 20s avg : {drift_start:.3f} deg")
    print(f"  Last 20s avg  : {drift_end:.3f} deg")
    print(f"  Drift rate    : {drift_rate:+.3f} deg/min")
    target_ok = "PASS" if abs(drift_rate) <= 0.3 else "FAIL"
    print(f"  Target <=0.3  : {target_ok}")

# Print time-series snapshot at key points
print(f"\n--- Time Series Snapshot (every 5s) ---")
print(f"  {'Time(s)':>8} | {'gyro':>8} | {'mag':>8} | {'ekf':>8}")
for i in range(0, n, 125):
    t = i / 25.0
    marker = ""
    if abs(i - max_rate_idx) < 15:
        marker = " <-- ROTATION"
    elif settle_time and abs(i - settle_time) < 15:
        marker = " <-- SETTLED"
    print(f"  {t:8.1f} | {data_gyro[i]:8.2f} | {data_mag[i]:8.2f} | {data_ekf[i]:8.2f}{marker}")

print("\n" + "=" * 55)
