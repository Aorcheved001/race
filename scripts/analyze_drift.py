import re
import sys

logfile = r'd:\race\save\4.10\new_ins\baseline_01.txt'
if len(sys.argv) > 1:
    logfile = sys.argv[1]

roll_data = []
pitch_data = []
yaw_data = []
with open(logfile, 'r') as f:
    for line in f:
        att_match = re.search(r'imu_att:([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)', line)
        if att_match:
            roll_data.append(float(att_match.group(1)))
            pitch_data.append(float(att_match.group(2)))
            yaw_data.append(float(att_match.group(5)))
            continue

        yaw_match = re.search(r'imu_yaw:([-\d.]+),([-\d.]+),([-\d.]+)', line)
        if yaw_match:
            yaw_data.append(float(yaw_match.group(3)))

n = len(yaw_data) if len(yaw_data) > 0 else len(roll_data)
print(f"Total samples: {n}")
if n < 10:
    print("Not enough data!")
    sys.exit(1)

def clean_series(series, jump_limit):
    if len(series) == 0:
        return []

    cleaned = [series[0]]
    for i in range(1, len(series)):
        if abs(series[i] - cleaned[-1]) > jump_limit:
            continue
        cleaned.append(series[i])
    return cleaned

def analyze_series(name, series, target):
    cleaned = clean_series(series, 500 if name == "yaw_ekf" else 30)
    if len(cleaned) < 10:
        print(f"{name}: not enough data after cleaning")
        return

    first_100 = cleaned[:100]
    last_100 = cleaned[-100:]
    avg_first = sum(first_100) / len(first_100)
    avg_last = sum(last_100) / len(last_100)
    total_drift = avg_last - avg_first
    duration_min = len(cleaned) / 25.0 / 60.0
    drift_per_min = total_drift / duration_min if duration_min > 0 else 0
    data_max = max(cleaned)
    data_min = min(cleaned)
    range_drift = data_max - data_min
    status = "PASS" if abs(drift_per_min) <= target else "FAIL - need tuning"

    print("")
    print(f"=== {name.upper()} DRIFT ANALYSIS ===")
    print(f"Duration: {duration_min:.2f} min ({len(cleaned)} samples)")
    print(f"First 4s avg: {avg_first:.3f} deg")
    print(f"Last 4s avg:  {avg_last:.3f} deg")
    print(f"Total drift:  {total_drift:+.3f} deg")
    print(f"Drift rate:   {drift_per_min:+.4f} deg/min")
    print(f"Max: {data_max:.3f}  Min: {data_min:.3f}  Range: {range_drift:.3f} deg")
    print(f"Target: <= {target:.4f} deg/min")
    print(f"Status: {status}")

if len(roll_data) > 0 and len(pitch_data) > 0:
    analyze_series("roll", roll_data, 0.05)
    analyze_series("pitch", pitch_data, 0.05)

if len(yaw_data) > 0:
    analyze_series("yaw_ekf", yaw_data, 0.3)
