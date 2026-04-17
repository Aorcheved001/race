import re
import sys

logfile = r'd:\race\save\4.10\new_ins\baseline_01.txt'
if len(sys.argv) > 1:
    logfile = sys.argv[1]

data = []
with open(logfile, 'r') as f:
    for line in f:
        m = re.search(r'imu_yaw:([-\d.]+),([-\d.]+),([-\d.]+)', line)
        if m:
            data.append(float(m.group(3)))

n = len(data)
print(f"Total samples: {n}")
if n < 10:
    print("Not enough data!")
    sys.exit(1)

cleaned = [data[0]]
for i in range(1, n):
    if abs(data[i] - cleaned[-1]) > 500:
        continue
    cleaned.append(data[i])

print(f"After cleaning: {len(cleaned)}")

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

print("")
print("=== DRIFT ANALYSIS ===")
print(f"Duration: {duration_min:.2f} min ({len(cleaned)} samples)")
print(f"First 4s avg yaw_ekf: {avg_first:.2f} deg")
print(f"Last 4s avg yaw_ekf:  {avg_last:.2f} deg")
print(f"Total drift (avg):     {total_drift:+.2f} deg")
print(f"Drift rate (avg):      {drift_per_min:+.3f} deg/min")
print(f"Max:  {data_max:.2f}  Min: {data_min:.2f}  Range: {range_drift:.2f} deg")
print(f"Range rate: {range_drift/ duration_min :+.3f} deg/min")
print("")
target = 0.3
status = "PASS" if abs(drift_per_min) <= target else "FAIL - need tuning"
print(f"Target: <= {target:.3f} deg/min")
print(f"Status: {status}")
