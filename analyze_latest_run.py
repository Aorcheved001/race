# -*- coding: utf-8 -*-
"""分析最新follow数据"""
import math

# Read follow data
with open('track_data/subj3_follow_20260614_190119.txt', 'r') as f:
    lines = f.readlines()

# Parse header
header = {}
for line in lines:
    if line.startswith('#'):
        if ':' in line and not line.startswith('# Format') and not line.startswith('#---'):
            key, val = line[2:].split(':', 1)
            header[key.strip()] = val.strip()
    else:
        break

print('=== HEADER ===')
for k, v in header.items():
    print(f'  {k}: {v}')

# Parse data points
data = []
for line in lines:
    if line.startswith('#') or not line.strip():
        continue
    parts = line.strip().split(',')
    if len(parts) == 5:
        data.append({
            'idx': int(parts[0]),
            'x': float(parts[1]),
            'y': float(parts[2]),
            'yaw': float(parts[3]),
            'target_idx': int(parts[4])
        })

print(f'\nTotal data lines: {len(data)}')

# Separate car and target positions
car_positions = [d for d in data if d['idx'] == 0]
target_positions = [d for d in data if d['idx'] != 0]

print(f'Car position samples: {len(car_positions)}')
print(f'Target position samples: {len(target_positions)}')

# Target index progression
print('\n=== TARGET INDEX PROGRESSION ===')
prev_target = None
changes = []
for d in car_positions:
    t = d['target_idx']
    if t != prev_target:
        changes.append((t, d['x'], d['y']))
        prev_target = t

for t, x, y in changes:
    print(f'  target_idx -> {t} at car ({x:.2f}, {y:.2f})')

max_target = max(d['target_idx'] for d in car_positions)
print(f'\nMax target_idx reached: {max_target} / 288')
print(f'Progress: {max_target/288*100:.0f}%')

# Read drive path
drive_data = []
with open('track_data/subj3_drive_20260614_185850.txt', 'r') as f:
    for line in f:
        if line.startswith('#') or not line.strip():
            continue
        parts = line.strip().split(',')
        if len(parts) == 5:
            drive_data.append({
                'idx': int(parts[0]),
                'x': float(parts[1]),
                'y': float(parts[2]),
                'yaw': float(parts[3])
            })

print(f'\nDrive path points: {len(drive_data)}')

# Calculate lateral deviations
max_dev = 0
max_dev_pos = None
deviations = []
for d in car_positions:
    min_dist = float('inf')
    closest_idx = 0
    for p in drive_data:
        dist = math.sqrt((d['x'] - p['x'])**2 + (d['y'] - p['y'])**2)
        if dist < min_dist:
            min_dist = dist
            closest_idx = p['idx']
    deviations.append(min_dist)
    if min_dist > max_dev:
        max_dev = min_dist
        max_dev_pos = (d['x'], d['y'], closest_idx)

print(f'\n=== LATERAL DEVIATION ===')
print(f'Max lateral deviation: {max_dev:.3f}m at car ({max_dev_pos[0]:.2f}, {max_dev_pos[1]:.2f}), path idx={max_dev_pos[2]}')
print(f'Avg lateral deviation: {sum(deviations)/len(deviations):.3f}m')

# Deviation distribution
bins = [0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 2.0]
for i in range(len(bins)-1):
    count = sum(1 for d in deviations if bins[i] <= d < bins[i+1])
    pct = count / len(deviations) * 100
    print(f'  {bins[i]:.1f}-{bins[i+1]:.1f}m: {count} ({pct:.1f}%)')

# Find where deviation > 0.5m
big_dev = [(i, deviations[i], car_positions[i]) for i in range(len(deviations)) if deviations[i] > 0.5]
print(f'\nSamples with deviation > 0.5m: {len(big_dev)} / {len(deviations)} ({len(big_dev)/len(deviations)*100:.1f}%)')

# Analyze yaw difference
print('\n=== YAW ANALYSIS ===')
yaw_diffs = []
for d in car_positions:
    tidx = d['target_idx']
    if tidx > 0 and tidx <= len(drive_data):
        path_yaw = drive_data[min(tidx-1, len(drive_data)-1)]['yaw']
        car_yaw = d['yaw']
        diff = car_yaw - path_yaw
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        yaw_diffs.append(diff)

if yaw_diffs:
    abs_yaws = [abs(y) for y in yaw_diffs]
    print(f'Max yaw error: {max(abs_yaws)*180/math.pi:.1f} deg')
    print(f'Avg yaw error: {sum(abs_yaws)/len(abs_yaws)*180/math.pi:.1f} deg')
    
    # Yaw error distribution
    yaw_bins = [0, 5, 10, 15, 20, 30, 45, 90]
    for i in range(len(yaw_bins)-1):
        count = sum(1 for y in abs_yaws if yaw_bins[i] <= y*180/math.pi < yaw_bins[i+1])
        pct = count / len(abs_yaws) * 100
        print(f'  {yaw_bins[i]}-{yaw_bins[i+1]} deg: {count} ({pct:.1f}%)')

# Analyze car speed (distance between consecutive car positions)
print('\n=== SPEED ANALYSIS ===')
speeds = []
for i in range(1, len(car_positions)):
    dx = car_positions[i]['x'] - car_positions[i-1]['x']
    dy = car_positions[i]['y'] - car_positions[i-1]['y']
    dist = math.sqrt(dx*dx + dy*dy)
    speeds.append(dist)

if speeds:
    avg_speed = sum(speeds) / len(speeds)
    # Each sample = 4ms, so speed = dist / 0.004
    avg_speed_ms = avg_speed / 0.004
    print(f'Avg step distance: {avg_speed*1000:.1f}mm / 4ms')
    print(f'Avg speed: {avg_speed_ms:.2f} m/s')
    print(f'Max step distance: {max(speeds)*1000:.1f}mm')
    print(f'Min step distance: {min(speeds)*1000:.1f}mm')

# Analyze stuck detection - find periods where target_idx doesn't change
print('\n=== STUCK/PROGRESS ANALYSIS ===')
stuck_count = 0
stuck_periods = []
current_stuck = 0
prev_tidx = car_positions[0]['target_idx']
for d in car_positions[1:]:
    if d['target_idx'] == prev_tidx:
        current_stuck += 1
    else:
        if current_stuck > 50:  # > 200ms stuck
            stuck_periods.append(current_stuck)
        current_stuck = 0
        prev_tidx = d['target_idx']

if current_stuck > 50:
    stuck_periods.append(current_stuck)

print(f'Stuck periods (>200ms without progress): {len(stuck_periods)}')
for i, s in enumerate(stuck_periods):
    print(f'  Period {i+1}: {s*4}ms ({s} frames)')

# Path curvature analysis
print('\n=== PATH CURVATURE (drive) ===')
curvatures = []
for i in range(1, len(drive_data)-1):
    p0 = drive_data[i-1]
    p1 = drive_data[i]
    p2 = drive_data[i+1]
    
    # Vectors
    v1x = p1['x'] - p0['x']
    v1y = p1['y'] - p0['y']
    v2x = p2['x'] - p1['x']
    v2y = p2['y'] - p1['y']
    
    # Angle change
    a1 = math.atan2(v1y, v1x)
    a2 = math.atan2(v2y, v2x)
    da = a2 - a1
    while da > math.pi: da -= 2*math.pi
    while da < -math.pi: da += 2*math.pi
    
    # Arc length
    ds = math.sqrt(v1x**2 + v1y**2)
    if ds > 0.001:
        kappa = abs(da) / ds  # curvature = 1/R
        R = 1.0 / kappa if kappa > 0.001 else 999
        curvatures.append((p1['idx'], R, da*180/math.pi))

# Find tight curves
tight_curves = [(idx, R, da) for idx, R, da in curvatures if R < 3.0]
print(f'Points with R < 3m: {len(tight_curves)}')
for idx, R, da in tight_curves[:20]:
    print(f'  idx={idx}: R={R:.2f}m, angle_change={da:.1f}deg')
