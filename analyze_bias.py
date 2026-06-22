# -*- coding: utf-8 -*-
"""
Bias comparison: static vs dynamic
Usage: py analyze_bias.py track_data/yaw_log_*.txt
"""
import numpy as np
import sys, os, glob

def analyze_bias(filepath):
    ts, raw_gyro, yaw_6axis, yaw_final = [], [], [], []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split(',')
            if len(parts) >= 4:
                try:
                    ts.append(float(parts[0]))
                    raw_gyro.append(float(parts[1]))
                    yaw_6axis.append(float(parts[2]))
                    yaw_final.append(float(parts[3]))
                except ValueError:
                    continue
    
    ts = np.array(ts)
    raw_gyro = np.array(raw_gyro)
    yaw_6axis = np.array(yaw_6axis)
    yaw_final = np.array(yaw_final)
    
    if len(ts) < 100:
        print("ERROR: too few samples (%d), need at least 10s" % len(ts))
        return
    
    dt = np.mean(np.diff(ts))
    print("File: %s" % filepath)
    print("Samples: %d, dt=%.1fms, duration=%.1fs" % (len(ts), dt*1000, ts[-1]-ts[0]))
    
    # auto-segment by gyro variance
    window = min(50, len(ts)//4)
    gyro_var = np.array([np.var(raw_gyro[max(0,i-window):i+window]) 
                          for i in range(len(raw_gyro))])
    
    var_threshold = np.median(gyro_var) * 3
    is_static = gyro_var < var_threshold
    
    changes = np.diff(is_static.astype(int))
    starts = np.where(changes != 0)[0] + 1
    starts = np.concatenate([[0], starts, [len(ts)]])
    
    print("\n%60s" % "=" * 60)
    print("%3s %6s %6s %12s %12s" % ("Seg", "Type", "Dur", "gyro_mean", "gyro_std"))
    print("%60s" % "=" * 60)
    
    static_biases = []
    dynamic_biases = []
    
    for i in range(len(starts)-1):
        s, e = starts[i], starts[i+1]
        if e - s < 20:
            continue
        seg_type = "STATIC" if is_static[s] else "MOVING"
        dur = ts[e-1] - ts[s]
        mean_g = np.mean(raw_gyro[s:e])
        std_g = np.std(raw_gyro[s:e])
        
        print("%3d %6s %5.1fs %12.4f %12.4f" % (i, seg_type, dur, mean_g, std_g))
        
        if is_static[s] and dur > 1.0:
            static_biases.append(mean_g)
        elif not is_static[s] and dur > 1.0:
            dynamic_biases.append(mean_g)
    
    print("\n%60s" % "=" * 60)
    print("Summary")
    print("%60s" % "=" * 60)
    
    if static_biases:
        sb = np.array(static_biases)
        print("STATIC bias:  mean=%.4f deg/s, std=%.4f, segments=%d" % (np.mean(sb), np.std(sb), len(sb)))
    else:
        print("STATIC bias:  not detected (keep car still >2s)")
    
    if dynamic_biases:
        db = np.array(dynamic_biases)
        print("MOVING bias:  mean=%.4f deg/s, std=%.4f, segments=%d" % (np.mean(db), np.std(db), len(db)))
    else:
        print("MOVING bias:  not detected (push throttle)")
    
    if static_biases and dynamic_biases:
        diff = np.mean(db) - np.mean(sb)
        print("\nVibe bias offset = %+.4f deg/s" % diff)
        if abs(diff) < 0.1:
            print("[OK] No significant difference (<0.1 deg/s), no compensation needed")
        elif abs(diff) < 0.3:
            print("[WARN] Bias differs, 30s drift=%.1f deg, recommend compensation" % (abs(diff)*30))
        else:
            print("[FAIL] Large bias difference, 30s drift=%.1f deg, MUST compensate" % (abs(diff)*30))
    
    print("\nGlobal raw_gyro: mean=%.4f deg/s, median=%.4f deg/s" % (np.mean(raw_gyro), np.median(raw_gyro)))

if __name__ == '__main__':
    if len(sys.argv) > 1:
        analyze_bias(sys.argv[1])
    else:
        files = sorted(glob.glob('track_data/yaw_log_*.txt'))
        if files:
            analyze_bias(files[-1])
        else:
            print("Usage: py analyze_bias.py track_data/yaw_log_*.txt")
