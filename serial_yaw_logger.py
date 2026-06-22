# -*- coding: utf-8 -*-
import serial
import time
import os
from datetime import datetime

PORT = 'COM24'
BAUD = 115200
SAVE_DIR = 'track_data'

os.makedirs(SAVE_DIR, exist_ok=True)

timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
save_path = os.path.join(SAVE_DIR, 'yaw_log_%s.txt' % timestamp)

ser = serial.Serial(PORT, BAUD, timeout=0.1)
print('[PORT] %s @ %d' % (PORT, BAUD))
print('[SAVE] %s' % save_path)
print('[RECV] Ctrl+C to stop\n')

count = 0
t0 = time.time()

with open(save_path, 'w', buffering=1) as f:
    f.write('# ts,raw_gyro_deg_s,yaw_6axis_deg,yaw_final_deg,pitch_deg,roll_deg,speed_mps\n')
    try:
        while True:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            if line.startswith('Z:'):
                parts = line[2:].split(',')
                if len(parts) >= 6:
                    ts = time.time() - t0
                    f.write('%.3f,%s,%s,%s,%s,%s,%s\n' % (ts, parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]))
                    count += 1
                    if count % 25 == 0:
                        print('[%d] gyro=%8s yaw=%8s pitch=%6s roll=%6s spd=%s' % (count, parts[0], parts[1], parts[3], parts[4], parts[5]))
            else:
                ts = time.time() - t0
                f.write('%.3f,# %s\n' % (ts, line))
    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print('\n[STOP] %d yaw records, %.1fs' % (count, elapsed))
        print('[FILE] %s' % save_path)
    finally:
        ser.close()
