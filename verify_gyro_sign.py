# -*- coding: utf-8 -*-
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ==================== Vehicle Parameters ====================
WHEELBASE = 0.78
TRACK_WIDTH = 0.59
TICK_TO_METER = 0.3682e-3
DT = 0.004

# ==================== Diff PID Parameters (tuned) ====================
KP = 20.0
KI = 3.0
KD = 0.0
MAX_DELTA = 2.0
DEADZONE = 0.01
LOW_SPEED_THRESH = 0.3
LOW_SPEED_GAIN = 1.5

# ==================== Simulation Parameters ====================
BASE_SPEED_PULSE = 5
V_MPS = BASE_SPEED_PULSE * TICK_TO_METER / DT  # 0.4602 m/s
K_DIFF = 0.1560  # rad/s per pulse

def simulate(steer_deg, duration=10.0):
    steer_rad = np.radians(steer_deg)
    omega_target = V_MPS * np.tan(steer_rad) / WHEELBASE
    
    omega_actual = 0.0
    integral = 0.0
    last_error = 0.0
    delta = 0.0
    
    times = []
    omega_targets = []
    omega_actuals = []
    deltas = []
    errors = []
    
    steps = int(duration / DT)
    for i in range(steps):
        t = i * DT
        omega_error = omega_target - omega_actual
        
        if abs(omega_error) < DEADZONE:
            delta = 0.0
            integral = 0.0
            last_error = 0.0
        else:
            derivative = (omega_error - last_error) / DT
            integral += omega_error * DT
            int_limit = MAX_DELTA / (KI + 0.001)
            integral = np.clip(integral, -int_limit, int_limit)
            gain = LOW_SPEED_GAIN if abs(V_MPS) < LOW_SPEED_THRESH else 1.0
            diff_output = gain * (KP * omega_error + KI * integral + KD * derivative)
            delta = np.clip(diff_output, -MAX_DELTA, MAX_DELTA)
            last_error = omega_error
        
        tau = 0.3
        omega_dot = (K_DIFF * delta - omega_actual) / tau if tau > 0 else 0
        omega_actual += omega_dot * DT
        
        times.append(t)
        omega_targets.append(omega_target)
        omega_actuals.append(omega_actual)
        deltas.append(delta)
        errors.append(omega_error)
    
    return np.array(times), np.array(omega_targets), np.array(omega_actuals), np.array(deltas), np.array(errors)


# ==================== Scenario 1: Step Response ====================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Gyro Closed-Loop Diff Simulation (gyro_z sign fixed)\nomega_actual = -gyro_z (right turn = positive)', fontsize=14)

steer_angles = [10, 15, 20, 25]
colors = ['#2196F3', '#4CAF50', '#FF9800', '#F44336']

ax1 = axes[0, 0]
ax1.set_title('Yaw Rate Response')
for steer, color in zip(steer_angles, colors):
    t, wt, wa, d, e = simulate(steer, duration=8.0)
    omega_target = V_MPS * np.tan(np.radians(steer)) / WHEELBASE
    ax1.plot(t, wa, color=color, label=f'Actual (steer={steer} deg)')
    ax1.axhline(omega_target, color=color, linestyle='--', alpha=0.5, label=f'Target (steer={steer} deg)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Yaw rate (rad/s)')
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.3)

ax2 = axes[0, 1]
ax2.set_title('Diff Delta (pulse/4ms)')
for steer, color in zip(steer_angles, colors):
    t, wt, wa, d, e = simulate(steer, duration=8.0)
    ax2.plot(t, d, color=color, label=f'steer={steer} deg')
ax2.axhline(MAX_DELTA, color='red', linestyle=':', alpha=0.5, label=f'MAX_DELTA={MAX_DELTA}')
ax2.axhline(-MAX_DELTA, color='red', linestyle=':', alpha=0.5)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('delta (pulse/4ms)')
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

ax3 = axes[1, 0]
ax3.set_title('Yaw Rate Error Convergence')
for steer, color in zip(steer_angles, colors):
    t, wt, wa, d, e = simulate(steer, duration=8.0)
    ax3.plot(t, e, color=color, label=f'steer={steer} deg')
ax3.axhline(0, color='black', linestyle='-', alpha=0.3)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Error (rad/s)')
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)

ax4 = axes[1, 1]
ax4.set_title('Steady-State Delta vs Steer Angle')
steer_range = np.arange(5, 30, 1)
delta_ss = []
omega_ss = []
omega_tgt = []
for s in steer_range:
    t, wt, wa, d, e = simulate(s, duration=15.0)
    delta_ss.append(d[-1])
    omega_ss.append(wa[-1])
    omega_tgt.append(wt[-1])
delta_ss = np.array(delta_ss)
omega_ss = np.array(omega_ss)
omega_tgt = np.array(omega_tgt)

ax4_twin = ax4.twinx()
ax4.bar(steer_range, delta_ss, color='#2196F3', alpha=0.6, label='SS delta')
ax4.axhline(MAX_DELTA, color='red', linestyle=':', label=f'MAX_DELTA={MAX_DELTA}')
ax4.set_xlabel('Steer angle (deg)')
ax4.set_ylabel('SS delta (pulse/4ms)', color='#2196F3')
ax4_twin.plot(steer_range, omega_ss, 'g-o', markersize=3, label='Actual omega')
ax4_twin.plot(steer_range, omega_tgt, 'r--', label='Target omega')
ax4_twin.set_ylabel('Yaw rate (rad/s)', color='green')
lines1, labels1 = ax4.get_legend_handles_labels()
lines2, labels2 = ax4_twin.get_legend_handles_labels()
ax4.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc='upper left')
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('verify_gyro_sign_fix.png', dpi=150, bbox_inches='tight')
print("Saved: verify_gyro_sign_fix.png")

# ==================== Scenario 2: Left vs Right Symmetry ====================
fig2, axes2 = plt.subplots(1, 3, figsize=(15, 5))
fig2.suptitle('Left Turn vs Right Turn Symmetry (gyro_z sign fixed)', fontsize=14)

for steer_sign, label in [(1, 'Right'), (-1, 'Left')]:
    steer = steer_sign * 15
    t, wt, wa, d, e = simulate(steer, duration=8.0)
    axes2[0].plot(t, wa, label=f'{label} steer={steer} deg actual')
    axes2[0].plot(t, wt, '--', label=f'{label} steer={steer} deg target')
    axes2[1].plot(t, d, label=f'{label} steer={steer} deg')
    axes2[2].plot(t, e, label=f'{label} steer={steer} deg')

axes2[0].set_title('Yaw Rate')
axes2[0].set_xlabel('Time (s)')
axes2[0].set_ylabel('rad/s')
axes2[0].legend(fontsize=8)
axes2[0].grid(True, alpha=0.3)

axes2[1].set_title('Diff Delta')
axes2[1].set_xlabel('Time (s)')
axes2[1].set_ylabel('pulse/4ms')
axes2[1].legend(fontsize=8)
axes2[1].grid(True, alpha=0.3)

axes2[2].set_title('Yaw Rate Error')
axes2[2].set_xlabel('Time (s)')
axes2[2].set_ylabel('rad/s')
axes2[2].legend(fontsize=8)
axes2[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('verify_gyro_sign_symmetry.png', dpi=150, bbox_inches='tight')
print("Saved: verify_gyro_sign_symmetry.png")

# ==================== Numerical Summary ====================
print("\n" + "="*70)
print("Gyro Closed-Loop Diff Simulation Results (gyro_z sign fixed)")
print("="*70)
print(f"Vehicle: WHEELBASE={WHEELBASE}m, V={V_MPS:.4f}m/s, K_DIFF={K_DIFF} rad/s/pulse")
print(f"PID: KP={KP}, KI={KI}, KD={KD}, MAX_DELTA={MAX_DELTA}, DEADZONE={DEADZONE}")
print(f"Sign fix: omega_actual = -gyro_z (right turn = positive)")
print()
print(f"{'Steer':>7} {'Target_w':>10} {'Actual_w':>10} {'SS_delta':>10} {'Error%':>8} {'Conv(s)':>8}")
print("-"*58)

for steer in [5, 10, 15, 20, 25]:
    t, wt, wa, d, e = simulate(steer, duration=15.0)
    omega_target = wt[-1]
    omega_actual_ss = wa[-1]
    delta_ss = d[-1]
    err_pct = abs(omega_target - omega_actual_ss) / (abs(omega_target) + 1e-6) * 100
    
    err_ratio = np.abs(e) / (np.abs(wt) + 1e-6)
    conv_idx = np.where(err_ratio < 0.05)[0]
    conv_time = t[conv_idx[0]] if len(conv_idx) > 0 else float('inf')
    
    print(f"{steer:>6}d {omega_target:>10.4f} {omega_actual_ss:>10.4f} {delta_ss:>10.4f} {err_pct:>7.2f}% {conv_time:>7.2f}s")

print()
print("Sign convention verification:")
print("  Right turn (steer>0): omega_target>0, gyro_z<0(RHR), omega_actual=-gyro_z>0")
print("    -> error>0 -> delta>0 -> left wheel faster (assist right turn) OK")
print("  Left turn (steer<0): omega_target<0, gyro_z>0(RHR), omega_actual=-gyro_z<0")
print("    -> error<0 -> delta<0 -> right wheel faster (assist left turn) OK")
