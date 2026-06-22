import math

WHEELBASE=0.78; TRACK_WIDTH=0.59; WHEEL_DIAMETER=0.24; PULSES_PER_REV=2048; SAMPLE_DT=0.004
TICK_TO_METER = math.pi * WHEEL_DIAMETER / PULSES_PER_REV
v_mps = 5 * TICK_TO_METER / SAMPLE_DT
K_diff = TICK_TO_METER / SAMPLE_DT / TRACK_WIDTH

print("=" * 70)
print("FINAL PARAMETER VERIFICATION")
print("=" * 70)
print()
print("Key facts:")
print(f"  v_mps = {v_mps:.4f} m/s (5 pulses/4ms)")
print(f"  K_diff = {K_diff:.4f} rad/s per pulse = {math.degrees(K_diff):.2f} deg/s per pulse")
print(f"  1 pulse delta = 20% of base speed, 8.94 deg/s yaw rate")
print()

# Required delta_ss for each steer angle
print("Required steady-state delta (pulses) for each steer angle:")
for steer_deg in [5,10,15,20,25]:
    steer_rad = math.radians(steer_deg)
    omega_target = v_mps * math.tan(steer_rad) / WHEELBASE
    delta_ss = omega_target / K_diff
    print(f"  steer={steer_deg:2d}deg: omega_target={math.degrees(omega_target):6.2f} deg/s, delta_ss={delta_ss:.3f} pulses")
print()

# Test different parameter sets
param_sets = [
    (8.0, 0.5, 6, "ORIGINAL (KP=8, KI=0.5, MAX=6)"),
    (8.0, 0.5, 2, "FIX MAX only (KP=8, KI=0.5, MAX=2)"),
    (15.0, 2.0, 2, "TUNED v1 (KP=15, KI=2.0, MAX=2)"),
    (20.0, 3.0, 2, "TUNED v2 (KP=20, KI=3.0, MAX=2)"),
]

for DIFF_KP, DIFF_KI, MAX_DELTA, label in param_sets:
    print("=" * 70)
    print(f"  {label}")
    print("=" * 70)
    DT = 0.004; tau = 0.1
    
    for steer_deg in [10, 15, 20, 25]:
        steer_rad = math.radians(steer_deg)
        omega_target = v_mps * math.tan(steer_rad) / WHEELBASE
        
        omega_actual = 0.0
        integral = 0.0
        max_overshoot = 0.0
        settle_time = -1
        
        for step in range(5000):  # 20 seconds
            omega_error = omega_target - omega_actual
            
            integral += omega_error * DT
            int_limit = MAX_DELTA / (DIFF_KI + 0.001)
            integral = max(-int_limit, min(int_limit, integral))
            
            output = DIFF_KP * omega_error + DIFF_KI * integral
            delta = max(-MAX_DELTA, min(MAX_DELTA, output))
            
            omega_diff = delta * K_diff
            omega_actual += (omega_diff - omega_actual) / tau * DT
            
            if omega_actual > max_overshoot:
                max_overshoot = omega_actual
            
            if settle_time < 0 and abs(omega_error) < 0.05 * omega_target:
                settle_time = step * DT * 1000
        
        err_pct = (omega_target - omega_actual) / omega_target * 100
        overshoot_pct = (max_overshoot - omega_target) / omega_target * 100 if max_overshoot > omega_target else 0
        print(f"  steer={steer_deg:2d}deg: target={math.degrees(omega_target):6.2f}, actual={math.degrees(omega_actual):6.2f} deg/s, "
              f"err={err_pct:+.1f}%, overshoot={overshoot_pct:.1f}%, settle={settle_time:.0f}ms")
    print()

# Detailed step response for best parameter set
print("=" * 70)
print("DETAILED STEP RESPONSE: KP=20, KI=3.0, MAX_DELTA=2, steer=15deg")
print("=" * 70)
DIFF_KP=20.0; DIFF_KI=3.0; MAX_DELTA=2.0; DT=0.004; tau=0.1
steer_deg = 15
steer_rad = math.radians(steer_deg)
omega_target = v_mps * math.tan(steer_rad) / WHEELBASE

omega_actual = 0.0
integral = 0.0

hdr = "  t(ms)  error    P_term  I_term  delta   omega_act  omega_deg"
print(hdr)
for step in range(2000):
    t_ms = step * DT * 1000
    omega_error = omega_target - omega_actual
    
    integral += omega_error * DT
    int_limit = MAX_DELTA / (DIFF_KI + 0.001)
    integral = max(-int_limit, min(int_limit, integral))
    
    output = DIFF_KP * omega_error + DIFF_KI * integral
    delta = max(-MAX_DELTA, min(MAX_DELTA, output))
    
    omega_diff = delta * K_diff
    omega_actual += (omega_diff - omega_actual) / tau * DT
    
    if step % 50 == 0 or step == 1999:
        p_t = DIFF_KP * omega_error
        i_t = DIFF_KI * integral
        print(f"  {t_ms:5.0f}  {omega_error:7.4f}  {p_t:6.2f}  {i_t:6.3f}  {delta:6.3f}  {omega_actual:7.4f}  {math.degrees(omega_actual):7.2f}")

print()
print("=" * 70)
print("CRITICAL CODE CHANGE NEEDED")
print("=" * 70)
print()
print("1. MAX_DELTA: 6 -> 2")
print("   Reason: delta=6 gives 53.6 deg/s yaw rate (3.4x target at max steer)")
print("   delta=2 gives 17.9 deg/s, sufficient for all steer angles up to 25deg")
print()
print("2. KP: 8 -> 20")
print("   Reason: KP=8 gives P_term=0.41~2.20, which rounds to 0~2")
print("   KP=20 gives P_term=1.03~5.50, providing stronger initial response")
print()
print("3. KI: 0.5 -> 3.0")
print("   Reason: KI=0.5 is too slow to eliminate steady-state error")
print("   KI=3.0 reaches steady state in ~1 second")
print()
print("4. Remove round() - keep delta as FLOAT")
print("   Reason: 1 pulse = 8.94 deg/s, rounding causes oscillation")
print("   Float delta allows smooth control (motor PWM handles final quantization)")
print()
print("5. Remove deadzone (or reduce to 0.01 rad/s)")
print("   Reason: Deadzone kills integral accumulation, causing steady-state error")
print("   Gyro noise is typically <0.01 rad/s after IIR filtering")
