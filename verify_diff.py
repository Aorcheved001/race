import math

WHEELBASE=0.78; TRACK_WIDTH=0.59; WHEEL_DIAMETER=0.24; PULSES_PER_REV=2048; SAMPLE_DT=0.004
TICK_TO_METER = math.pi * WHEEL_DIAMETER / PULSES_PER_REV
v_mps = 5 * TICK_TO_METER / SAMPLE_DT  # 0.46 m/s
DIFF_KP=8.0; DIFF_KI=0.5; MAX_DELTA=6; DT=0.004

print("=" * 70)
print("PROBLEM 1: MAX_DELTA=6 is WAY too large")
print("=" * 70)
print(f"  delta=6 pulses -> outer wheel = 5+6 = 11 pulses/4ms")
print(f"  That is 11/5 = 2.2x the base speed!")
print(f"  omega_diff = 0.936 rad/s = 53.6 deg/s")
print(f"  This is 3.4x the omega_target at delta=25deg (15.76 deg/s)")
print(f"  Result: MASSIVE overshoot, car will spin!")
print()

print("=" * 70)
print("PROBLEM 2: PID output is tiny compared to MAX_DELTA")
print("=" * 70)
for steer_deg in [5,10,15,20,25]:
    steer_rad = math.radians(steer_deg)
    omega_target = v_mps * math.tan(steer_rad) / WHEELBASE
    omega_error = omega_target  # actual=0
    p_term = DIFF_KP * omega_error
    print(f"  delta={steer_deg:2d}deg: P_term={p_term:.2f} pulses (of max {MAX_DELTA})")
print(f"  Even at delta=25deg, P_term=2.20, only 37% of MAX_DELTA")
print(f"  The PID will NEVER reach MAX_DELTA in normal operation")
print()

print("=" * 70)
print("PROBLEM 3: Closed-loop steady state analysis")
print("=" * 70)
print("  At steady state: omega_actual = omega_target")
print("  Then omega_error = 0, P_term = 0")
print("  But KI * integral provides steady-state output!")
print()

print("=" * 70)
print("Simulating closed-loop step response (delta=15deg)")
print("=" * 70)
steer_deg = 15
steer_rad = math.radians(steer_deg)
omega_target = v_mps * math.tan(steer_rad) / WHEELBASE
print(f"  Target: delta={steer_deg}deg, omega_target={omega_target:.4f} rad/s = {math.degrees(omega_target):.2f} deg/s")

# Simple simulation
tau = 0.1  # vehicle yaw time constant
omega_actual = 0.0
integral = 0.0
last_error = omega_target

header = f"  {'t(ms)':>6} {'omega_err':>10} {'P_term':>8} {'I_term':>8} {'delta':>6} {'omega_act':>10} {'omega_deg':>10}"
print(header)

for step in range(500):  # 500 steps = 2 seconds
    t_ms = step * DT * 1000
    omega_error = omega_target - omega_actual
    
    # Deadzone
    if abs(omega_error) < 0.05:
        delta = 0
        integral = 0
    else:
        integral += omega_error * DT
        int_limit = MAX_DELTA / (DIFF_KI + 0.001)
        integral = max(-int_limit, min(int_limit, integral))
        
        output = DIFF_KP * omega_error + DIFF_KI * integral
        delta = max(-MAX_DELTA, min(MAX_DELTA, round(output)))
    
    last_error = omega_error
    
    # Vehicle response
    omega_diff = delta * (TICK_TO_METER / SAMPLE_DT) / TRACK_WIDTH
    omega_actual += (omega_diff - omega_actual) / tau * DT
    
    if step % 50 == 0 or step == 499:
        p_t = DIFF_KP * omega_error
        i_t = DIFF_KI * integral
        print(f"  {t_ms:6.0f} {omega_error:10.4f} {p_t:8.2f} {i_t:8.2f} {delta:6d} {omega_actual:10.4f} {math.degrees(omega_actual):10.2f}")

print()
print("=" * 70)
print("CONCLUSION & RECOMMENDATION")
print("=" * 70)
print("The closed-loop WORKS but MAX_DELTA=6 is dangerous!")
print()
print("Recommended MAX_DELTA values:")
for max_d in [1,2,3]:
    omega_d = max_d * (TICK_TO_METER / SAMPLE_DT) / TRACK_WIDTH
    ratio = (5+max_d)/5
    print(f"  MAX_DELTA={max_d}: outer/inner ratio={ratio:.2f}, omega_diff={math.degrees(omega_d):.1f} deg/s")

print()
print("=" * 70)
print("Re-simulate with MAX_DELTA=2")
print("=" * 70)
MAX_DELTA_NEW = 2
omega_actual = 0.0
integral = 0.0
last_error = omega_target

print(header)
for step in range(500):
    t_ms = step * DT * 1000
    omega_error = omega_target - omega_actual
    
    if abs(omega_error) < 0.05:
        delta = 0
        integral = 0
    else:
        integral += omega_error * DT
        int_limit = MAX_DELTA_NEW / (DIFF_KI + 0.001)
        integral = max(-int_limit, min(int_limit, integral))
        
        output = DIFF_KP * omega_error + DIFF_KI * integral
        delta = max(-MAX_DELTA_NEW, min(MAX_DELTA_NEW, round(output)))
    
    last_error = omega_error
    
    omega_diff = delta * (TICK_TO_METER / SAMPLE_DT) / TRACK_WIDTH
    omega_actual += (omega_diff - omega_actual) / tau * DT
    
    if step % 50 == 0 or step == 499:
        p_t = DIFF_KP * omega_error
        i_t = DIFF_KI * integral
        print(f"  {t_ms:6.0f} {omega_error:10.4f} {p_t:8.2f} {i_t:8.2f} {delta:6d} {omega_actual:10.4f} {math.degrees(omega_actual):10.2f}")

print()
print("=" * 70)
print("Re-simulate with MAX_DELTA=3")
print("=" * 70)
MAX_DELTA_NEW = 3
omega_actual = 0.0
integral = 0.0
last_error = omega_target

print(header)
for step in range(500):
    t_ms = step * DT * 1000
    omega_error = omega_target - omega_actual
    
    if abs(omega_error) < 0.05:
        delta = 0
        integral = 0
    else:
        integral += omega_error * DT
        int_limit = MAX_DELTA_NEW / (DIFF_KI + 0.001)
        integral = max(-int_limit, min(int_limit, integral))
        
        output = DIFF_KP * omega_error + DIFF_KI * integral
        delta = max(-MAX_DELTA_NEW, min(MAX_DELTA_NEW, round(output)))
    
    last_error = omega_error
    
    omega_diff = delta * (TICK_TO_METER / SAMPLE_DT) / TRACK_WIDTH
    omega_actual += (omega_diff - omega_actual) / tau * DT
    
    if step % 50 == 0 or step == 499:
        p_t = DIFF_KP * omega_error
        i_t = DIFF_KI * integral
        print(f"  {t_ms:6.0f} {omega_error:10.4f} {p_t:8.2f} {i_t:8.2f} {delta:6d} {omega_actual:10.4f} {math.degrees(omega_actual):10.2f}")
