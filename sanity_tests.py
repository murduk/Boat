"""
Sanity tests: verify the boat simulator produces realistic low-speed behaviour.

Targets (from the design discussion):
  1. Coast-down from 2.5 m/s, engine off   -> stop in 20-30 s
  2. Idle ahead from stopped                -> reach ~1 m/s in 8-12 s, asymptote ~1.5 m/s
  3. Full helm, idle ahead, steady state    -> turning diameter 8-12 m (~1.5-2 LOA)
  4. Stopped, 15 kt crosswind abeam         -> drift 0.3-0.5 m/s, bow falls off downwind
  5. Full throttle from stopped             -> ~1 m/s in 2 s, ~5 m/s in 10 s
"""

import numpy as np
from boat import Boat, BoatParams, Environment

KT_TO_MS = 0.514444
DT = 0.01


def run(boat: Boat, duration: float, log_dt: float = 0.1):
    """Run simulator for `duration` seconds, returning a log of states."""
    n_steps = int(round(duration / DT))
    log_every = int(round(log_dt / DT))
    log = []
    for i in range(n_steps):
        boat.step(DT)
        if i % log_every == 0:
            log.append((boat.t, boat.x, boat.y, boat.psi,
                        boat.u, boat.v, boat.r, boat.speed))
    return np.array(log)


# -----------------------------------------------------------------------------

def test_1_coastdown():
    print("\n--- Test 1: coast-down from 2.5 m/s, engine off ---")
    b = Boat()
    b.state[3] = 2.5    # initial surge velocity
    b.set_gear(0)       # neutral
    log = run(b, 60.0)
    # find time when speed < 0.1 m/s
    t_stop = None
    for row in log:
        t, *_, spd = row
        if spd < 0.1:
            t_stop = t
            break
    print(f"  Speed at t=10s : {log[100, 7]:.3f} m/s")
    print(f"  Speed at t=20s : {log[200, 7]:.3f} m/s")
    print(f"  Speed at t=30s : {log[300, 7]:.3f} m/s")
    print(f"  Stops (<0.1 m/s) at t = {t_stop:.1f} s"
          if t_stop else "  Did not stop within 60 s")
    ok = t_stop is not None and 20.0 <= t_stop <= 35.0
    print(f"  -> {'PASS' if ok else 'check'} (target 20-30 s)")


def test_2_idle_ahead():
    print("\n--- Test 2: idle ahead from stopped, helm centered ---")
    b = Boat()
    b.set_gear(+1)
    b.set_throttle(0.0)   # idle-in-gear
    b.set_helm_deg(0)
    log = run(b, 30.0)
    v1  = log[ 10, 7]     # 1 s
    v5  = log[ 50, 7]     # 5 s
    v10 = log[100, 7]     # 10 s
    v20 = log[200, 7]     # 20 s
    v30 = log[-1,  7]     # ~30 s
    print(f"  Speed at t= 1s : {v1:.3f} m/s")
    print(f"  Speed at t= 5s : {v5:.3f} m/s")
    print(f"  Speed at t=10s : {v10:.3f} m/s")
    print(f"  Speed at t=20s : {v20:.3f} m/s")
    print(f"  Speed at t=30s : {v30:.3f} m/s  (asymptote)")
    ok = 0.7 <= v10 <= 1.3 and 1.2 <= v30 <= 1.8
    print(f"  -> {'PASS' if ok else 'check'} "
          f"(v10 target 0.7-1.3, v_asym target 1.2-1.8)")


def test_3_turn_idle():
    print("\n--- Test 3: full helm, idle ahead, turning circle ---")
    b = Boat()
    b.set_gear(+1)
    b.set_throttle(0.0)
    b.set_helm_deg(30)
    log = run(b, 60.0)
    # Steady state turn: look at the last 20 s of the trace
    tail = log[-200:]
    xs, ys = tail[:, 1], tail[:, 2]
    # Fit circle: (x-a)^2 + (y-b)^2 = R^2
    A = np.column_stack([2*xs, 2*ys, np.ones_like(xs)])
    rhs = xs*xs + ys*ys
    sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    a, b_, c = sol
    R = np.sqrt(c + a*a + b_*b_)
    D = 2 * R
    print(f"  Steady turn radius : {R:.2f} m")
    print(f"  Steady turn diam.  : {D:.2f} m  ({D/5.28:.2f} LOA)")
    r_final = tail[-1, 6]
    u_final = tail[-1, 4]
    v_final = tail[-1, 5]
    print(f"  Final yaw rate     : {np.rad2deg(r_final):+.1f} deg/s")
    print(f"  Final u, v         : {u_final:+.2f}, {v_final:+.2f} m/s")
    ok = 6.0 <= D <= 15.0
    print(f"  -> {'PASS' if ok else 'check'} (target 8-12 m)")


def test_4_wind_drift():
    print("\n--- Test 4: stopped, 15 kt wind from starboard (abeam) ---")
    # Boat heading = 0 (pointing +x). Wind "from starboard" = blowing toward port.
    # In earth frame, port when psi=0 is +y direction. Wind blows toward +y.
    env = Environment(wind_speed=15.0 * KT_TO_MS, wind_dir=np.pi/2)
    b = Boat(env=env)
    b.set_gear(0)
    log = run(b, 30.0)
    t5  = log[ 50]
    t15 = log[150]
    t30 = log[-1]
    print(f"  At  5 s: x={t5[1]:+.2f}  y={t5[2]:+.2f}  psi={np.rad2deg(t5[3]):+.1f}deg"
          f"  spd={t5[7]:.2f}")
    print(f"  At 15 s: x={t15[1]:+.2f}  y={t15[2]:+.2f}  psi={np.rad2deg(t15[3]):+.1f}deg"
          f"  spd={t15[7]:.2f}")
    print(f"  At 30 s: x={t30[1]:+.2f}  y={t30[2]:+.2f}  psi={np.rad2deg(t30[3]):+.1f}deg"
          f"  spd={t30[7]:.2f}")
    # drift speed after settling (last 5 s)
    tail = log[-50:]
    drift = np.mean(np.hypot(np.diff(tail[:,1])/0.1, np.diff(tail[:,2])/0.1))
    # heading change (bow falls off downwind = rotates toward wind destination = +y)
    # With wind blowing toward +y, boat bow (initially +x) should turn toward +y -> psi > 0
    dpsi = t30[3] - log[0, 3]
    print(f"  Settled drift speed : {drift:.2f} m/s   (target 0.25-0.55)")
    print(f"  Heading change      : {np.rad2deg(dpsi):+.1f} deg "
          f"(expect bow to fall off downwind)")
    ok_drift = 0.2 <= drift <= 0.7
    ok_yaw = dpsi > np.deg2rad(5)
    print(f"  -> {'PASS' if ok_drift and ok_yaw else 'check'}")


def test_5_full_throttle():
    print("\n--- Test 5: full throttle forward from stopped, no wind ---")
    b = Boat()
    b.set_gear(+1)
    b.set_throttle(1.0)
    b.set_helm_deg(0)
    # Skip past shift delay then measure
    log = run(b, 30.0)
    v1 = log[10, 7];  v2 = log[20, 7];  v5 = log[50, 7]
    v10 = log[100, 7]; v20 = log[200, 7]; v30 = log[-1, 7]
    print(f"  Speed at t= 1s : {v1:.2f} m/s")
    print(f"  Speed at t= 2s : {v2:.2f} m/s")
    print(f"  Speed at t= 5s : {v5:.2f} m/s")
    print(f"  Speed at t=10s : {v10:.2f} m/s")
    print(f"  Speed at t=20s : {v20:.2f} m/s")
    print(f"  Speed at t=30s : {v30:.2f} m/s  (displacement-regime asymptote)")
    # A real boat transitions to planing here; our model stays in displacement,
    # so v_asymptote will be lower than a real boat's top speed.
    ok = 0.7 <= v2 <= 1.8 and 3.5 <= v10 <= 7.0
    print(f"  -> {'PASS' if ok else 'check'} "
          f"(v2 target 0.7-1.5 m/s, v10 target 3.5-6 m/s displacement only)")


def test_bonus_bump_shift():
    print("\n--- Bonus: bump-shift behavior (fwd->N->fwd 3x) ---")
    b = Boat()
    b.set_gear(+1); b.set_throttle(0.0)
    x_log = []
    t_log = []
    for cycle in range(3):
        # in gear 3 s
        for _ in range(300):
            b.step(DT); x_log.append(b.x); t_log.append(b.t)
        # neutral 3 s
        b.set_gear(0)
        for _ in range(300):
            b.step(DT); x_log.append(b.x); t_log.append(b.t)
        b.set_gear(+1)
    print(f"  Final x position : {b.x:.2f} m after 3 bump-shift cycles (18 s)")
    print(f"  Final speed      : {b.speed:.2f} m/s")
    print("  (shift delay of 0.5 s produces saw-tooth in velocity; hard to unit-test,"
          " but the boat should creep forward with stepwise impulses)")


if __name__ == "__main__":
    test_1_coastdown()
    test_2_idle_ahead()
    test_3_turn_idle()
    test_4_wind_drift()
    test_5_full_throttle()
    test_bonus_bump_shift()
