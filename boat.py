"""
17ft deck boat with single outboard - low-speed / docking simulator.

State:  [x, y, psi, u, v, r]
        x, y  : earth-frame position (m)
        psi   : heading, 0 = +x axis, CCW positive (rad)
        u, v  : body-frame surge / sway velocity (m/s)
        r     : yaw rate (rad/s)

Inputs: throttle  in [-1, +1]   (user-demanded, gear sign implicit)
        helm      in [-pi/6, +pi/6]  (outboard steering angle, rad)
        gear_cmd  in {-1, 0, +1}     (reverse, neutral, forward)

Environment: wind speed + direction (TO which wind blows, earth frame)
             current speed + direction
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np


# -----------------------------------------------------------------------------
# Parameters
# -----------------------------------------------------------------------------

@dataclass
class BoatParams:
    # --- Geometry ---
    LOA: float = 5.28           # length overall [m]  (17'4")
    beam: float = 2.59          # max beam [m]  (102")
    draft: float = 0.28         # draft at loaded displacement [m]
    x_engine: float = -3.25     # engine (prop) location along x from CG [m]
                                # (aft of transom ~-2.9m by ~0.35m)
    x_windage: float = 0.40     # windage center ahead of CG [m]

    # --- Mass / inertia (loaded: dry + 2 adults + half fuel + gear) ---
    mass: float = 1160.0        # [kg]
    Iz: float = 4200.0          # yaw inertia [kg m^2]
    mx: float = 70.0            # surge added mass [kg]
    my: float = 870.0           # sway added mass [kg]
    Jz: float = 1680.0          # added yaw inertia [kg m^2]

    # --- Hull damping (linear + quadratic) ---
    d_u: float = 110.0          # [N s/m]
    d_u2: float = 40.0          # [N s^2/m^2]
    d_v: float = 1200.0         # [N s/m]
    d_v2: float = 500.0         # [N s^2/m^2]
    d_r: float = 1900.0         # [N m s]
    d_r2: float = 1100.0        # [N m s^2]

    # --- Outboard / thrust ---
    T_bollard_fwd: float = 3300.0   # full-throttle bollard thrust forward [N]
    T_bollard_rev: float = 2200.0   # full-throttle bollard thrust reverse [N]
    T_idle_fwd: float = 110.0       # idle-in-gear forward thrust [N] (~1 kt crawl)
    T_idle_rev: float = 70.0        # idle-in-gear reverse thrust [N]
    u_max_full: float = 18.0        # thrust -> 0 speed at full throttle [m/s]
    u_max_idle: float = 1.5         # thrust -> 0 speed at idle [m/s]
    max_helm: float = np.deg2rad(30.0)
    shift_delay: float = 0.5        # [s]
    throttle_tau: float = 0.8       # [s] RPM build-up time constant

    # --- Prop walk (fraction of axial thrust, at engine location) ---
    walk_fwd: float = -0.02     # small stern-to-port in forward, right-hand prop
    walk_rev: float = +0.08     # stern-to-starboard in reverse

    # --- Windage ---
    A_frontal: float = 2.8      # [m^2]
    A_lateral: float = 7.5      # [m^2]
    Cx_wind: float = 0.70
    Cy_wind: float = 0.95

    # --- Air / water density ---
    rho_air: float = 1.225
    rho_water: float = 1025.0


@dataclass
class Environment:
    wind_speed: float = 0.0      # [m/s]
    wind_dir: float = 0.0        # earth-frame direction wind blows TOWARD [rad]
    current_speed: float = 0.0   # [m/s]
    current_dir: float = 0.0     # earth-frame direction current flows TOWARD [rad]


# -----------------------------------------------------------------------------
# Controller state (shift delay machine)
# -----------------------------------------------------------------------------

@dataclass
class EngineState:
    throttle_cmd: float = 0.0          # user command, [-1, 1]; sign is ignored, gear has sign
    throttle_eff: float = 0.0          # filtered throttle (RPM build-up lag)
    gear_cmd: int = 0                  # user command, {-1, 0, +1}
    gear_effective: int = 0            # what's actually engaged
    shift_timer: float = 0.0           # [s] remaining in shift dead-zone

    def set_gear(self, new_gear: int, delay: float):
        new_gear = int(np.sign(new_gear))
        if new_gear != self.gear_cmd:
            self.gear_cmd = new_gear
            self.gear_effective = 0
            self.shift_timer = delay

    def update(self, dt: float, throttle_tau: float):
        if self.shift_timer > 0.0:
            self.shift_timer -= dt
            if self.shift_timer <= 0.0:
                self.shift_timer = 0.0
                self.gear_effective = self.gear_cmd
        # first-order lag on throttle (RPM build-up)
        alpha = dt / max(throttle_tau, 1e-6)
        alpha = min(alpha, 1.0)
        self.throttle_eff += alpha * (abs(self.throttle_cmd) - self.throttle_eff)


# -----------------------------------------------------------------------------
# Force model
# -----------------------------------------------------------------------------

def thrust_force(
    p: BoatParams,
    engine: EngineState,
    u: float, v: float, r: float,
    helm: float,
) -> tuple[float, float, float]:
    """
    Return (Fx_body, Fy_body, N) from the outboard, applied at engine location.
    """
    g = engine.gear_effective
    if g == 0:
        return 0.0, 0.0, 0.0

    # Throttle magnitude maps to static thrust, interpolated from idle to bollard.
    # gear sign chooses forward/reverse thrust curve; filtered throttle magnitude.
    tau = float(np.clip(engine.throttle_eff, 0.0, 1.0))
    if g > 0:
        T_static = p.T_idle_fwd + tau * (p.T_bollard_fwd - p.T_idle_fwd)
    else:
        T_static = p.T_idle_rev + tau * (p.T_bollard_rev - p.T_idle_rev)

    # u_max scales with throttle (roughly). Linear interp between idle and full.
    u_max = p.u_max_idle + tau * (p.u_max_full - p.u_max_idle)

    # Velocity at engine, projected onto thrust axis.
    # Engine is at (x_E, 0) in body frame, so v_at_engine = v + x_E * r (sway).
    u_E = u
    v_E = v + p.x_engine * r
    # Thrust axis unit vector in body frame depends on helm.
    # For forward gear, thrust points +x_body rotated by helm.
    # For reverse, thrust points -x_body rotated by helm (prop reversed, lower
    # unit still pivots the same way, so steering still works similarly).
    sign = 1.0 if g > 0 else -1.0
    tx = sign * np.cos(helm)
    ty = sign * np.sin(helm)

    # Inflow along thrust axis
    u_inflow = u_E * tx + v_E * ty
    # Thrust falls off with inflow velocity (simple momentum model)
    T = T_static * (1.0 - u_inflow / u_max)
    T = max(T, 0.0)

    Fx = T * tx
    Fy = T * ty

    # Prop walk: lateral kick at the engine
    walk_coef = p.walk_fwd if g > 0 else p.walk_rev
    Fy_walk = walk_coef * T   # positive = to starboard in body frame
    Fy += Fy_walk

    # Moment about CG from force at (x_engine, 0)
    N = p.x_engine * Fy    # M = r x F, z-component = x * Fy (since force y-only at y=0... plus Fx contributes 0)

    return Fx, Fy, N


def hull_damping(p: BoatParams, u_rel: float, v_rel: float, r: float
                 ) -> tuple[float, float, float]:
    Fx = -p.d_u * u_rel - p.d_u2 * abs(u_rel) * u_rel
    Fy = -p.d_v * v_rel - p.d_v2 * abs(v_rel) * v_rel
    N  = -p.d_r * r      - p.d_r2 * abs(r) * r
    return Fx, Fy, N


def wind_force(p: BoatParams, env: Environment,
               psi: float, u: float, v: float
               ) -> tuple[float, float, float]:
    """
    Wind is expressed as blowing TOWARD wind_dir in earth frame.
    Compute apparent wind in body frame, then body-frame forces.
    """
    if env.wind_speed <= 0.0:
        return 0.0, 0.0, 0.0

    # Wind velocity vector in earth frame
    Wx_e = env.wind_speed * np.cos(env.wind_dir)
    Wy_e = env.wind_speed * np.sin(env.wind_dir)

    # Boat velocity in earth frame
    Bx_e = u * np.cos(psi) - v * np.sin(psi)
    By_e = u * np.sin(psi) + v * np.cos(psi)

    # Apparent wind (wind - boat) in earth frame, then rotate to body frame
    Ax_e = Wx_e - Bx_e
    Ay_e = Wy_e - By_e
    c, s = np.cos(psi), np.sin(psi)
    Ax_b =  c * Ax_e + s * Ay_e
    Ay_b = -s * Ax_e + c * Ay_e

    V_rel2 = Ax_b * Ax_b + Ay_b * Ay_b
    V_rel = np.sqrt(V_rel2)
    if V_rel < 1e-6:
        return 0.0, 0.0, 0.0

    # Relative wind angle in body frame (from which wind is blowing — onto boat)
    # We want force direction along the apparent wind vector.
    # Use coefficient * A * q, with sign from direction cosines.
    q = 0.5 * p.rho_air * V_rel2
    # Simplified: decompose into axial and lateral components of apparent wind
    # direction (unit vector), scale each by its coefficient and reference area.
    nx = Ax_b / V_rel
    ny = Ay_b / V_rel
    Fx = q * p.Cx_wind * p.A_frontal * nx
    Fy = q * p.Cy_wind * p.A_lateral * ny
    N  = Fy * p.x_windage
    return Fx, Fy, N


def body_water_relative(env: Environment, psi: float, u: float, v: float
                        ) -> tuple[float, float]:
    """Subtract current (in body frame) from body velocity."""
    if env.current_speed <= 0.0:
        return u, v
    Cx_e = env.current_speed * np.cos(env.current_dir)
    Cy_e = env.current_speed * np.sin(env.current_dir)
    c, s = np.cos(psi), np.sin(psi)
    Cx_b =  c * Cx_e + s * Cy_e
    Cy_b = -s * Cx_e + c * Cy_e
    return u - Cx_b, v - Cy_b


# -----------------------------------------------------------------------------
# State derivative
# -----------------------------------------------------------------------------

def state_derivative(state: np.ndarray,
                     p: BoatParams,
                     engine: EngineState,
                     helm: float,
                     env: Environment) -> np.ndarray:
    x, y, psi, u, v, r = state

    # Water-relative velocities for hydrodynamic drag
    u_rel, v_rel = body_water_relative(env, psi, u, v)

    # Forces
    Ftx, Fty, Nt = thrust_force(p, engine, u, v, r, helm)
    Fhx, Fhy, Nh = hull_damping(p, u_rel, v_rel, r)
    Fwx, Fwy, Nw = wind_force(p, env, psi, u, v)

    # Effective masses
    Mu = p.mass + p.mx
    Mv = p.mass + p.my
    Mr = p.Iz + p.Jz

    # Body-frame equations of motion with rigid-body coupling (Coriolis)
    u_dot = (Ftx + Fhx + Fwx + Mv * v * r) / Mu
    v_dot = (Fty + Fhy + Fwy - Mu * u * r) / Mv
    r_dot = (Nt  + Nh  + Nw ) / Mr

    # Kinematics
    x_dot = u * np.cos(psi) - v * np.sin(psi)
    y_dot = u * np.sin(psi) + v * np.cos(psi)
    psi_dot = r

    return np.array([x_dot, y_dot, psi_dot, u_dot, v_dot, r_dot])


# -----------------------------------------------------------------------------
# Simulator wrapper with RK4
# -----------------------------------------------------------------------------

class Boat:
    def __init__(self,
                 params: BoatParams | None = None,
                 env: Environment | None = None,
                 state: np.ndarray | None = None):
        self.p = params or BoatParams()
        self.env = env or Environment()
        self.state = state if state is not None else np.zeros(6)
        self.engine = EngineState()
        self.helm = 0.0
        self.t = 0.0

    # --- control interface ---
    def set_throttle(self, throttle: float):
        self.engine.throttle_cmd = float(np.clip(throttle, -1.0, 1.0))

    def set_gear(self, gear: int):
        self.engine.set_gear(int(gear), self.p.shift_delay)

    def set_helm(self, helm_rad: float):
        self.helm = float(np.clip(helm_rad, -self.p.max_helm, self.p.max_helm))

    def set_helm_deg(self, helm_deg: float):
        self.set_helm(np.deg2rad(helm_deg))

    # --- simulation ---
    def step(self, dt: float):
        """Advance one step using RK4. Caller should use dt ~ 0.01 s."""
        self.engine.update(dt, self.p.throttle_tau)
        s = self.state
        k1 = state_derivative(s,               self.p, self.engine, self.helm, self.env)
        k2 = state_derivative(s + 0.5*dt*k1,   self.p, self.engine, self.helm, self.env)
        k3 = state_derivative(s + 0.5*dt*k2,   self.p, self.engine, self.helm, self.env)
        k4 = state_derivative(s + dt*k3,       self.p, self.engine, self.helm, self.env)
        self.state = s + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        # wrap heading to [-pi, pi]
        self.state[2] = (self.state[2] + np.pi) % (2*np.pi) - np.pi
        self.t += dt

    # --- accessors ---
    @property
    def x(self): return self.state[0]
    @property
    def y(self): return self.state[1]
    @property
    def psi(self): return self.state[2]
    @property
    def u(self): return self.state[3]
    @property
    def v(self): return self.state[4]
    @property
    def r(self): return self.state[5]
    @property
    def speed(self): return float(np.hypot(self.u, self.v))
