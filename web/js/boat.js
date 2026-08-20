/** Port of boat.py — 17ft deck boat docking physics. */

export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;
export const KT_TO_MS = 0.514444;

export function clamp(x, lo, hi) {
  return Math.max(lo, Math.min(hi, x));
}

export function BoatParams(overrides = {}) {
  return {
    LOA: 5.28,
    beam: 2.59,
    draft: 0.28,
    x_engine: -3.25,
    x_windage: 0.4,
    mass: 1160.0,
    Iz: 4200.0,
    mx: 70.0,
    my: 870.0,
    Jz: 1680.0,
    d_u: 110.0,
    d_u2: 40.0,
    d_v: 1200.0,
    d_v2: 500.0,
    d_r: 1900.0,
    d_r2: 1100.0,
    T_bollard_fwd: 3300.0,
    T_bollard_rev: 2200.0,
    T_idle_fwd: 110.0,
    T_idle_rev: 70.0,
    u_max_full: 18.0,
    u_max_idle: 1.5,
    max_helm: 30 * DEG2RAD,
    shift_delay: 0.5,
    throttle_tau: 0.8,
    walk_fwd: -0.02,
    walk_rev: 0.08,
    A_frontal: 2.8,
    A_lateral: 7.5,
    Cx_wind: 0.7,
    Cy_wind: 0.95,
    rho_air: 1.225,
    rho_water: 1025.0,
    ...overrides,
  };
}

export function Environment(overrides = {}) {
  return {
    wind_speed: 0.0,
    wind_dir: 0.0,
    current_speed: 0.0,
    current_dir: 0.0,
    ...overrides,
  };
}

export class EngineState {
  constructor() {
    this.throttle_cmd = 0.0;
    this.throttle_eff = 0.0;
    this.gear_cmd = 0;
    this.gear_effective = 0;
    this.shift_timer = 0.0;
  }

  set_gear(new_gear, delay) {
    new_gear = Math.sign(new_gear) | 0;
    if (new_gear !== this.gear_cmd) {
      this.gear_cmd = new_gear;
      this.gear_effective = 0;
      this.shift_timer = delay;
    }
  }

  update(dt, throttle_tau) {
    if (this.shift_timer > 0.0) {
      this.shift_timer -= dt;
      if (this.shift_timer <= 0.0) {
        this.shift_timer = 0.0;
        this.gear_effective = this.gear_cmd;
      }
    }
    let alpha = dt / Math.max(throttle_tau, 1e-6);
    alpha = Math.min(alpha, 1.0);
    this.throttle_eff += alpha * (Math.abs(this.throttle_cmd) - this.throttle_eff);
  }
}

function thrust_force(p, engine, u, v, r, helm) {
  const g = engine.gear_effective;
  if (g === 0) return [0, 0, 0];

  const tau = clamp(engine.throttle_eff, 0, 1);
  let T_static;
  if (g > 0) {
    T_static = p.T_idle_fwd + tau * (p.T_bollard_fwd - p.T_idle_fwd);
  } else {
    T_static = p.T_idle_rev + tau * (p.T_bollard_rev - p.T_idle_rev);
  }

  const u_max = p.u_max_idle + tau * (p.u_max_full - p.u_max_idle);
  const u_E = u;
  const v_E = v + p.x_engine * r;
  const sign = g > 0 ? 1.0 : -1.0;
  const tx = sign * Math.cos(helm);
  const ty = sign * Math.sin(helm);

  const u_inflow = u_E * tx + v_E * ty;
  let T = T_static * (1.0 - u_inflow / u_max);
  T = Math.max(T, 0.0);

  let Fx = T * tx;
  let Fy = T * ty;
  const walk_coef = g > 0 ? p.walk_fwd : p.walk_rev;
  Fy += walk_coef * T;
  const N = p.x_engine * Fy;
  return [Fx, Fy, N];
}

function hull_damping(p, u_rel, v_rel, r) {
  const Fx = -p.d_u * u_rel - p.d_u2 * Math.abs(u_rel) * u_rel;
  const Fy = -p.d_v * v_rel - p.d_v2 * Math.abs(v_rel) * v_rel;
  const N = -p.d_r * r - p.d_r2 * Math.abs(r) * r;
  return [Fx, Fy, N];
}

function wind_force(p, env, psi, u, v) {
  if (env.wind_speed <= 0.0) return [0, 0, 0];

  const Wx_e = env.wind_speed * Math.cos(env.wind_dir);
  const Wy_e = env.wind_speed * Math.sin(env.wind_dir);
  const Bx_e = u * Math.cos(psi) - v * Math.sin(psi);
  const By_e = u * Math.sin(psi) + v * Math.cos(psi);
  const Ax_e = Wx_e - Bx_e;
  const Ay_e = Wy_e - By_e;
  const c = Math.cos(psi);
  const s = Math.sin(psi);
  const Ax_b = c * Ax_e + s * Ay_e;
  const Ay_b = -s * Ax_e + c * Ay_e;

  const V_rel2 = Ax_b * Ax_b + Ay_b * Ay_b;
  const V_rel = Math.sqrt(V_rel2);
  if (V_rel < 1e-6) return [0, 0, 0];

  const q = 0.5 * p.rho_air * V_rel2;
  const nx = Ax_b / V_rel;
  const ny = Ay_b / V_rel;
  const Fx = q * p.Cx_wind * p.A_frontal * nx;
  const Fy = q * p.Cy_wind * p.A_lateral * ny;
  const N = Fy * p.x_windage;
  return [Fx, Fy, N];
}

function body_water_relative(env, psi, u, v) {
  if (env.current_speed <= 0.0) return [u, v];
  const Cx_e = env.current_speed * Math.cos(env.current_dir);
  const Cy_e = env.current_speed * Math.sin(env.current_dir);
  const c = Math.cos(psi);
  const s = Math.sin(psi);
  const Cx_b = c * Cx_e + s * Cy_e;
  const Cy_b = -s * Cx_e + c * Cy_e;
  return [u - Cx_b, v - Cy_b];
}

function state_derivative(state, p, engine, helm, env, out) {
  const psi = state[2];
  const u = state[3];
  const v = state[4];
  const r = state[5];

  const [u_rel, v_rel] = body_water_relative(env, psi, u, v);
  const [Ftx, Fty, Nt] = thrust_force(p, engine, u, v, r, helm);
  const [Fhx, Fhy, Nh] = hull_damping(p, u_rel, v_rel, r);
  const [Fwx, Fwy, Nw] = wind_force(p, env, psi, u, v);

  const Mu = p.mass + p.mx;
  const Mv = p.mass + p.my;
  const Mr = p.Iz + p.Jz;

  out[0] = u * Math.cos(psi) - v * Math.sin(psi);
  out[1] = u * Math.sin(psi) + v * Math.cos(psi);
  out[2] = r;
  out[3] = (Ftx + Fhx + Fwx + Mv * v * r) / Mu;
  out[4] = (Fty + Fhy + Fwy - Mu * u * r) / Mv;
  out[5] = (Nt + Nh + Nw) / Mr;
  return out;
}

function addScaled(a, b, s, out) {
  for (let i = 0; i < 6; i++) out[i] = a[i] + s * b[i];
  return out;
}

export class Boat {
  constructor(params = null, env = null, state = null) {
    this.p = params || BoatParams();
    this.env = env || Environment();
    this.state = state ? Float64Array.from(state) : new Float64Array(6);
    this.engine = new EngineState();
    this.helm = 0.0;
    this.t = 0.0;
    this._k1 = new Float64Array(6);
    this._k2 = new Float64Array(6);
    this._k3 = new Float64Array(6);
    this._k4 = new Float64Array(6);
    this._tmp = new Float64Array(6);
  }

  get x() { return this.state[0]; }
  get y() { return this.state[1]; }
  get psi() { return this.state[2]; }
  get u() { return this.state[3]; }
  get v() { return this.state[4]; }
  get r() { return this.state[5]; }
  get speed() { return Math.hypot(this.u, this.v); }

  set_throttle(throttle) {
    this.engine.throttle_cmd = clamp(throttle, -1, 1);
  }

  set_gear(gear) {
    this.engine.set_gear(gear | 0, this.p.shift_delay);
  }

  set_helm(helm_rad) {
    this.helm = clamp(helm_rad, -this.p.max_helm, this.p.max_helm);
  }

  set_helm_deg(helm_deg) {
    this.set_helm(helm_deg * DEG2RAD);
  }

  step(dt) {
    this.engine.update(dt, this.p.throttle_tau);
    const s = this.state;
    const p = this.p;
    const eng = this.engine;
    const helm = this.helm;
    const env = this.env;

    state_derivative(s, p, eng, helm, env, this._k1);
    addScaled(s, this._k1, 0.5 * dt, this._tmp);
    state_derivative(this._tmp, p, eng, helm, env, this._k2);
    addScaled(s, this._k2, 0.5 * dt, this._tmp);
    state_derivative(this._tmp, p, eng, helm, env, this._k3);
    addScaled(s, this._k3, dt, this._tmp);
    state_derivative(this._tmp, p, eng, helm, env, this._k4);

    const h = dt / 6.0;
    for (let i = 0; i < 6; i++) {
      s[i] += h * (this._k1[i] + 2 * this._k2[i] + 2 * this._k3[i] + this._k4[i]);
    }
    s[2] = ((s[2] + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
    this.t += dt;
  }
}
