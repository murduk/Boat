/** Touch + keyboard input for the web simulator. */

import { DEG2RAD, KT_TO_MS, clamp } from "./boat.js";

export function createInput(opts) {
  const {
    getBoat,
    cam,
    canvas,
    env,
    getHelmHold,
    setHelmHold,
    onReset,
    onToggleHelp,
    onFollow,
    syncUI,
  } = opts;

  const keys = new Set();
  let helmLeftHeld = false;
  let helmRightHeld = false;
  let usingTouchThrottle = false;
  let usingTouchHelm = false;

  // Pointer pan / pinch on canvas
  const pointers = new Map();
  let lastPinchDist = null;
  let panning = false;

  function boat() {
    return getBoat();
  }

  function isTouchDevice() {
    return window.matchMedia("(pointer: coarse)").matches || navigator.maxTouchPoints > 0;
  }

  function setGear(g) {
    boat().set_gear(g);
    syncUI();
  }

  function setThrottleFrac(frac) {
    boat().set_throttle(clamp(frac, 0, 1));
    syncUI();
  }

  function setHelmDeg(deg) {
    boat().set_helm_deg(clamp(deg, -30, 30));
    syncUI();
  }

  // --- DOM controls ---
  const throttleEl = document.getElementById("throttle");
  const helmEl = document.getElementById("helm");
  const helmLeft = document.getElementById("helm-left");
  const helmRight = document.getElementById("helm-right");

  throttleEl.addEventListener("input", () => {
    usingTouchThrottle = true;
    setThrottleFrac(Number(throttleEl.value) / 100);
  });
  throttleEl.addEventListener("change", () => {
    usingTouchThrottle = true;
    setThrottleFrac(Number(throttleEl.value) / 100);
  });

  helmEl.addEventListener("input", () => {
    usingTouchHelm = true;
    setHelmDeg(Number(helmEl.value));
  });
  helmEl.addEventListener("change", () => {
    usingTouchHelm = true;
    setHelmDeg(Number(helmEl.value));
  });

  function bindHold(btn, flagSetter) {
    const down = (e) => {
      e.preventDefault();
      flagSetter(true);
    };
    const up = (e) => {
      e.preventDefault();
      flagSetter(false);
    };
    btn.addEventListener("pointerdown", down);
    btn.addEventListener("pointerup", up);
    btn.addEventListener("pointerleave", up);
    btn.addEventListener("pointercancel", up);
  }

  bindHold(helmLeft, (v) => { helmLeftHeld = v; });
  bindHold(helmRight, (v) => { helmRightHeld = v; });

  document.querySelectorAll(".gear-btn").forEach((btn) => {
    btn.addEventListener("click", () => setGear(Number(btn.dataset.gear)));
  });

  document.getElementById("btn-reset").addEventListener("click", () => onReset());
  document.getElementById("btn-help").addEventListener("click", () => onToggleHelp());
  document.getElementById("help-close").addEventListener("click", () => onToggleHelp(false));
  document.getElementById("btn-helm-mode").addEventListener("click", () => {
    setHelmHold(!getHelmHold());
    syncUI();
  });
  document.getElementById("btn-follow").addEventListener("click", () => {
    cam.follow = true;
    onFollow?.();
    syncUI();
  });

  const envBtn = document.getElementById("btn-env");
  const envPanel = document.getElementById("env-panel");
  envBtn.addEventListener("click", () => {
    envPanel.classList.toggle("hidden");
    envBtn.classList.toggle("active", !envPanel.classList.contains("hidden"));
  });

  const windSpeed = document.getElementById("wind-speed");
  const windDir = document.getElementById("wind-dir-slider");
  const currSpeed = document.getElementById("curr-speed");
  const currDir = document.getElementById("curr-dir-slider");

  function syncEnvFromSliders() {
    env.wind_speed = Number(windSpeed.value) * KT_TO_MS;
    env.wind_dir = Number(windDir.value) * DEG2RAD;
    env.current_speed = Number(currSpeed.value) * KT_TO_MS;
    env.current_dir = Number(currDir.value) * DEG2RAD;
    document.getElementById("wind-kt").textContent = Number(windSpeed.value).toFixed(1);
    document.getElementById("wind-dir").textContent = String(windDir.value);
    document.getElementById("curr-kt").textContent = Number(currSpeed.value).toFixed(1);
    document.getElementById("curr-dir").textContent = String(currDir.value);
  }

  [windSpeed, windDir, currSpeed, currDir].forEach((el) => {
    el.addEventListener("input", syncEnvFromSliders);
  });

  // --- Keyboard ---
  window.addEventListener("keydown", (e) => {
    if (e.repeat) {
      keys.add(e.code);
      return;
    }
    keys.add(e.code);
    switch (e.code) {
      case "KeyH":
        onToggleHelp();
        break;
      case "KeyT":
        setHelmHold(!getHelmHold());
        syncUI();
        break;
      case "KeyR":
        onReset();
        break;
      case "KeyQ":
        setGear(-1);
        break;
      case "KeyE":
        setGear(1);
        break;
      case "Space":
        e.preventDefault();
        setGear(0);
        break;
      case "BracketLeft":
        env.wind_speed = Math.max(0, env.wind_speed - KT_TO_MS);
        windSpeed.value = String(env.wind_speed / KT_TO_MS);
        syncEnvFromSliders();
        break;
      case "BracketRight":
        env.wind_speed += KT_TO_MS;
        windSpeed.value = String(env.wind_speed / KT_TO_MS);
        syncEnvFromSliders();
        break;
      case "Semicolon":
        env.wind_dir -= 10 * DEG2RAD;
        windDir.value = String(((env.wind_dir * (180 / Math.PI)) % 360 + 360) % 360);
        syncEnvFromSliders();
        break;
      case "Quote":
        env.wind_dir += 10 * DEG2RAD;
        windDir.value = String(((env.wind_dir * (180 / Math.PI)) % 360 + 360) % 360);
        syncEnvFromSliders();
        break;
      case "Comma":
        env.current_speed = Math.max(0, env.current_speed - 0.2 * KT_TO_MS);
        currSpeed.value = String(env.current_speed / KT_TO_MS);
        syncEnvFromSliders();
        break;
      case "Period":
        env.current_speed += 0.2 * KT_TO_MS;
        currSpeed.value = String(env.current_speed / KT_TO_MS);
        syncEnvFromSliders();
        break;
      default:
        break;
    }
  });

  window.addEventListener("keyup", (e) => {
    keys.delete(e.code);
  });

  // --- Canvas pan / pinch / wheel ---
  canvas.addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      cam.zoom(e.deltaY < 0 ? 1.1 : 1 / 1.1);
    },
    { passive: false },
  );

  function pointerDist() {
    const pts = [...pointers.values()];
    if (pts.length < 2) return null;
    const dx = pts[0].x - pts[1].x;
    const dy = pts[0].y - pts[1].y;
    return Math.hypot(dx, dy);
  }

  canvas.addEventListener("pointerdown", (e) => {
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pointers.size === 1) {
      panning = true;
      cam.follow = false;
      onFollow?.();
      syncUI();
    } else if (pointers.size === 2) {
      panning = false;
      lastPinchDist = pointerDist();
    }
  });

  canvas.addEventListener("pointermove", (e) => {
    if (!pointers.has(e.pointerId)) return;
    const prev = pointers.get(e.pointerId);
    const cur = { x: e.clientX, y: e.clientY };
    pointers.set(e.pointerId, cur);

    if (pointers.size === 2) {
      const dist = pointerDist();
      if (lastPinchDist && dist) {
        cam.zoom(dist / lastPinchDist);
      }
      lastPinchDist = dist;
      return;
    }

    if (panning && pointers.size === 1) {
      cam.panPixels(cur.x - prev.x, cur.y - prev.y);
    }
  });

  function endPointer(e) {
    pointers.delete(e.pointerId);
    if (pointers.size < 2) lastPinchDist = null;
    if (pointers.size === 0) panning = false;
  }

  canvas.addEventListener("pointerup", endPointer);
  canvas.addEventListener("pointercancel", endPointer);

  function updateContinuous(dt) {
    const b = boat();
    // Keyboard throttle (only if not driving via touch slider this frame)
    if (!usingTouchThrottle) {
      if (keys.has("KeyW")) {
        b.set_throttle(clamp(b.engine.throttle_cmd + 0.6 * dt, 0, 1));
      }
      if (keys.has("KeyS")) {
        b.set_throttle(clamp(b.engine.throttle_cmd - 0.6 * dt, 0, 1));
      }
    }

    let helmActive = false;
    const helmRate = 60 * DEG2RAD * dt;

    if (!usingTouchHelm) {
      if (keys.has("KeyA") || helmLeftHeld) {
        b.set_helm(b.helm - helmRate);
        helmActive = true;
      }
      if (keys.has("KeyD") || helmRightHeld) {
        b.set_helm(b.helm + helmRate);
        helmActive = true;
      }
    } else if (helmLeftHeld || helmRightHeld) {
      if (helmLeftHeld) b.set_helm(b.helm - helmRate);
      if (helmRightHeld) b.set_helm(b.helm + helmRate);
      helmActive = true;
    }

    if (!helmActive && !getHelmHold()) {
      const rate = 45 * DEG2RAD * dt;
      if (b.helm > 0) b.set_helm(Math.max(0, b.helm - rate));
      else if (b.helm < 0) b.set_helm(Math.min(0, b.helm + rate));
    }

    // Keyboard can take over after touch slider use
    if (keys.has("KeyW") || keys.has("KeyS")) usingTouchThrottle = false;
    if (keys.has("KeyA") || keys.has("KeyD")) usingTouchHelm = false;
  }

  return {
    updateContinuous,
    isTouchDevice,
    syncEnvFromSliders,
  };
}
