/** Main loop: physics, render, HUD. */

import { Boat, BoatParams, Environment, KT_TO_MS, RAD2DEG } from "./boat.js";
import {
  Camera,
  Wake,
  COLORS,
  GRID_M,
  PIXELS_PER_M_DEFAULT,
  drawGrid,
  drawScene,
  drawBoat,
  drawVelocity,
  resizeCanvas,
} from "./render.js";
import { createInput } from "./input.js";

const DT = 1 / 100;

const canvas = document.getElementById("sim");
const hudLeft = document.getElementById("hud-left");
const hudEngine = document.getElementById("hud-engine");
const helpEl = document.getElementById("help");
const followBtn = document.getElementById("btn-follow");
const helmModeBtn = document.getElementById("btn-helm-mode");

const cam = new Camera(window.innerWidth, window.innerHeight, PIXELS_PER_M_DEFAULT);
let ctx = resizeCanvas(canvas, cam);
const wake = new Wake();
const env = Environment();

let boat = makeBoat();
let showHelp = false;
let helmHold = window.matchMedia("(pointer: coarse)").matches || navigator.maxTouchPoints > 0;

function makeBoat() {
  const b = new Boat(BoatParams(), env);
  b.state[0] = -5 * GRID_M;
  b.state[1] = 4 * GRID_M;
  b.state[2] = 0;
  b.state[3] = 0;
  b.state[4] = 0;
  b.state[5] = 0;
  b.t = 0;
  b.helm = 0;
  b.engine.throttle_cmd = 0;
  b.engine.throttle_eff = 0;
  b.engine.gear_cmd = 0;
  b.engine.gear_effective = 0;
  b.engine.shift_timer = 0;
  return b;
}

function reset() {
  boat = makeBoat();
  wake.clear();
  cam.follow = true;
  cam.centerX = boat.x;
  cam.centerY = boat.y;
  syncUI();
}

function toggleHelp(force) {
  showHelp = typeof force === "boolean" ? force : !showHelp;
  helpEl.classList.toggle("hidden", !showHelp);
}

function syncUI() {
  const thr = Math.abs(boat.engine.throttle_cmd);
  const helmDeg = boat.helm * RAD2DEG;

  document.getElementById("throttle").value = String(Math.round(thr * 100));
  document.getElementById("throttle-val").textContent = `${Math.round(thr * 100)}%`;
  document.getElementById("helm").value = String(helmDeg.toFixed(1));
  document.getElementById("helm-val").textContent = `${helmDeg >= 0 ? "+" : ""}${helmDeg.toFixed(1)}°`;

  document.querySelectorAll(".gear-btn").forEach((btn) => {
    btn.classList.toggle("active", Number(btn.dataset.gear) === boat.engine.gear_cmd);
  });

  helmModeBtn.textContent = helmHold ? "Hold" : "Spring";
  helmModeBtn.classList.toggle("active", helmHold);
  followBtn.classList.toggle("hidden", cam.follow);
}

function updateHud() {
  const speedKt = boat.speed / KT_TO_MS;
  const hdg = ((boat.psi * RAD2DEG) % 360 + 360) % 360;
  hudLeft.innerHTML =
    `SPEED  ${boat.speed.toFixed(2)} m/s  (${speedKt.toFixed(1)} kt)<br>` +
    `HDG    ${hdg.toFixed(1)}°<br>` +
    `u ${boat.u >= 0 ? "+" : ""}${boat.u.toFixed(2)}  ` +
    `v ${boat.v >= 0 ? "+" : ""}${boat.v.toFixed(2)}  ` +
    `r ${(boat.r * RAD2DEG) >= 0 ? "+" : ""}${(boat.r * RAD2DEG).toFixed(1)}°/s`;

  const gearStr = { [-1]: "REV", 0: "NEU", 1: "FWD" };
  const shift = boat.engine.shift_timer > 0;
  let gearText = `GEAR ${gearStr[boat.engine.gear_cmd]}`;
  if (shift) gearText += `  (shifting… ${boat.engine.shift_timer.toFixed(1)}s)`;

  const thr = Math.abs(boat.engine.throttle_cmd) * 100;
  const thrEff = boat.engine.throttle_eff * 100;
  const helmDeg = boat.helm * RAD2DEG;
  const mode = helmHold ? "HOLD" : "SPRING";

  hudEngine.innerHTML =
    `<span class="${shift ? "warn" : ""}">${gearText}</span><br>` +
    `THR ${thr.toFixed(0)}%  RPM ${thrEff.toFixed(0)}%<br>` +
    `HELM ${helmDeg >= 0 ? "+" : ""}${helmDeg.toFixed(1)}°  ${mode}`;
}

const input = createInput({
  getBoat: () => boat,
  cam,
  canvas,
  env,
  getHelmHold: () => helmHold,
  setHelmHold: (v) => { helmHold = v; },
  onReset: reset,
  onToggleHelp: toggleHelp,
  onFollow: () => followBtn.classList.toggle("hidden", cam.follow),
  syncUI,
});

window.addEventListener("resize", () => {
  ctx = resizeCanvas(canvas, cam);
});

cam.centerX = boat.x;
cam.centerY = boat.y;
syncUI();
input.syncEnvFromSliders();

let last = performance.now();

function frame(now) {
  let frameDt = (now - last) / 1000;
  last = now;
  frameDt = Math.min(frameDt, 0.1);

  input.updateContinuous(frameDt);

  const steps = Math.max(1, Math.round(frameDt / DT));
  for (let i = 0; i < steps; i++) boat.step(DT);

  if (boat.speed > 0.2) wake.add(boat.x, boat.y, boat.t);
  cam.followBoat(boat);

  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, cam.w, cam.h);
  drawGrid(ctx, cam);
  wake.draw(ctx, cam, boat.t);
  drawScene(ctx, cam);
  drawBoat(ctx, cam, boat);
  drawVelocity(ctx, cam, boat);

  updateHud();
  // Keep sliders roughly in sync when keyboard drives controls
  document.getElementById("throttle").value = String(Math.round(Math.abs(boat.engine.throttle_cmd) * 100));
  document.getElementById("throttle-val").textContent =
    `${Math.round(Math.abs(boat.engine.throttle_cmd) * 100)}%`;
  const hd = boat.helm * RAD2DEG;
  document.getElementById("helm").value = String(hd.toFixed(1));
  document.getElementById("helm-val").textContent = `${hd >= 0 ? "+" : ""}${hd.toFixed(1)}°`;
  document.querySelectorAll(".gear-btn").forEach((btn) => {
    btn.classList.toggle("active", Number(btn.dataset.gear) === boat.engine.gear_cmd);
  });

  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
