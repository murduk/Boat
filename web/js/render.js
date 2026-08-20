/** Camera, scene, boat, wake, and grid rendering. */

export const COLORS = {
  bg: "#123250",
  grid: "#1e415f",
  dock: "#78552a",
  dockEdge: "#463219",
  buoyRed: "#c83232",
  buoyGreen: "#28aa46",
  boatFill: "#e6e6f0",
  boatEdge: "#1e2850",
  console: "#505f82",
  wake: "#78aac8",
  vel: "#78ffa0",
  wash: "#b4dcff",
  skegFwd: "#ffc83c",
  skegRev: "#ff5a5a",
  skegNeu: "#aaaaaa",
};

export const GRID_M = 5.0;
export const PIXELS_PER_M_DEFAULT = 18.0;

export class Camera {
  constructor(w, h, ppm = PIXELS_PER_M_DEFAULT) {
    this.w = w;
    this.h = h;
    this.ppm = ppm;
    this.centerX = 0;
    this.centerY = 0;
    this.follow = true;
  }

  resize(w, h) {
    this.w = w;
    this.h = h;
  }

  worldToScreen(x, y) {
    const sx = this.w * 0.5 + (x - this.centerX) * this.ppm;
    const sy = this.h * 0.5 - (y - this.centerY) * this.ppm;
    return [sx, sy];
  }

  screenToWorld(sx, sy) {
    const x = (sx - this.w * 0.5) / this.ppm + this.centerX;
    const y = (this.h * 0.5 - sy) / this.ppm + this.centerY;
    return [x, y];
  }

  zoom(factor) {
    this.ppm = Math.max(3, Math.min(80, this.ppm * factor));
  }

  panPixels(dx, dy) {
    this.centerX -= dx / this.ppm;
    this.centerY += dy / this.ppm;
  }

  followBoat(boat, alpha = 0.08) {
    if (!this.follow) return;
    this.centerX += (boat.x - this.centerX) * alpha;
    this.centerY += (boat.y - this.centerY) * alpha;
  }
}

export const SCENE = {
  rects: [
    [-40.0, -2.0, 80.0, 2.0],
    [5.0, 0.0, 1.0, 12.0],
    [13.0, 0.0, 1.0, 12.0],
  ],
  buoys: [
    [-20.0, 15.0, COLORS.buoyRed],
    [-5.0, 15.0, COLORS.buoyGreen],
  ],
};

function boatOutline(params) {
  const L = params.LOA;
  const B = params.beam;
  const x_bow = L * 0.45;
  const x_stern = -L * 0.55;
  return [
    [x_bow, 0],
    [x_bow * 0.55, B * 0.48],
    [-x_bow * 0.1, B * 0.5],
    [x_stern, B * 0.46],
    [x_stern, -B * 0.46],
    [-x_bow * 0.1, -B * 0.5],
    [x_bow * 0.55, -B * 0.48],
  ];
}

function bodyToWorld(bx, by, psi, ox, oy) {
  const c = Math.cos(psi);
  const s = Math.sin(psi);
  return [ox + c * bx - s * by, oy + s * bx + c * by];
}

export class Wake {
  constructor(life = 6.0, maxPoints = 300) {
    this.life = life;
    this.maxPoints = maxPoints;
    this.points = [];
  }

  clear() {
    this.points.length = 0;
  }

  add(x, y, t) {
    const pts = this.points;
    if (pts.length) {
      const last = pts[pts.length - 1];
      const dx = x - last.x;
      const dy = y - last.y;
      if (dx * dx + dy * dy < 0.25) return;
    }
    pts.push({ x, y, t });
    if (pts.length > this.maxPoints) pts.shift();
  }

  draw(ctx, cam, now) {
    for (const p of this.points) {
      const age = now - p.t;
      if (age > this.life) continue;
      const a = 1 - age / this.life;
      const [sx, sy] = cam.worldToScreen(p.x, p.y);
      const r = Math.max(1, 0.3 * cam.ppm * (0.3 + 0.7 * a));
      ctx.globalAlpha = 0.35 + 0.45 * a;
      ctx.fillStyle = COLORS.wake;
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.globalAlpha = 1;
  }
}

export function drawGrid(ctx, cam) {
  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth = 1;
  const [x0, y0] = cam.screenToWorld(0, cam.h);
  const [x1, y1] = cam.screenToWorld(cam.w, 0);

  let gx = Math.floor(x0 / GRID_M) * GRID_M;
  while (gx < x1) {
    const [sx] = cam.worldToScreen(gx, 0);
    ctx.beginPath();
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, cam.h);
    ctx.stroke();
    gx += GRID_M;
  }

  let gy = Math.floor(y0 / GRID_M) * GRID_M;
  while (gy < y1) {
    const [, sy] = cam.worldToScreen(0, gy);
    ctx.beginPath();
    ctx.moveTo(0, sy);
    ctx.lineTo(cam.w, sy);
    ctx.stroke();
    gy += GRID_M;
  }
}

export function drawScene(ctx, cam) {
  for (const [rx, ry, rw, rh] of SCENE.rects) {
    const corners = [
      [rx, ry],
      [rx + rw, ry],
      [rx + rw, ry + rh],
      [rx, ry + rh],
    ];
    ctx.beginPath();
    corners.forEach(([x, y], i) => {
      const [sx, sy] = cam.worldToScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    });
    ctx.closePath();
    ctx.fillStyle = COLORS.dock;
    ctx.fill();
    ctx.strokeStyle = COLORS.dockEdge;
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  for (const [bx, by, color] of SCENE.buoys) {
    const [sx, sy] = cam.worldToScreen(bx, by);
    const r = Math.max(3, 0.5 * cam.ppm);
    ctx.beginPath();
    ctx.arc(sx, sy, r, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = "#000";
    ctx.lineWidth = 1;
    ctx.stroke();
  }
}

export function drawBoat(ctx, cam, boat) {
  const p = boat.p;
  const outline = boatOutline(p);
  const psi = boat.psi;
  const ox = boat.x;
  const oy = boat.y;

  ctx.beginPath();
  outline.forEach(([bx, by], i) => {
    const [wx, wy] = bodyToWorld(bx, by, psi, ox, oy);
    const [sx, sy] = cam.worldToScreen(wx, wy);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
  ctx.fillStyle = COLORS.boatFill;
  ctx.fill();
  ctx.strokeStyle = COLORS.boatEdge;
  ctx.lineWidth = 2;
  ctx.stroke();

  const consolePts = [
    [0.5, -0.4],
    [1.2, -0.4],
    [1.2, 0.4],
    [0.5, 0.4],
  ];
  ctx.beginPath();
  consolePts.forEach(([bx, by], i) => {
    const [wx, wy] = bodyToWorld(bx, by, psi, ox, oy);
    const [sx, sy] = cam.worldToScreen(wx, wy);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
  ctx.fillStyle = COLORS.console;
  ctx.fill();

  const engBody = [p.x_engine + 0.25, 0];
  const skegDir = [-Math.cos(boat.helm), -Math.sin(boat.helm)];
  const tipBody = [engBody[0] + skegDir[0] * 0.7, engBody[1] + skegDir[1] * 0.7];
  const [ewx, ewy] = bodyToWorld(engBody[0], engBody[1], psi, ox, oy);
  const [twx, twy] = bodyToWorld(tipBody[0], tipBody[1], psi, ox, oy);
  const [esx, esy] = cam.worldToScreen(ewx, ewy);
  const [tsx, tsy] = cam.worldToScreen(twx, twy);

  const gear = boat.engine.gear_effective;
  ctx.strokeStyle =
    gear > 0 ? COLORS.skegFwd : gear < 0 ? COLORS.skegRev : COLORS.skegNeu;
  ctx.lineWidth = Math.max(2, 0.18 * cam.ppm);
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(esx, esy);
  ctx.lineTo(tsx, tsy);
  ctx.stroke();

  const throttle = gear !== 0 ? boat.engine.throttle_eff : 0;
  if (gear !== 0 && throttle > 0.02) {
    const sign = gear > 0 ? 1 : -1;
    const washLen = 0.6 + 2.5 * throttle;
    const washTip = [
      tipBody[0] + sign * skegDir[0] * washLen,
      tipBody[1] + sign * skegDir[1] * washLen,
    ];
    const [wwx, wwy] = bodyToWorld(washTip[0], washTip[1], psi, ox, oy);
    const [wsx, wsy] = cam.worldToScreen(wwx, wwy);
    ctx.strokeStyle = COLORS.wash;
    ctx.lineWidth = Math.max(2, 0.12 * cam.ppm);
    ctx.beginPath();
    ctx.moveTo(tsx, tsy);
    ctx.lineTo(wsx, wsy);
    ctx.stroke();
  }
}

export function drawVelocity(ctx, cam, boat) {
  const [cx, cy] = cam.worldToScreen(boat.x, boat.y);
  const ux = boat.u * Math.cos(boat.psi) - boat.v * Math.sin(boat.psi);
  const uy = boat.u * Math.sin(boat.psi) + boat.v * Math.cos(boat.psi);
  if (Math.abs(ux) + Math.abs(uy) <= 0.05) return;
  const sx = cx + ux * cam.ppm * 1.5;
  const sy = cy - uy * cam.ppm * 1.5;
  ctx.strokeStyle = COLORS.vel;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(sx, sy);
  ctx.stroke();
}

export function resizeCanvas(canvas, cam) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = window.innerWidth;
  const h = window.innerHeight;
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  canvas.width = Math.floor(w * dpr);
  canvas.height = Math.floor(h * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  cam.resize(w, h);
  return ctx;
}
