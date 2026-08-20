"""
Interactive 2D top-down boat simulator.

Controls
--------
  W / S        : throttle up / down (0..100% of current gear)
  A / D        : helm left / right (rudder)
  T            : toggle helm hold (rudder stays) / spring-to-center
  Q / E        : shift to reverse / forward (Space = neutral)
  Space        : neutral
  R            : reset
  [ / ]        : decrease / increase wind speed
  ; / '        : rotate wind direction
  H            : toggle help overlay
  Esc          : quit

  Mouse wheel  : zoom in/out
  Mouse drag   : pan camera

Run:
  pip install pygame numpy
  python interactive_sim.py
"""

import math
import sys
import numpy as np
import pygame

from boat import Boat, BoatParams, Environment

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
WINDOW_W, WINDOW_H = 1280, 800
BG_COLOR       = (18,  50,  80)       # deep water
DOCK_COLOR     = (120,  85,  40)
BUOY_RED       = (200,  50,  50)
BUOY_GREEN     = ( 40, 170,  70)
BOAT_FILL      = (230, 230, 240)
BOAT_EDGE      = ( 30,  40,  80)
WAKE_COLOR     = (120, 170, 200)
HUD_COLOR      = (240, 240, 240)
HUD_DIM        = (160, 170, 180)
WARN_COLOR     = (250, 190,  80)

DT = 1.0 / 100.0     # physics at 100 Hz
FPS = 60             # rendering at 60 Hz — multiple physics steps per frame
PIXELS_PER_M_DEFAULT = 18.0
GRID_M = 5.0            # world-space grid spacing [m]

KT_TO_MS = 0.514444


# -----------------------------------------------------------------------------
# Camera: converts world meters <-> screen pixels, with zoom and pan
# -----------------------------------------------------------------------------
class Camera:
    def __init__(self, w, h, ppm):
        self.w = w; self.h = h
        self.ppm = ppm                # pixels per meter
        self.center = np.array([0.0, 0.0])  # world point at screen center
        self.follow = True

    def world_to_screen(self, x, y):
        sx = self.w * 0.5 + (x - self.center[0]) * self.ppm
        # note: world y is "north" so screen y is flipped
        sy = self.h * 0.5 - (y - self.center[1]) * self.ppm
        return sx, sy

    def screen_to_world(self, sx, sy):
        x = (sx - self.w * 0.5) / self.ppm + self.center[0]
        y = (self.h * 0.5 - sy) / self.ppm + self.center[1]
        return x, y

    def zoom(self, factor, around=None):
        self.ppm = float(np.clip(self.ppm * factor, 3.0, 80.0))


# -----------------------------------------------------------------------------
# Dock / scene
# -----------------------------------------------------------------------------
class Scene:
    """Static geometry — a slip with finger piers."""
    def __init__(self):
        # A main dock along y=0 from x=-40 to x=40 (1m thick, to the south side)
        # Two finger piers creating a slip between x=5 and x=13
        self.rects = [
            # main dock (south of y=0)
            (-40.0, -2.0, 80.0, 2.0),
            # left finger pier (into +y)
            ( 5.0,  0.0,  1.0, 12.0),
            # right finger pier
            (13.0,  0.0,  1.0, 12.0),
        ]
        # Buoys (red = port side entering, green = stbd)
        self.buoys = [
            (-20.0, 15.0, BUOY_RED),
            ( -5.0, 15.0, BUOY_GREEN),
        ]

    def draw(self, surf, cam: Camera):
        for (rx, ry, rw, rh) in self.rects:
            # four corners
            pts_w = [(rx, ry), (rx+rw, ry), (rx+rw, ry+rh), (rx, ry+rh)]
            pts_s = [cam.world_to_screen(*p) for p in pts_w]
            pygame.draw.polygon(surf, DOCK_COLOR, pts_s)
            pygame.draw.polygon(surf, (70, 50, 25), pts_s, 2)
        for (bx, by, color) in self.buoys:
            sx, sy = cam.world_to_screen(bx, by)
            pygame.draw.circle(surf, color, (int(sx), int(sy)),
                               max(3, int(0.5 * cam.ppm)))
            pygame.draw.circle(surf, (0, 0, 0), (int(sx), int(sy)),
                               max(3, int(0.5 * cam.ppm)), 1)


# -----------------------------------------------------------------------------
# Boat rendering
# -----------------------------------------------------------------------------
def boat_outline(params: BoatParams):
    """Top-down hull outline, body-frame with CG at origin. +x = bow."""
    L = params.LOA
    B = params.beam
    x_bow   = +L * 0.45
    x_stern = -L * 0.55
    pts = [
        ( x_bow,                 0.0       ),
        ( x_bow * 0.55,         +B * 0.48  ),
        (-x_bow * 0.1,          +B * 0.50  ),
        ( x_stern,              +B * 0.46  ),
        ( x_stern,              -B * 0.46  ),
        (-x_bow * 0.1,          -B * 0.50  ),
        ( x_bow * 0.55,         -B * 0.48  ),
    ]
    return np.array(pts)


def draw_boat(surf, cam: Camera, boat: Boat):
    p = boat.p
    outline = boat_outline(p)
    c, s = np.cos(boat.psi), np.sin(boat.psi)
    R = np.array([[c, -s], [s, c]])
    pts_w = outline @ R.T + np.array([boat.x, boat.y])
    pts_s = [cam.world_to_screen(*pw) for pw in pts_w]

    # Wake trail dots could go here (skipped for now)

    pygame.draw.polygon(surf, BOAT_FILL, pts_s)
    pygame.draw.polygon(surf, BOAT_EDGE, pts_s, 2)

    # Console block (just visual)
    console = np.array([[0.5, -0.4], [1.2, -0.4], [1.2, 0.4], [0.5, 0.4]])
    console_w = console @ R.T + np.array([boat.x, boat.y])
    pygame.draw.polygon(surf, (80, 95, 130),
                        [cam.world_to_screen(*p) for p in console_w])

    # Outboard lower unit: draw the skeg extending aft of the engine mounting
    # point, rotated by the helm angle. The skeg is what you see from above
    # on a real outboard — the leg that pivots with the wheel.
    #
    # Sign convention:
    #   helm > 0  =>  outboard points to STARBOARD (body -y)
    #                 In forward gear, prop wash exits to starboard, pushing
    #                 stern to port — bow turns starboard.
    #                 In reverse gear, prop pulls stern to starboard — bow
    #                 turns port.
    engine_pos_body = np.array([p.x_engine + 0.25, 0.0])  # pivot point (transom)
    engine_pos_world = R @ engine_pos_body + np.array([boat.x, boat.y])

    # Skeg extends aft (body -x) and rotated so helm>0 points to starboard (-y)
    gear = boat.engine.gear_effective
    skeg_dir_body = np.array([-np.cos(boat.helm), -np.sin(boat.helm)])
    skeg_tip_body = engine_pos_body + skeg_dir_body * 0.7
    skeg_tip_world = R @ skeg_tip_body + np.array([boat.x, boat.y])

    # Color = state: yellow=forward, red=reverse, gray=neutral
    skeg_color = (
        (255, 200, 60) if gear > 0 else
        (255,  90, 90) if gear < 0 else
        (170, 170, 170)
    )
    pygame.draw.line(
        surf, skeg_color,
        cam.world_to_screen(*engine_pos_world),
        cam.world_to_screen(*skeg_tip_world),
        max(2, int(0.18 * cam.ppm)),
    )

    # Prop wash puff: a little arrow showing water direction when in gear + throttle
    throttle = boat.engine.throttle_eff if gear != 0 else 0.0
    if gear != 0 and throttle > 0.02:
        # Forward gear: water is thrown aft along the skeg direction (same
        # as skeg_dir_body).
        # Reverse gear: water is pulled in from aft and expelled forward, so
        # wash direction is opposite of skeg.
        sign = +1.0 if gear > 0 else -1.0
        wash_dir_body = sign * skeg_dir_body
        wash_tip_body = skeg_tip_body + wash_dir_body * (0.6 + 2.5 * throttle)
        wash_tip_world = R @ wash_tip_body + np.array([boat.x, boat.y])
        pygame.draw.line(
            surf, (180, 220, 255),
            cam.world_to_screen(*skeg_tip_world),
            cam.world_to_screen(*wash_tip_world),
            max(2, int(0.12 * cam.ppm)),
        )


def draw_velocity_vectors(surf, cam: Camera, boat: Boat):
    """Small arrows showing CG velocity and wind."""
    cx, cy = cam.world_to_screen(boat.x, boat.y)
    # boat velocity in earth frame
    ux = boat.u * np.cos(boat.psi) - boat.v * np.sin(boat.psi)
    uy = boat.u * np.sin(boat.psi) + boat.v * np.cos(boat.psi)
    sx = cx + ux * cam.ppm * 1.5
    sy = cy - uy * cam.ppm * 1.5
    if abs(ux) + abs(uy) > 0.05:
        pygame.draw.line(surf, (120, 255, 160), (cx, cy), (sx, sy), 2)


# -----------------------------------------------------------------------------
# HUD
# -----------------------------------------------------------------------------
class HUD:
    def __init__(self):
        pygame.font.init()
        self.font = pygame.font.SysFont("consolas,menlo,dejavusansmono", 16)
        self.font_big = pygame.font.SysFont("consolas,menlo,dejavusansmono",
                                            20, bold=True)
        self.font_small = pygame.font.SysFont("consolas,menlo,dejavusansmono", 13)

    def draw(self, surf, boat: Boat, env: Environment, show_help: bool,
             helm_hold: bool = False):
        w, h = surf.get_size()

        # --- Top-left: state readout ---
        lines = [
            ("SPEED",   f"{boat.speed:6.2f} m/s  ({boat.speed/KT_TO_MS:5.1f} kt)"),
            ("HDG",     f"{(np.rad2deg(boat.psi)+360)%360:6.1f} deg"),
            ("SURGE u", f"{boat.u:+6.2f} m/s"),
            ("SWAY  v", f"{boat.v:+6.2f} m/s"),
            ("YAW r  ", f"{np.rad2deg(boat.r):+6.1f} deg/s"),
        ]
        y = 12
        for label, val in lines:
            t = self.font.render(f"{label:7} {val}", True, HUD_COLOR)
            surf.blit(t, (14, y)); y += 20

        # --- Top-right: env ---
        env_lines = [
            f"WIND  {env.wind_speed/KT_TO_MS:4.1f} kt  "
            f"TOWARD {(np.rad2deg(env.wind_dir)+360)%360:5.1f}°",
            f"CURR  {env.current_speed/KT_TO_MS:4.1f} kt  "
            f"TOWARD {(np.rad2deg(env.current_dir)+360)%360:5.1f}°",
        ]
        for i, s in enumerate(env_lines):
            t = self.font.render(s, True, HUD_DIM)
            surf.blit(t, (w - t.get_width() - 14, 12 + i*20))

        # --- Bottom-left: engine status ---
        gear_eff = boat.engine.gear_effective
        gear_cmd = boat.engine.gear_cmd
        gear_str = {-1: "REV", 0: "NEU", 1: "FWD"}
        shift_active = boat.engine.shift_timer > 0
        gear_text = f"GEAR {gear_str[gear_cmd]}"
        if shift_active:
            gear_text += f"  (shifting… {boat.engine.shift_timer:.1f}s)"
        elif gear_eff != gear_cmd:
            gear_text += "  (!)"

        color = WARN_COLOR if shift_active else HUD_COLOR
        t = self.font_big.render(gear_text, True, color)
        surf.blit(t, (14, h - 110))

        thr = abs(boat.engine.throttle_cmd)
        thr_eff = boat.engine.throttle_eff
        t = self.font.render(f"THROTTLE {thr*100:4.0f}%  (RPM {thr_eff*100:4.0f}%)",
                             True, HUD_COLOR)
        surf.blit(t, (14, h - 80))

        helm_deg = np.rad2deg(boat.helm)
        helm_mode = "HOLD" if helm_hold else "SPRING"
        t = self.font.render(f"HELM     {helm_deg:+5.1f}°  {helm_mode}", True, HUD_COLOR)
        surf.blit(t, (14, h - 58))

        # Throttle bar
        self._bar(surf, 14, h - 32, 200, 10, thr, (80, 180, 255))
        # Helm bar (centered)
        self._bar_signed(surf, 230, h - 32, 200, 10,
                         helm_deg / 30.0, (120, 220, 140))

        # --- Help overlay ---
        if show_help:
            self._draw_help(surf)

    def _bar(self, surf, x, y, w, h, frac, color):
        pygame.draw.rect(surf, (60, 60, 70), (x, y, w, h))
        pygame.draw.rect(surf, color, (x, y, int(w * max(0, min(1, frac))), h))
        pygame.draw.rect(surf, HUD_DIM, (x, y, w, h), 1)

    def _bar_signed(self, surf, x, y, w, h, v, color):
        pygame.draw.rect(surf, (60, 60, 70), (x, y, w, h))
        mid = x + w // 2
        length = int((w // 2) * max(-1, min(1, v)))
        if length >= 0:
            pygame.draw.rect(surf, color, (mid, y, length, h))
        else:
            pygame.draw.rect(surf, color, (mid + length, y, -length, h))
        pygame.draw.line(surf, HUD_COLOR, (mid, y-1), (mid, y+h+1), 1)
        pygame.draw.rect(surf, HUD_DIM, (x, y, w, h), 1)

    def _draw_help(self, surf):
        lines = [
            "KEY CONTROLS",
            "  W / S    throttle up / down",
            "  A / D    helm left / right",
            "  T        helm hold / spring-to-center",
            "  Q        shift to reverse",
            "  E        shift to forward",
            "  Space    neutral",
            "  R        reset",
            "  [ / ]    wind speed  ↓ / ↑",
            "  ; / '    wind dir    rot",
            "  , / .    current speed ↓ / ↑",
            "  H        toggle this help",
            "  Mouse wheel  zoom",
            "  Esc      quit",
        ]
        pad = 10
        line_h = 18
        box_w = 320
        box_h = pad*2 + line_h * len(lines)
        x0 = surf.get_width() - box_w - 14
        y0 = 60
        s = pygame.Surface((box_w, box_h), pygame.SRCALPHA)
        s.fill((0, 0, 0, 180))
        surf.blit(s, (x0, y0))
        for i, ln in enumerate(lines):
            font = self.font_big if i == 0 else self.font_small
            t = font.render(ln, True, HUD_COLOR)
            surf.blit(t, (x0 + pad, y0 + pad + i * line_h))


# -----------------------------------------------------------------------------
# Wake trail: store (x, y, t) points and fade them out
# -----------------------------------------------------------------------------
class Wake:
    def __init__(self, life=6.0, max_points=300):
        self.life = life
        self.max_points = max_points
        self.points = []   # list of (x, y, t_created)

    def add(self, x, y, t):
        if self.points:
            lx, ly, _ = self.points[-1]
            if (x-lx)**2 + (y-ly)**2 < 0.25:
                return
        self.points.append((x, y, t))
        if len(self.points) > self.max_points:
            self.points.pop(0)

    def draw(self, surf, cam, now):
        for (x, y, tc) in self.points:
            age = now - tc
            if age > self.life:
                continue
            a = 1.0 - age / self.life
            sx, sy = cam.world_to_screen(x, y)
            col = (WAKE_COLOR[0], WAKE_COLOR[1], WAKE_COLOR[2], int(180*a))
            r = max(1, int(0.3 * cam.ppm * (0.3 + 0.7*a)))
            pygame.draw.circle(surf, WAKE_COLOR, (int(sx), int(sy)), r)


# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("17ft Deck Boat — Docking Simulator")
    clock = pygame.time.Clock()

    cam = Camera(WINDOW_W, WINDOW_H, PIXELS_PER_M_DEFAULT)
    scene = Scene()
    hud = HUD()
    wake = Wake()

    env = Environment(wind_speed=0.0, wind_dir=0.0)

    def make_boat():
        b = Boat(env=env)
        # Start on a grid intersection, outside the slip, heading +x
        b.state = np.array([-5.0 * GRID_M, 4.0 * GRID_M, 0.0, 0.0, 0.0, 0.0])
        return b

    boat = make_boat()

    # Input state
    keys_held = set()
    show_help = True
    helm_hold = False
    dragging = False
    drag_start = None

    def set_helm_rate(dt, direction):
        # direction: -1 = left (starboard helm), +1 = right (port helm)
        # Wheel rate: ~60 deg/s in helm angle (rudder sweeps 30 deg in 0.5 s)
        rate = np.deg2rad(60.0) * dt
        new_helm = boat.helm + direction * rate
        boat.set_helm(new_helm)

    def return_helm_to_center(dt):
        rate = np.deg2rad(45.0) * dt
        if boat.helm > 0:
            boat.set_helm(max(0.0, boat.helm - rate))
        elif boat.helm < 0:
            boat.set_helm(min(0.0, boat.helm + rate))

    def change_throttle(dt, direction):
        rate = 0.6 * dt   # full sweep in ~1.7 s
        new_thr = boat.engine.throttle_cmd + direction * rate
        boat.set_throttle(float(np.clip(new_thr, 0.0, 1.0)))

    running = True
    while running:
        frame_dt = clock.tick(FPS) / 1000.0
        # cap dt to avoid big jumps if the window was dragged
        frame_dt = min(frame_dt, 0.1)

        # --- events ---
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                keys_held.add(ev.key)
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_h:
                    show_help = not show_help
                elif ev.key == pygame.K_t:
                    helm_hold = not helm_hold
                elif ev.key == pygame.K_r:
                    boat = make_boat()
                    wake = Wake()
                elif ev.key == pygame.K_q:
                    boat.set_gear(-1)
                elif ev.key == pygame.K_e:
                    boat.set_gear(+1)
                elif ev.key == pygame.K_SPACE:
                    boat.set_gear(0)
                elif ev.key == pygame.K_LEFTBRACKET:
                    env.wind_speed = max(0.0, env.wind_speed - 1.0 * KT_TO_MS)
                elif ev.key == pygame.K_RIGHTBRACKET:
                    env.wind_speed += 1.0 * KT_TO_MS
                elif ev.key == pygame.K_SEMICOLON:
                    env.wind_dir -= np.deg2rad(10)
                elif ev.key == pygame.K_QUOTE:
                    env.wind_dir += np.deg2rad(10)
                elif ev.key == pygame.K_COMMA:
                    env.current_speed = max(0.0,
                                            env.current_speed - 0.2 * KT_TO_MS)
                elif ev.key == pygame.K_PERIOD:
                    env.current_speed += 0.2 * KT_TO_MS
            elif ev.type == pygame.KEYUP:
                keys_held.discard(ev.key)
            elif ev.type == pygame.MOUSEWHEEL:
                factor = 1.1 if ev.y > 0 else 1/1.1
                cam.zoom(factor)
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                dragging = True; drag_start = ev.pos
                cam.follow = False
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                dragging = False
            elif ev.type == pygame.MOUSEMOTION and dragging:
                dx = ev.pos[0] - drag_start[0]
                dy = ev.pos[1] - drag_start[1]
                cam.center[0] -= dx / cam.ppm
                cam.center[1] += dy / cam.ppm
                drag_start = ev.pos

        # Continuous key handling (held keys)
        if pygame.K_w in keys_held:
            change_throttle(frame_dt, +1)
        if pygame.K_s in keys_held:
            change_throttle(frame_dt, -1)

        helm_active = False
        if pygame.K_a in keys_held:
            set_helm_rate(frame_dt, -1); helm_active = True
        if pygame.K_d in keys_held:
            set_helm_rate(frame_dt, +1); helm_active = True
        if not helm_active and not helm_hold:
            return_helm_to_center(frame_dt)

        # --- physics: multiple fixed-dt steps per frame ---
        steps = max(1, int(round(frame_dt / DT)))
        for _ in range(steps):
            boat.step(DT)

        # --- wake ---
        if boat.speed > 0.2:
            wake.add(boat.x, boat.y, boat.t)

        # --- camera follow ---
        if cam.follow:
            cam.center[0] += (boat.x - cam.center[0]) * 0.08
            cam.center[1] += (boat.y - cam.center[1]) * 0.08

        # --- render ---
        screen.fill(BG_COLOR)

        # grid lines
        x0, y0 = cam.screen_to_world(0, WINDOW_H)
        x1, y1 = cam.screen_to_world(WINDOW_W, 0)
        gx = math.floor(x0 / GRID_M) * GRID_M
        while gx < x1:
            sx, _ = cam.world_to_screen(gx, 0)
            pygame.draw.line(screen, (30, 65, 95), (sx, 0), (sx, WINDOW_H), 1)
            gx += GRID_M
        gy = math.floor(y0 / GRID_M) * GRID_M
        while gy < y1:
            _, sy = cam.world_to_screen(0, gy)
            pygame.draw.line(screen, (30, 65, 95), (0, sy), (WINDOW_W, sy), 1)
            gy += GRID_M

        wake.draw(screen, cam, boat.t)
        scene.draw(screen, cam)
        draw_boat(screen, cam, boat)
        draw_velocity_vectors(screen, cam, boat)
        hud.draw(screen, boat, env, show_help, helm_hold)

        # Press-H-for-help hint when help is hidden
        if not show_help:
            t = hud.font_small.render("press H for help",
                                      True, HUD_DIM)
            screen.blit(t, (WINDOW_W - t.get_width() - 14,
                            WINDOW_H - 22))

        pygame.display.flip()

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
