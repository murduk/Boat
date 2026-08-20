# 17ft Deck Boat — Docking Simulator

A physics-accurate 2D top-down simulator for a 17ft deck boat with a single
outboard, focused on the low-speed / docking regime. Python (pygame) and a
mobile-friendly web version share the same physics model.

## Installation

```bash
pip install numpy pygame matplotlib
```

(matplotlib is only needed for the scripted demos — not the interactive sim.)

## Running

### Web (mobile-friendly)

Needs a local static server (ES modules won’t load from `file://`):

```bash
python -m http.server 8000 --directory web
```

Then open `http://localhost:8000` on your computer, or from a phone on the
same Wi‑Fi use `http://<your-lan-ip>:8000` (find the IP with `ipconfig` /
`ifconfig`).

Touch: throttle and helm sliders, FWD / N / REV, Reset / Hold / Wind / Help.
On phones, helm defaults to **Hold** (rudder stays). Pinch to zoom, drag to
pan; tap **Follow** to re-center on the boat.

Desktop keyboard in the web app matches the pygame controls below (plus **T**
for hold/spring).

### Python (desktop)

```bash
python interactive_sim.py     # the interactive simulator
python sanity_tests.py        # validate physics against realistic targets
python docking_demo.py        # scripted demo, generates trajectory plot
python docking_sequence.py    # scripted demo, generates filmstrip
```

## Controls (interactive_sim.py)

| Key        | Action                               |
|------------|--------------------------------------|
| W / S      | Throttle up / down (RPM lag ~0.8s)   |
| A / D      | Helm left / right                    |
| T          | Toggle helm hold / spring-to-center  |
| Q          | Shift to reverse (0.5s shift delay)  |
| E          | Shift to forward                     |
| Space      | Shift to neutral                     |
| R          | Reset position                       |
| [ / ]      | Wind speed ↓/↑                       |
| ; / '      | Wind direction rotate                |
| , / .      | Current speed ↓/↑                    |
| H          | Toggle help overlay                  |
| Mouse wheel| Zoom                                 |
| Mouse drag | Pan camera (disables auto-follow)    |
| Esc        | Quit                                 |

## Files

| File                  | Purpose                                        |
|-----------------------|------------------------------------------------|
| `boat.py`             | Physics model (forces, EoM, RK4 integrator)    |
| `interactive_sim.py`  | Pygame interactive top-down simulator          |
| `web/`                | Mobile-friendly Canvas web simulator           |
| `web/js/boat.js`      | JS port of `boat.py`                           |
| `sanity_tests.py`     | Validates 5 reference maneuvers                |
| `docking_demo.py`     | Scripted trajectory with matplotlib output     |
| `docking_sequence.py` | Scripted filmstrip snapshot                    |

## Parameters

All physical parameters are in the `BoatParams` dataclass in `boat.py`
(and mirrored in `web/js/boat.js`): dimensions, mass, added masses, hull
damping, outboard thrust curves, prop walk, windage coefficients. Tweak
there to tune feel.

## Known limitations

- **No planing regime.** Model is valid up to ~6 m/s; above that a real
  boat would transition to planing which is not modeled.
- **No dock collision.** The boat will pass through the dock geometry.
  Fender/contact model is a suggested next step.
- **Hull damping coefficients are empirically tuned**, not measured. They
  match the five sanity-test targets but individual split between sway
  and yaw damping is not independently validated.
- **Prop walk is simplified.** Uses fixed fractional lateral force; a
  real outboard has RPM-dependent walk with subtle direction changes.

## Tuning tips

1. If the boat feels too sluggish, reduce `d_u` and `d_v`.
2. If it pivots too tight with full helm, increase `d_r`.
3. If wind doesn't push it enough, check `A_lateral` and `Cy_wind`.
4. Shift delay and throttle lag are in `BoatParams` at `shift_delay`
   and `throttle_tau`.


