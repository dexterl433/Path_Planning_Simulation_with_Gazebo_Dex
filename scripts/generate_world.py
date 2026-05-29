#!/usr/bin/env python3
"""
World generator for SEN771 AGVC.
Generates 15 random obstacles, shows the field, lets user click 4 targets,
then writes:
  - /tmp/sen771_world.sdf       (Gazebo loads this)
  - /tmp/sen771_obstacles.json  (planner reads obstacle layout)
  - /tmp/sen771_targets.json    (planner reads user-selected targets)

Usage:
  python3 generate_world.py <obs_size>
  python3 generate_world.py 6      # all obstacles base 6 m
  python3 generate_world.py        # default: 6 m

Each obstacle is randomly oriented tall (base × base*2) or wide (base*2 × base).
Maximum obs_size: 10 m.
"""

import random
import math
import json
import sys
import os

# ── Field ────────────────────────────────────────────────────────────────────
FIELD_W  = 120.0
FIELD_H  = 90.0
MARGIN   = 6.0

ROBOT_START = (3.0, 45.0)
CLEAR_DIST  = 10.0
N_OBS       = 15
INFLATE_M   = 2.5   # safety margin in metres (matches planner's InflateCells*MESH+0.5)


def _overlaps_existing(x, y, w, h, placed, min_spacing):
    for ox, oy, ow, oh in placed:
        if (abs(x - ox) < (w + ow) / 2 + min_spacing and
                abs(y - oy) < (h + oh) / 2 + min_spacing):
            return True
    return False


def _too_close_to_start(x, y):
    return math.hypot(x - ROBOT_START[0], y - ROBOT_START[1]) < CLEAR_DIST


def generate_obstacles(obs_size):
    min_spacing = obs_size * 1.5 + 3.0
    placed = []
    for _ in range(N_OBS):
        for _attempt in range(2000):
            if random.random() < 0.5:
                w, h = obs_size, obs_size * 2
            else:
                w, h = obs_size * 2, obs_size

            half_w, half_h = w / 2, h / 2
            x = random.uniform(MARGIN + half_w, FIELD_W - MARGIN - half_w)
            y = random.uniform(MARGIN + half_h, FIELD_H - MARGIN - half_h)

            if _too_close_to_start(x, y):
                continue
            if _overlaps_existing(x, y, w, h, placed, min_spacing):
                continue

            placed.append((x, y, w, h))
            break
        else:
            placed.append((
                random.uniform(20, 100),
                random.uniform(5,  85),
                obs_size, obs_size * 2,
            ))

    return placed


def _point_on_obstacle(px, py, obstacles):
    """Return True if (px, py) is within the inflated boundary of any obstacle."""
    for ox, oy, ow, oh in obstacles:
        if (abs(px - ox) < ow / 2 + INFLATE_M and
                abs(py - oy) < oh / 2 + INFLATE_M):
            return True
    return False


def _point_in_field(px, py):
    return MARGIN <= px <= FIELD_W - MARGIN and MARGIN <= py <= FIELD_H - MARGIN


def pick_targets_click(obstacles):
    """Show the field with obstacles and let the user click 4 target locations."""
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mp

    fig, ax = plt.subplots(figsize=(12, 9))
    ax.set_xlim(-5, FIELD_W + 5)
    ax.set_ylim(-5, FIELD_H + 5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')

    # Field boundary
    ax.add_patch(mp.Rectangle((0, 0), FIELD_W, FIELD_H,
                               fill=False, edgecolor='blue', linewidth=2))
    # Grid
    for x in range(0, int(FIELD_W)+1, 10):
        ax.axvline(x, color=[0.9,0.9,0.9], linewidth=0.5)
    for y in range(0, int(FIELD_H)+1, 10):
        ax.axhline(y, color=[0.9,0.9,0.9], linewidth=0.5)
    # Obstacles
    for ox, oy, ow, oh in obstacles:
        ax.add_patch(mp.Rectangle((ox - ow/2, oy - oh/2), ow, oh,
                                   facecolor=[1, 0, 0, 0.7], edgecolor='red'))
    # Robot start
    ax.plot(*ROBOT_START, 'ms', markersize=12)
    ax.text(ROBOT_START[0]+2, ROBOT_START[1]+2, 'Start', fontsize=9,
            fontweight='bold', color='m')

    targets = []
    N = 4

    def on_click(event):
        if event.inaxes != ax or len(targets) >= N:
            return
        px, py = event.xdata, event.ydata
        if not _point_in_field(px, py):
            ax.set_title(f'Target {len(targets)+1}/{N}: outside field boundary — try again',
                         color='red')
            fig.canvas.draw()
            return
        if _point_on_obstacle(px, py, obstacles):
            ax.set_title(f'Target {len(targets)+1}/{N}: on obstacle — try again',
                         color='red')
            fig.canvas.draw()
            return
        # Check not too close to existing targets
        for tx, ty in targets:
            if math.hypot(px - tx, py - ty) < 8.0:
                ax.set_title(f'Target {len(targets)+1}/{N}: too close to another target',
                             color='red')
                fig.canvas.draw()
                return

        targets.append((px, py))
        t = len(targets)
        ax.plot(px, py, 'go', markersize=12, markerfacecolor='g')
        ax.text(px+2, py+2, f'T{t}', fontsize=10, fontweight='bold',
                color=[0, 0.5, 0])
        if t < N:
            ax.set_title(f'Click Target {t+1} of {N}', color=[0, 0.5, 0])
        else:
            ax.set_title('All 4 targets placed — close this window to continue',
                         color='blue')
        fig.canvas.draw()

    fig.canvas.mpl_connect('button_press_event', on_click)
    ax.set_title('Click to place Target 1 of 4 — avoid red obstacles',
                 color=[0, 0.5, 0])
    plt.tight_layout()
    plt.show()  # blocks until window is closed

    return targets


def pick_targets_input(obstacles):
    """Fallback: type x y coordinates if no display is available."""
    print('[generate_world] No display — enter target coordinates manually.')
    print(f'  Field: x 0-{FIELD_W} m, y 0-{FIELD_H} m. Avoid red obstacles.')
    targets = []
    t = 0
    while t < 4:
        raw = input(f'  Target {t+1} [x y]: ').strip()
        try:
            px, py = map(float, raw.split())
        except ValueError:
            print('  Enter two numbers e.g. 60 45'); continue
        if not _point_in_field(px, py):
            print('  Outside field boundary.'); continue
        if _point_on_obstacle(px, py, obstacles):
            print('  On or too close to an obstacle.'); continue
        if any(math.hypot(px-tx, py-ty) < 8.0 for tx, ty in targets):
            print('  Too close to another target.'); continue
        targets.append((px, py))
        print(f'  T{t+1}: ({px:.1f}, {py:.1f}) — confirmed')
        t += 1
    return targets


def build_sdf_block(obstacles):
    lines = [
        '    <!-- ═══════════════════════════════════════════════',
        '         15 OBSTACLES  (red boxes, height 2 m — randomly generated)',
        '    ═══════════════════════════════════════════════ -->',
        '',
    ]
    for i, (x, y, w, h) in enumerate(obstacles):
        name = f'obs_{i+1:02d}'
        lines.append(
            f'    <model name="{name}"><static>true</static>'
            f'<pose>{x:.2f} {y:.2f} 1 0 0 0</pose>\n'
            f'      <link name="link">'
            f'<collision name="c"><geometry><box><size>{w:.2f} {h:.2f} 2</size></box></geometry></collision>\n'
            f'      <visual name="v"><geometry><box><size>{w:.2f} {h:.2f} 2</size></box></geometry>\n'
            f'        <material><ambient>0.9 0.05 0.05 1</ambient>'
            f'<diffuse>1 0.05 0.05 1</diffuse></material></visual></link></model>\n'
        )
    return '\n'.join(lines)


def main():
    DEFAULT_SIZE = 6.0

    if len(sys.argv) >= 2:
        try:
            obs_size = float(sys.argv[1])
        except ValueError:
            print('[generate_world] Invalid size — using default 6 m.')
            obs_size = DEFAULT_SIZE
    else:
        obs_size = DEFAULT_SIZE

    obs_size = max(1.0, min(10.0, obs_size))
    print(f'[generate_world] Obstacle size: {obs_size:.1f} m')

    # ── Generate obstacles ───────────────────────────────────────────────────
    obstacles = generate_obstacles(obs_size)
    print(f'[generate_world] Placed {len(obstacles)} obstacles.')

    # ── User picks targets ───────────────────────────────────────────────────
    print('[generate_world] Opening field — click 4 target locations...')
    try:
        targets = pick_targets_click(obstacles)
        if len(targets) < 4:
            print('[generate_world] Window closed before 4 targets — falling back to input.')
            targets = pick_targets_input(obstacles)
    except Exception as e:
        print(f'[generate_world] Click UI failed ({e}) — falling back to input.')
        targets = pick_targets_input(obstacles)

    print('[generate_world] Targets selected:')
    for i, (tx, ty) in enumerate(targets):
        print(f'  T{i+1}: ({tx:.1f}, {ty:.1f})')

    # ── Load template & write SDF ────────────────────────────────────────────
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('sen771_agvc')
    except Exception:
        share = os.path.join(os.path.dirname(__file__), '..', 'worlds')

    template_path = os.path.join(share, 'worlds', 'sen771_world_template.sdf')
    if not os.path.exists(template_path):
        template_path = os.path.join(os.path.dirname(__file__),
                                     '..', 'worlds', 'sen771_world_template.sdf')

    with open(template_path) as f:
        template = f.read()

    sdf_block = build_sdf_block(obstacles)
    world_sdf = template.replace('{OBSTACLES}', sdf_block)

    out_sdf = '/tmp/sen771_world.sdf'
    with open(out_sdf, 'w') as f:
        f.write(world_sdf)
    print(f'[generate_world] World written → {out_sdf}')

    obs_json = [
        {'name': f'O{i+1}', 'x': x, 'y': y, 'w': w, 'h': h}
        for i, (x, y, w, h) in enumerate(obstacles)
    ]
    with open('/tmp/sen771_obstacles.json', 'w') as f:
        json.dump(obs_json, f, indent=2)
    print(f'[generate_world] Obstacles written → /tmp/sen771_obstacles.json')

    tgt_json = [{'name': f'T{i+1}', 'x': tx, 'y': ty}
                for i, (tx, ty) in enumerate(targets)]
    with open('/tmp/sen771_targets.json', 'w') as f:
        json.dump(tgt_json, f, indent=2)
    print(f'[generate_world] Targets written → /tmp/sen771_targets.json')


if __name__ == '__main__':
    main()
