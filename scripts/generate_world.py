#!/usr/bin/env python3
"""
World generator for SEN771 AGVC.
Generates 15 random obstacles, writes:
  - /tmp/sen771_world.sdf       (Gazebo loads this)
  - /tmp/sen771_obstacles.json  (planner reads this)

Usage:
  python3 generate_world.py <avg_obstacle_size_metres>
  python3 generate_world.py          # picks random size 3-8 m
"""

import random
import math
import json
import sys
import os

# ── Field ────────────────────────────────────────────────────────────────────
FIELD_W  = 120.0   # x extent
FIELD_H  = 90.0    # y extent
MARGIN   = 6.0     # keep obstacles this far from walls

# ── Fixed positions that must stay clear ────────────────────────────────────
ROBOT_START = (3.0, 45.0)
TARGETS = [
    (116.0, 54.5),
    (82.5,  15.5),
    (82.5,  75.5),
    (60.5,  54.5),
]
CLEAR_DIST = 10.0   # min distance from robot start and each target

# ── Obstacle count ───────────────────────────────────────────────────────────
N_OBS = 15

# ── Obstacle size bounds ─────────────────────────────────────────────────────
MIN_OBS_BASE = 4.0    # minimum short side (metres)
MAX_OBS_BASE = 12.0   # maximum short side (metres)


def _overlaps_existing(x, y, w, h, placed):
    for ox, oy, ow, oh in placed:
        gap = 3.0
        if (abs(x - ox) < (w + ow) / 2 + gap and
                abs(y - oy) < (h + oh) / 2 + gap):
            return True
    return False


def _too_close_to_fixed(x, y):
    if math.hypot(x - ROBOT_START[0], y - ROBOT_START[1]) < CLEAR_DIST:
        return True
    for tx, ty in TARGETS:
        if math.hypot(x - tx, y - ty) < CLEAR_DIST:
            return True
    return False


def generate_obstacles(avg_size):
    placed = []   # list of (x, y, w, h)
    for _ in range(N_OBS):
        for _attempt in range(2000):
            # Size: avg ± 40%, clamped to [MIN_OBS_BASE, MAX_OBS_BASE]
            # One side is doubled to make fat/thin rectangles
            base = avg_size * random.uniform(0.6, 1.4)
            base = max(MIN_OBS_BASE, min(MAX_OBS_BASE, base))
            if random.random() < 0.5:
                w, h = base, base * 2        # tall/fat
            else:
                w, h = base * 2, base        # wide/thin

            half_w, half_h = w / 2, h / 2
            x = random.uniform(MARGIN + half_w, FIELD_W - MARGIN - half_w)
            y = random.uniform(MARGIN + half_h, FIELD_H - MARGIN - half_h)

            if _too_close_to_fixed(x, y):
                continue
            if _overlaps_existing(x, y, w, h, placed):
                continue

            placed.append((x, y, w, h))
            break
        else:
            # Fallback — find any safe spot (should rarely happen)
            placed.append((
                random.uniform(20, 100),
                random.uniform(5,  85),
                avg_size, avg_size * 2,
            ))

    return placed


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
    # ── Average obstacle size ────────────────────────────────────────────────
    if len(sys.argv) > 1:
        try:
            avg_size = float(sys.argv[1])
        except ValueError:
            print(f'[generate_world] Invalid size "{sys.argv[1]}", using random.')
            avg_size = random.uniform(3.0, 8.0)
    else:
        avg_size = random.uniform(3.0, 8.0)

    avg_size = max(1.0, min(15.0, avg_size))   # clamp to sane range
    print(f'[generate_world] Average obstacle size: {avg_size:.2f} m')

    # ── Generate obstacle positions ──────────────────────────────────────────
    obstacles = generate_obstacles(avg_size)
    print(f'[generate_world] Placed {len(obstacles)} obstacles.')

    # ── Load template ────────────────────────────────────────────────────────
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('sen771_agvc')
    except Exception:
        share = os.path.join(os.path.dirname(__file__), '..', 'worlds')

    template_path = os.path.join(share, 'worlds', 'sen771_world_template.sdf')
    if not os.path.exists(template_path):
        # Try relative fallback (running from sen771_final/scripts/)
        template_path = os.path.join(os.path.dirname(__file__),
                                     '..', 'worlds', 'sen771_world_template.sdf')

    with open(template_path) as f:
        template = f.read()

    # ── Fill template ────────────────────────────────────────────────────────
    sdf_block = build_sdf_block(obstacles)
    world_sdf = template.replace('{OBSTACLES}', sdf_block)

    # ── Write generated SDF ──────────────────────────────────────────────────
    out_sdf = '/tmp/sen771_world.sdf'
    with open(out_sdf, 'w') as f:
        f.write(world_sdf)
    print(f'[generate_world] World written → {out_sdf}')

    # ── Write JSON for planner ───────────────────────────────────────────────
    obs_json = [
        {'name': f'O{i+1}', 'x': x, 'y': y, 'w': w, 'h': h}
        for i, (x, y, w, h) in enumerate(obstacles)
    ]
    out_json = '/tmp/sen771_obstacles.json'
    with open(out_json, 'w') as f:
        json.dump(obs_json, f, indent=2)
    print(f'[generate_world] Obstacles written → {out_json}')


if __name__ == '__main__':
    main()
