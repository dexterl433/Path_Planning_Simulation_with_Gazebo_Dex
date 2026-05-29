#!/usr/bin/env python3
"""
SEN771 AGVC - Path Planning Node
Student: Dexter Leong | s223026243

ROS 2 Jazzy node:
  - Builds occupancy grid for a 120x90m soccer field with 15 obstacles
  - Solves TSP (brute-force 4!) to find optimal target visit order
  - Runs A*, BFS, RRT path planning for each algorithm
  - Drives the robot via /sen771_robot/cmd_vel using a proportional controller
  - Publishes occupancy grid + planned paths to RViz2
  - Saves planned-vs-actual comparison plots to ~/ros2_ws/
"""

import math
import random
import itertools
import threading
import time
import os
import json
import tracemalloc
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mp

# ── Field & grid parameters ──────────────────────────────────────────────────
FIELD_X = 120
FIELD_Y  = 90
MESH     = 1
NX       = FIELD_X // MESH
NY       = FIELD_Y // MESH
OBS_W    = 5

# Obstacles loaded from /tmp/sen771_obstacles.json (written by generate_world.py).
# Falls back to the fixed MATLAB-spec layout if the JSON is not found.
_OBS_JSON = '/tmp/sen771_obstacles.json'
_OBS_FALLBACK = [
    ('O1',   30, 70,  5, 10),
    ('O2',   30, 45,  5, 10),
    ('O3',   30, 20,  5, 10),
    ('O5',   45, 30,  5, 10),
    ('O6',   60, 80, 10,  5),
    ('O7',   60, 65, 10,  5),
    ('O8',   60, 25, 10,  5),
    ('O9',   60, 10, 10,  5),
    ('O11',  75, 30,  5, 10),
    ('O12',  90, 70,  5, 10),
    ('O13',  90, 45,  5, 10),
    ('O14',  90, 20,  5, 10),
    ('O15', 109, 45,  5, 10),
]
if os.path.exists(_OBS_JSON):
    with open(_OBS_JSON) as _f:
        OBS = [(d['name'], d['x'], d['y'], d['w'], d['h'])
               for d in json.load(_f)]
    print(f'[planner] Loaded {len(OBS)} obstacles from {_OBS_JSON}')
else:
    OBS = _OBS_FALLBACK
    print(f'[planner] No generated obstacles found — using fixed layout ({len(OBS)} obstacles)')

_TGT_JSON = '/tmp/sen771_targets.json'
_TGT_FALLBACK = [
    (116.0, 54.5),
    (82.5,  15.5),
    (82.5,  75.5),
    (60.5,  54.5),
]
if os.path.exists(_TGT_JSON):
    with open(_TGT_JSON) as _f:
        TARGET_POS = [(d['x'], d['y']) for d in json.load(_f)]
    print(f'[planner] Loaded {len(TARGET_POS)} targets from {_TGT_JSON}')
else:
    TARGET_POS = _TGT_FALLBACK
    print(f'[planner] No target file found — using fixed layout ({len(TARGET_POS)} targets)')

ROBOT_START_M = (3.0, 45.0)
CMD_TOPIC     = '/sen771_robot/cmd_vel'
ODOM_TOPIC    = '/sen771_robot/odometry'
GT_POSE_TOPIC = '/model/sen771_robot/pose'   # Gazebo ground truth — no drift

# Controller constants
KP_LINEAR  = 1.2   # forward speed ramp: v = KP_LINEAR × dist (capped in drive loop)
KP_ANGULAR = 3.0   # unused by Pure Pursuit; kept for compatibility
MAX_SPD    = 3.5   # unused by Pure Pursuit; kept for compatibility

# Grid / field constants
GRID_BOUNDARY = 5    # cells — inflate field boundary in occupancy grid
SENSE_RANGE   = 8    # cells — virtual fan-beam obstacle sensor radius


class SEN771PlannerNode(Node):

    def __init__(self):
        super().__init__('sen771_planner')

        # ── Robot pose (written by pose callbacks, read by drive thread) ──
        self._tsp_order     = None     # set after TSP; used in plots
        self._lock          = threading.Lock()
        self._rx            = ROBOT_START_M[0]
        self._ry            = ROBOT_START_M[1]
        self._ryaw          = 0.0
        self._odom_received = False   # Gazebo-alive flag (set by odometry)
        self._gt_received   = False   # ground truth received flag

        # ── Latched QoS for one-shot map/path publishes ──
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ── Publishers ──
        self._cmd_pub   = self.create_publisher(Twist,         CMD_TOPIC,                   10)
        self._grid_pub  = self.create_publisher(OccupancyGrid, '/sen771/occupancy_grid',    latched)
        self._astar_pub     = self.create_publisher(Path,        '/sen771/astar_path',        latched)
        self._bfs_pub       = self.create_publisher(Path,        '/sen771/bfs_path',          latched)
        self._rrt_pub       = self.create_publisher(Path,        '/sen771/rrt_path',          latched)
        self._thetastar_pub = self.create_publisher(Path,        '/sen771/thetastar_path',    latched)
        self._pose_pub      = self.create_publisher(PoseStamped, '/sen771/robot_pose',        10)
        self._path_pubs = {
            'A*':     self._astar_pub,
            'BFS':    self._bfs_pub,
            'RRT':    self._rrt_pub,
            'Theta*': self._thetastar_pub,
        }

        # ── Subscribers ──
        self._odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self._pose_gt_sub = self.create_subscription(
            Pose, GT_POSE_TOPIC, self._gz_pose_cb, 10)

        self.get_logger().info('SEN771 Planner Node initialised.')
        self.get_logger().info(f'  Waiting for Gazebo on {ODOM_TOPIC}...')

        # ── Background thread runs the full planning + driving sequence ──
        self._run_thread = threading.Thread(target=self._run, daemon=True)
        self._run_thread.start()

    # ─────────────────────────────────────────────────────────────────────────
    # Odometry callback — only used to detect that Gazebo is alive.
    # Position is NOT read from here (wheel odometry drifts when hitting walls).
    # ─────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom_received = True

    # ─────────────────────────────────────────────────────────────────────────
    # Ground truth pose callback — Gazebo world-frame pose, no wheel drift.
    # ─────────────────────────────────────────────────────────────────────────
    def _gz_pose_cb(self, msg: Pose):
        q   = msg.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        with self._lock:
            self._rx          = msg.position.x
            self._ry          = msg.position.y
            self._ryaw        = yaw
            self._gt_received = True

    def _get_pose(self):
        with self._lock:
            return self._rx, self._ry, self._ryaw

    # ─────────────────────────────────────────────────────────────────────────
    # Main sequence  (runs in background thread so rclpy.spin() is never blocked)
    # ─────────────────────────────────────────────────────────────────────────
    def _run(self):
        # 1. Wait for Gazebo (odometry = Gazebo alive, ground truth = pose ready)
        while rclpy.ok() and not self._odom_received:
            time.sleep(0.1)
        self.get_logger().info('Gazebo alive — waiting for ground truth pose...')
        deadline = time.time() + 5.0
        while rclpy.ok() and not self._gt_received and time.time() < deadline:
            time.sleep(0.1)
        if self._gt_received:
            self.get_logger().info(f'  Ground truth pose active on {GT_POSE_TOPIC}')
        else:
            self.get_logger().warn(
                f'  Ground truth pose not received on {GT_POSE_TOPIC} — '
                'check bridge. Falling back to start position.')
        self.get_logger().info('Starting path planning.')
        time.sleep(0.5)

        # 1b. Re-read map JSONs now — Gazebo being alive guarantees generate_world.py
        #     has finished writing the files. The module-level read at import time can
        #     race against generate_world.py when launch starts both in parallel.
        global OBS, TARGET_POS
        if os.path.exists(_OBS_JSON):
            with open(_OBS_JSON) as _jf:
                OBS = [(d['name'], d['x'], d['y'], d['w'], d['h'])
                       for d in json.load(_jf)]
            self.get_logger().info(f'[map] Reloaded {len(OBS)} obstacles from {_OBS_JSON}')
        else:
            self.get_logger().warn('[map] No obstacle JSON found — using fallback layout')

        if os.path.exists(_TGT_JSON):
            with open(_TGT_JSON) as _jf:
                TARGET_POS = [(d['x'], d['y']) for d in json.load(_jf)]
            self.get_logger().info(f'[map] Reloaded {len(TARGET_POS)} targets from {_TGT_JSON}')
            for i, (tx, ty) in enumerate(TARGET_POS):
                self.get_logger().info(f'       T{i+1}: ({tx:.1f}, {ty:.1f})')
        else:
            self.get_logger().warn('[map] No target JSON found — using fallback positions')

        # 2. Build occupancy grid
        self.get_logger().info('[1/5] Building 120x90 occupancy grid...')
        grid, inflated = self._build_grid()
        occ  = sum(grid[c][r]     for c in range(NX) for r in range(NY))
        infl = sum(inflated[c][r] for c in range(NX) for r in range(NY))
        self.get_logger().info(f'      Occupied: {occ}  Inflated: {infl} / {NX*NY}')
        self._publish_grid(inflated)

        # 3. TSP optimisation
        self.get_logger().info('[2/5] Running TSP optimisation...')
        target_cells    = [self._m2cell(tx, ty) for tx, ty in TARGET_POS]
        start_cell      = self._m2cell(*ROBOT_START_M)
        best_order         = self._tsp(start_cell, target_cells)
        self._tsp_order    = best_order   # stored for plot labelling
        ordered_targets    = [target_cells[i]  for i in best_order]
        ordered_target_pos = [TARGET_POS[i]    for i in best_order]  # actual metres
        order_str          = ' → '.join(f'T{i+1}' for i in best_order)
        self.get_logger().info(f'      Best TSP order: Start → {order_str} → Start')

        # 4. Run all four algorithms
        self.get_logger().info('[3/5] Running path-finding algorithms...')
        results = {}
        for name, algo in [('A*', self._astar), ('BFS', self._bfs),
                           ('RRT', self._rrt), ('Theta*', self._theta_star)]:
            tracemalloc.start()
            cpu0    = time.process_time()
            t0      = time.time()
            path, nodes, anchors = self._plan_full(algo, start_cell, ordered_targets, inflated)
            elapsed  = time.time() - t0
            cpu_time = time.process_time() - cpu0
            _, peak_mem = tracemalloc.get_traced_memory()   # bytes
            tracemalloc.stop()
            results[name] = {
                'path': path, 'nodes': nodes,
                'time': elapsed, 'cpu_time': cpu_time,
                'peak_mem_kb': peak_mem / 1024,
                'anchors': anchors,
            }
            self.get_logger().info(
                f'      {name:6s}  cells={len(path):5d}  nodes={nodes:7d} '
                f' wall={elapsed:.3f}s  cpu={cpu_time:.3f}s '
                f' mem={peak_mem/1024:.1f} KB')
            self._publish_path(name, path)

        # 5. Save planned-paths plot
        self.get_logger().info('[4/5] Saving planned-paths plot...')
        self._save_planned_plot(grid, inflated, results, target_cells, start_cell)

        # 6. Drive 4 variants: A* (smoothed), BFS (smoothed),
        #    Theta* (smoothed), Theta* (raw).  Lets us see whether the
        #    post-process optimization helps even an any-angle path.
        self.get_logger().info('[5/5] Driving paths in Gazebo...')
        run_specs = [
            ('A* (smoothed)',     'A*',     True),
            ('BFS (smoothed)',    'BFS',    True),
            ('Theta* (smoothed)', 'Theta*', True),
            ('Theta* (raw)',      'Theta*', False),
        ]
        actual_paths     = {}
        simplified_paths = {}
        for label, algo, smooth in run_specs:
            self.get_logger().info(f'  ── {label} ──')
            self._stop_robot()
            self._reset_robot_pose(start_cell)
            actual, simple_m = self._drive_path(
                results[algo]['path'], inflated,
                results[algo]['anchors'], smooth=smooth,
                ordered_target_pos=ordered_target_pos)
            actual_paths[label]     = actual
            simplified_paths[label] = simple_m

        # 7. Save all result plots
        self._save_comparison_plot(grid, results, actual_paths, start_cell, run_specs)
        self._save_simplification_plot(grid, results, simplified_paths, run_specs)
        self._save_efficiency_plot(results, actual_paths, simplified_paths, run_specs)
        self._save_results_table(results, actual_paths, simplified_paths, run_specs)
        self.get_logger().info('All done! Plots saved to ~/ros2_ws/')

    # ─────────────────────────────────────────────────────────────────────────
    # Grid construction
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _m2cell(mx, my):
        return int(mx / MESH), int(my / MESH)

    @staticmethod
    def _cell2m(ci, ri):
        return ci * MESH + MESH / 2.0, ri * MESH + MESH / 2.0

    def _build_grid(self):
        grid = [[0] * NY for _ in range(NX)]
        for _, ox, oy, sx, sy in OBS:
            xmin, xmax = ox - sx / 2, ox + sx / 2
            ymin, ymax = oy - sy / 2, oy + sy / 2
            for ci in range(max(0, int(xmin / MESH)), min(NX - 1, int(xmax / MESH)) + 1):
                for ri in range(max(0, int(ymin / MESH)), min(NY - 1, int(ymax / MESH)) + 1):
                    cx, cy = self._cell2m(ci, ri)
                    if xmin <= cx <= xmax and ymin <= cy <= ymax:
                        grid[ci][ri] = 1
        inflated = [row[:] for row in grid]
        for ci in range(NX):
            for ri in range(NY):
                if grid[ci][ri] == 1:
                    for dci in range(-3, 4):
                        for dri in range(-3, 4):
                            nci, nri = ci + dci, ri + dri
                            if 0 <= nci < NX and 0 <= nri < NY:
                                inflated[nci][nri] = 1
        # Inflate field boundaries so paths stay away from walls
        for ci in range(NX):
            for ri in range(NY):
                if (ci < GRID_BOUNDARY or ci >= NX - GRID_BOUNDARY or
                        ri < GRID_BOUNDARY or ri >= NY - GRID_BOUNDARY):
                    inflated[ci][ri] = 1
        return grid, inflated

    def _nearest_free(self, cell, inflated):
        if not inflated[cell[0]][cell[1]]:
            return cell
        q, seen = deque([cell]), {cell}
        while q:
            c = q.popleft()
            for dc, dr in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nb = (c[0] + dc, c[1] + dr)
                if nb in seen or not (0 <= nb[0] < NX and 0 <= nb[1] < NY):
                    continue
                if not inflated[nb[0]][nb[1]]:
                    return nb
                seen.add(nb)
                q.append(nb)
        return cell

    # ─────────────────────────────────────────────────────────────────────────
    # TSP  (brute-force 4! = 24 permutations)
    # ─────────────────────────────────────────────────────────────────────────
    def _tsp(self, start, targets):
        best_d, best_o = math.inf, list(range(len(targets)))
        for perm in itertools.permutations(range(len(targets))):
            sc, d = start, 0.0
            for ti in perm:
                tc = targets[ti]
                d += math.hypot(tc[0] - sc[0], tc[1] - sc[1])
                sc = tc
            d += math.hypot(start[0] - sc[0], start[1] - sc[1])
            if d < best_d:
                best_d, best_o = d, list(perm)
        return best_o

    # ─────────────────────────────────────────────────────────────────────────
    # A*
    # ─────────────────────────────────────────────────────────────────────────
    def _astar(self, start, goal, inflated):
        if inflated[goal[0]][goal[1]]:
            return None, 0
        open_set = {start}
        g = {start: 0}
        f = {start: math.hypot(goal[0] - start[0], goal[1] - start[1])}
        came = {}
        nodes = 0
        while open_set:
            cur = min(open_set, key=lambda n: f.get(n, math.inf))
            if cur == goal:
                path = []
                while cur in came:
                    path.append(cur)
                    cur = came[cur]
                return list(reversed(path + [start])), nodes
            open_set.remove(cur)
            nodes += 1
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cur[0] + dc, cur[1] + dr)
                if not (0 <= nb[0] < NX and 0 <= nb[1] < NY):
                    continue
                if inflated[nb[0]][nb[1]]:
                    continue
                tg = g[cur] + 1
                if tg < g.get(nb, math.inf):
                    came[nb] = cur
                    g[nb]    = tg
                    f[nb]    = tg + math.hypot(goal[0] - nb[0], goal[1] - nb[1])
                    open_set.add(nb)
        return None, nodes

    # ─────────────────────────────────────────────────────────────────────────
    # BFS
    # ─────────────────────────────────────────────────────────────────────────
    def _bfs(self, start, goal, inflated):
        if inflated[goal[0]][goal[1]]:
            return None, 0
        queue   = deque([start])
        visited = {start}
        came    = {}
        nodes   = 0
        while queue:
            cur = queue.popleft()
            nodes += 1
            if cur == goal:
                path = []
                while cur in came:
                    path.append(cur)
                    cur = came[cur]
                return list(reversed(path + [start])), nodes
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cur[0] + dc, cur[1] + dr)
                if not (0 <= nb[0] < NX and 0 <= nb[1] < NY):
                    continue
                if inflated[nb[0]][nb[1]] or nb in visited:
                    continue
                visited.add(nb)
                came[nb] = cur
                queue.append(nb)
        return None, nodes

    # ─────────────────────────────────────────────────────────────────────────
    # RRT
    # ─────────────────────────────────────────────────────────────────────────
    def _rrt(self, start, goal, inflated, max_iter=8000, step=4, goal_bias=0.15):
        if inflated[goal[0]][goal[1]]:
            return None, 0
        nodes_list = [start]
        parent     = [-1]
        nodes_exp  = 0
        for _ in range(max_iter):
            sample = goal if random.random() < goal_bias else (
                random.randint(0, NX - 1), random.randint(0, NY - 1))
            dists  = [math.hypot(n[0] - sample[0], n[1] - sample[1]) for n in nodes_list]
            near_i = dists.index(min(dists))
            near   = nodes_list[near_i]
            d      = math.hypot(sample[0] - near[0], sample[1] - near[1])
            if d == 0:
                continue
            new = (
                max(0, min(NX - 1, round(near[0] + (sample[0] - near[0]) / d * min(step, d)))),
                max(0, min(NY - 1, round(near[1] + (sample[1] - near[1]) / d * min(step, d)))),
            )
            if inflated[new[0]][new[1]] or not self._line_free(near, new, inflated):
                continue
            nodes_list.append(new)
            parent.append(near_i)
            nodes_exp += 1
            if (math.hypot(new[0] - goal[0], new[1] - goal[1]) <= step
                    and self._line_free(new, goal, inflated)):
                nodes_list.append(goal)
                parent.append(len(nodes_list) - 2)
                path = []
                idx  = len(nodes_list) - 1
                while idx >= 0:
                    path.append(nodes_list[idx])
                    idx = parent[idx]
                return list(reversed(path)), nodes_exp
        return None, nodes_exp

    # ─────────────────────────────────────────────────────────────────────────
    # Theta*  (Daniel et al. 2010 — any-angle path planning)
    # ─────────────────────────────────────────────────────────────────────────
    def _theta_star(self, start, goal, inflated):
        """A* with built-in line-of-sight: produces smooth diagonal paths.

        Core modification vs A*: when relaxing a neighbour, first check if
        the current node's PARENT has line-of-sight to that neighbour.
        If yes, skip the current node and connect directly (any angle,
        Euclidean cost). Only fall back to standard A* if LOS fails.

        Ported from MATLAB Theta* in SEN771_Project_v7.m (Algorithm 7).
        """
        if inflated[goal[0]][goal[1]]:
            return None, 0

        open_set = {start}
        g        = {start: 0.0}
        f        = {start: math.hypot(goal[0] - start[0], goal[1] - start[1])}
        parent   = {start: start}   # start is its own parent
        nodes    = 0

        while open_set:
            cur = min(open_set, key=lambda n: f.get(n, math.inf))

            if cur == goal:
                path, node = [goal], goal
                while node != start:
                    node = parent[node]
                    path.append(node)
                return list(reversed(path)), nodes

            open_set.remove(cur)
            nodes += 1
            par = parent[cur]

            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cur[0] + dc, cur[1] + dr)
                if not (0 <= nb[0] < NX and 0 <= nb[1] < NY):
                    continue
                if inflated[nb[0]][nb[1]]:
                    continue

                # ── Theta* core: try grandparent shortcut first ──────────
                if self._line_free(par, nb, inflated):
                    tg = g.get(par, math.inf) + math.hypot(
                        nb[0] - par[0], nb[1] - par[1])
                    if tg < g.get(nb, math.inf):
                        parent[nb] = par   # any-angle: skip current node
                        g[nb] = tg
                        f[nb] = tg + math.hypot(goal[0] - nb[0], goal[1] - nb[1])
                        open_set.add(nb)
                else:
                    # Standard A* fallback (no line-of-sight through grandparent)
                    tg = g[cur] + 1.0
                    if tg < g.get(nb, math.inf):
                        parent[nb] = cur
                        g[nb] = tg
                        f[nb] = tg + math.hypot(goal[0] - nb[0], goal[1] - nb[1])
                        open_set.add(nb)

        return None, nodes

    # ─────────────────────────────────────────────────────────────────────────
    # Path utilities
    # ─────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _line_free(a, b, inflated):
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        if n == 0:
            return True
        for k in range(n + 1):
            ci = max(0, min(NX - 1, round(a[0] + (b[0] - a[0]) * k / n)))
            ri = max(0, min(NY - 1, round(a[1] + (b[1] - a[1]) * k / n)))
            if inflated[ci][ri]:
                return False
        return True

    def _plan_full(self, algo, start, targets, inflated):
        full, total_nodes = [], 0
        raw_wps   = [start] + list(targets) + [start]
        waypoints = [self._nearest_free(w, inflated) for w in raw_wps]
        for i in range(len(waypoints) - 1):
            seg, n = algo(waypoints[i], waypoints[i + 1], inflated)
            total_nodes += n
            if seg:
                full += seg[:-1]
            else:
                self.get_logger().warn(f'No path found for segment {i + 1}')
        # Append the final goal cell unless it's literally the last element.
        # Old "not in full" check was buggy for round-trip paths because the
        # start cell already exists at position 0, so the closing cell never
        # got appended and sparse algorithms (Theta*) ended ~20 m short.
        if not full or full[-1] != waypoints[-1]:
            full.append(waypoints[-1])
        return full, total_nodes, waypoints

    def _resample_polyline(self, waypoints_cells, step=1.0):
        """Resample a sparse waypoint list into dense step-m spaced (x, y)
        samples in world meters.  Pure Pursuit needs continuous samples along
        the polyline so its lookahead never skips between waypoints.
        """
        pts = []
        for i, c in enumerate(waypoints_cells):
            x, y = self._cell2m(*c)
            if i == 0:
                pts.append((x, y))
                continue
            x0, y0 = pts[-1]
            d = math.hypot(x - x0, y - y0)
            n = max(1, int(round(d / step)))
            for k in range(1, n + 1):
                t = k / n
                pts.append((x0 + t * (x - x0), y0 + t * (y - y0)))
        return pts

    def _simplify(self, path, inflated, anchors=None):
        """Greedy line-of-sight shortcut: collapse a dense grid path into the
        minimum set of waypoints whose connecting straight lines stay collision
        free.  Anchors (start + targets + start) are mandatory keep points so
        the loop is never short-cut past a target.
        """
        if len(path) < 2:
            return path
        anchor_set  = set(anchors) if anchors else set()
        anchor_idxs = [k for k, c in enumerate(path) if c in anchor_set]
        result = [path[0]]
        i = 0
        while i < len(path) - 1:
            next_anchor = next((k for k in anchor_idxs if k > i), len(path) - 1)
            # 40-cell lookahead — aggressive simplification gives ~10 waypoints
            j = min(len(path) - 1, i + 40, next_anchor)
            while j > i + 1 and not self._line_free(path[i], path[j], inflated):
                j -= 1
            result.append(path[j])
            i = j
        return result

    # ─────────────────────────────────────────────────────────────────────────
    # Robot control
    # ─────────────────────────────────────────────────────────────────────────
    def _publish_cmd(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _publish_ros_pose(self, x: float, y: float, yaw: float):
        ps = PoseStamped()
        ps.header.frame_id  = 'map'
        ps.header.stamp     = self.get_clock().now().to_msg()
        ps.pose.position.x  = x
        ps.pose.position.y  = y
        ps.pose.position.z  = 0.45
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        self._pose_pub.publish(ps)

    @staticmethod
    def _wrap(angle: float) -> float:
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _stop_robot(self):
        self._publish_cmd(0.0, 0.0)
        time.sleep(1.5)

    def _reset_robot_pose(self, start_cell):
        """Teleport the robot back to the start cell via Gazebo set_pose
        service so each driving variant gets a fair, identical starting state.
        """
        import subprocess
        x, y = ROBOT_START_M   # exact spawn position, not cell-centre
        req = (f'name: "sen771_robot" '
               f'position {{x: {x} y: {y} z: 0.45}} '
               f'orientation {{w: 1.0}}')
        subprocess.run(
            ['gz', 'service', '-s', '/world/sen771_world/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, timeout=6,
        )
        time.sleep(2.0)   # let physics settle

    def _obstacle_repulsion(self, rx, ry, ryaw, inflated):
        """Fan-beam proximity sensor — 11 beams over 180° arc ahead.

        Matches the virtual sensor from the reference controller. Returns
        (speed_penalty, repulsive_wz) blended into the proportional controller
        so the robot steers away from inflated obstacle zones before hitting them.
        """
        steer   = 0.0
        fwd_hit = 0.0
        for rel_deg in range(-90, 91, 18):
            rel_rad  = math.radians(rel_deg)
            beam_dir = ryaw + rel_rad
            for d in range(1, SENSE_RANGE + 1):
                cx = int((rx + d * math.cos(beam_dir)) / MESH)
                cy = int((ry + d * math.sin(beam_dir)) / MESH)
                hit = (not (0 <= cx < NX and 0 <= cy < NY)) or inflated[cx][cy]
                if hit:
                    prox = (SENSE_RANGE - d + 1) / SENSE_RANGE
                    steer -= math.sin(rel_rad) * prox * 2.0
                    if abs(rel_deg) <= 36:
                        fwd_hit = max(fwd_hit, prox)
                    break
        steer = max(-3.0, min(3.0, steer))
        return fwd_hit * 0.6, steer

    def _drive_path(self, path, inflated, anchors, smooth=True, ordered_target_pos=None):
        """Pure Pursuit path tracking — smooth curved motion, no stop-and-turn pivots.

        Instead of aiming at discrete waypoints (which forces the robot to stop
        and spin before each leg), Pure Pursuit steers toward a look-ahead point
        that slides continuously along the planned path.  The robot always moves
        forward, curving smoothly through corners.

        Reference: Coulter, R.C. (1992), CMU Robotics Institute.
        """
        # ── Optional post-process optimization (line-of-sight shortcutting) ──
        # Both modes resample to 1 m so Pure Pursuit has continuous samples
        # along the polyline (otherwise sparse waypoints like Theta*'s 18-
        # cell output cause the lookahead to skip and the controller to spin).
        # Only the SHORTCUTTING (simplify) is gated on `smooth`.
        if smooth:
            simple   = self._simplify(path, inflated, anchors)
            pts      = self._resample_polyline(simple, step=1.0)
            simple_m = [self._cell2m(c[0], c[1]) for c in simple]
            self.get_logger().info(
                f'    Smoothed: {len(path)} → {len(simple)} waypoints '
                f'({len(pts)} samples after resampling)')
        else:
            simple_m = [self._cell2m(c[0], c[1]) for c in path]
            pts      = self._resample_polyline(path, step=1.0)
            self.get_logger().info(
                f'    Raw (no simplify): {len(path)} waypoints '
                f'({len(pts)} samples after resampling)')
        actual = []

        CRUISE_SPD = 10.0  # m/s straight-line cruise
        GOAL_D     = 6.0   # m  — declare path complete within this distance of last cell

        # Anchor points (start + targets + start).
        # Use actual world coordinates for target anchors so the robot aims at
        # the real disc positions, not the grid-snapped nearest-free-cell.
        anchor_ms = [self._cell2m(*c) for c in (anchors or [])]
        if ordered_target_pos:
            # anchors layout: [start, T_tsp0, T_tsp1, ..., start]
            for k, (tx, ty) in enumerate(ordered_target_pos):
                if k + 1 < len(anchor_ms):
                    anchor_ms[k + 1] = (tx, ty)
        visited   = set()

        # Stuck guard — uses real ground-truth position (no odometry drift)
        last_ok  = None
        STUCK_S  = 12.0   # seconds without STUCK_M motion → stuck
        STUCK_M  = 0.3    # metres

        ptr      = 0
        iter_cnt = 0
        prev_wz  = 0.0   # for low-pass smoothing

        self.get_logger().info(
            f'  Pure Pursuit  cells={len(pts)}  L=adaptive  v_max={CRUISE_SPD}m/s')

        while rclpy.ok():
            rx, ry, ryaw = self._get_pose()
            actual.append((rx, ry))

            # ── Done? ─────────────────────────────────────────────────────────
            gx, gy = pts[-1]
            if math.hypot(gx - rx, gy - ry) < GOAL_D and ptr > len(pts) * 0.9:
                break

            # ── Pre-compute next unvisited target distance (before visit detection)
            # Must happen first so aim_override can reference it even after the
            # visit loop below marks the anchor as visited.
            next_anc_k  = None
            next_anc_x  = 0.0
            next_anc_y  = 0.0
            next_anc_d  = float('inf')
            for k, (ax, ay) in enumerate(anchor_ms):
                if k not in visited:
                    next_anc_k = k
                    next_anc_x = ax
                    next_anc_y = ay
                    next_anc_d = math.hypot(ax - rx, ay - ry)
                    break

            # ── Log anchor passes — stop, wait, then align to next segment ───
            for k, (ax, ay) in enumerate(anchor_ms):
                if k not in visited and math.hypot(ax - rx, ay - ry) < 0.3:
                    visited.add(k)
                    self.get_logger().info(
                        f'    ✓ anchor {k}/{len(anchor_ms)-1}'
                        f'  target=({ax:.0f},{ay:.0f})  robot=({rx:.1f},{ry:.1f})')

                    # Only stop-and-align at real targets (not start/return anchors)
                    is_target = 0 < k < len(anchor_ms) - 1
                    if is_target:
                        # 1. Stop at the disc
                        self._publish_cmd(0.0, 0.0)
                        time.sleep(1.0)

                        # 2. Rotate robot body to face the next path segment.
                        #    Use pts[ptr+20] so the target yaw reflects the
                        #    actual outgoing direction, not just 12 steps ahead.
                        tgt_idx = min(ptr + 20, len(pts) - 1)
                        tgt_yaw = math.atan2(
                            pts[tgt_idx][1] - pts[max(0, tgt_idx - 4)][1],
                            pts[tgt_idx][0] - pts[max(0, tgt_idx - 4)][0])
                        self.get_logger().info(
                            f'    Aligning robot body to yaw={math.degrees(tgt_yaw):.1f}°')

                        # Spin until the robot body is within 5° of target.
                        # Use vx=0.15 to prevent static-friction lockup at mu2=0.6.
                        # Loop runs until heading confirmed — no forced time-out.
                        settled = 0
                        for _ in range(500):   # up to 10 s hard cap
                            _, _, cur_yaw = self._get_pose()
                            err = self._wrap(tgt_yaw - cur_yaw)
                            if abs(err) < math.radians(5):
                                settled += 1
                                if settled >= 5:   # stable for 5 consecutive reads
                                    break
                            else:
                                settled = 0
                            wz_a = max(-2.0, min(2.0, 2.0 * err))
                            # tiny forward nudge so wheels don't seize in place
                            self._publish_cmd(0.15, wz_a)
                            time.sleep(0.02)

                        # Confirm final heading before releasing
                        self._publish_cmd(0.0, 0.0)
                        time.sleep(0.4)
                        _, _, final_yaw = self._get_pose()
                        final_err = abs(self._wrap(tgt_yaw - final_yaw))
                        self.get_logger().info(
                            f'    Body aligned — final err={math.degrees(final_err):.1f}°')
                        last_ok = None  # restart stuck timer after alignment pause
                        # Skip ptr past any path points now behind the robot
                        rx_now, ry_now, ryaw_now = self._get_pose()
                        for _j in range(ptr + 1, min(ptr + 120, len(pts) - 1)):
                            ang = self._wrap(math.atan2(pts[_j][1] - ry_now, pts[_j][0] - rx_now) - ryaw_now)
                            if abs(ang) < math.radians(90):
                                ptr = _j
                                break

                    prev_wz = 0.0   # reset smoothing so first step after target is clean

            # ── Refresh next_anc after visit detection ────────────────────────
            # next_anc was pre-computed before the visit loop ran.  If the visit
            # loop just marked that anchor as visited (+ ran stop-and-align), the
            # stale next_anc_d would make aim_override fire pointing BACK at the
            # anchor the robot just left, causing an immediate U-turn.
            if next_anc_k is not None and next_anc_k in visited:
                next_anc_k = None
                next_anc_d = float('inf')
                for _k, (_ax, _ay) in enumerate(anchor_ms):
                    if _k not in visited:
                        next_anc_k = _k
                        next_anc_x = _ax
                        next_anc_y = _ay
                        next_anc_d = math.hypot(_ax - rx, _ay - ry)
                        break

            # ── Advance ptr to nearest forward cell ───────────────────────────
            while ptr < len(pts) - 1:
                d_cur  = math.hypot(pts[ptr][0]   - rx, pts[ptr][1]   - ry)
                d_next = math.hypot(pts[ptr+1][0] - rx, pts[ptr+1][1] - ry)
                if d_next <= d_cur:
                    ptr += 1
                else:
                    break

            # ── Upcoming curvature — scan 40 m ahead to catch U-turns early ────
            max_turn = 0.0
            turn_dist = 999
            for j in range(ptr, min(ptr + 40, len(pts) - 2)):
                h0 = math.atan2(pts[j+1][1] - pts[j][1],   pts[j+1][0] - pts[j][0])
                h1 = math.atan2(pts[j+2][1] - pts[j+1][1], pts[j+2][0] - pts[j+1][0])
                t = abs(self._wrap(h1 - h0))
                if t > max_turn:
                    max_turn  = t
                    turn_dist = j - ptr

            uturn = max_turn > math.radians(120)
            sharp = math.radians(60) < max_turn <= math.radians(120)
            mild  = math.radians(30) < max_turn <= math.radians(60)

            if max_turn < math.radians(20) or turn_dist > 15:
                LOOKAHEAD = 6.0;  spd_limit = CRUISE_SPD; min_spd = 4.0
            elif uturn and turn_dist > 8:
                LOOKAHEAD = 8.0;  spd_limit = 3.0;        min_spd = 2.0
            elif uturn:
                LOOKAHEAD = 6.0;  spd_limit = 2.0;        min_spd = 1.5
            elif sharp and turn_dist > 6:
                LOOKAHEAD = 8.0;  spd_limit = 4.5;        min_spd = 3.0
            elif sharp:
                LOOKAHEAD = 6.0;  spd_limit = 3.0;        min_spd = 2.0
            elif mild:
                LOOKAHEAD = 8.0;  spd_limit = 5.0;        min_spd = 3.0
            else:
                LOOKAHEAD = 6.0;  spd_limit = CRUISE_SPD; min_spd = 4.0


            # ── Steer directly at next unvisited target when close ───────────
            aim_override = False
            is_real_target = (next_anc_k is not None and
                              0 < next_anc_k < len(anchor_ms) - 1)
            if next_anc_k is not None:
                if next_anc_d < 8.0:
                    aim_override = True
                    LOOKAHEAD = max(2.0, next_anc_d)
                elif next_anc_d < 20.0:
                    LOOKAHEAD = min(LOOKAHEAD, max(3.0, next_anc_d * 0.3))

            # ── Find look-ahead point ─────────────────────────────────────────
            if aim_override:
                lx, ly = next_anc_x, next_anc_y
            else:
                lx, ly = pts[-1]
                for i in range(ptr, len(pts)):
                    if math.hypot(pts[i][0] - rx, pts[i][1] - ry) >= LOOKAHEAD:
                        lx, ly = pts[i]
                        break

            # ── Pure Pursuit curvature κ = 2·sin(α)/L ─────────────────────────
            alpha = self._wrap(math.atan2(ly - ry, lx - rx) - ryaw)
            kappa = 2.0 * math.sin(alpha) / LOOKAHEAD

            # ── Speed ─────────────────────────────────────────────────────────
            dist_goal = math.hypot(gx - rx, gy - ry)
            vx = min(spd_limit, KP_LINEAR * dist_goal)
            speed_pen, _ = self._obstacle_repulsion(rx, ry, ryaw, inflated)
            vx *= max(0.5, 1.0 - speed_pen * 0.5)
            # Slow down within 15 m of the NEXT unvisited anchor only.
            # Skip at start (ptr < 30) so the anchor proximity of T2 doesn't
            # cap vx to 4 m/s while the robot is still near spawn — that low
            # speed combined with any initial heading correction looks like spinning.
            vx = max(min_spd, vx)
            # Target approach slowdown — only for real T1-T4 anchors, not start/return.
            # Whole-car geometry: chassis 3.0×1.7 m, disc radius 2.0 m.
            # Rear corner clears the disc edge when centre is ≤0.31 m from disc centre.
            # Floor of 0.2 m/s ensures the car is nearly stopped as it crosses that line.
            # Applied AFTER min_spd floor so it always wins.
            if is_real_target and next_anc_d < 30.0:
                approach_cap = max(0.2, next_anc_d * 0.25)  # 30m→7.5, 10m→2.5, 0.8m→0.2
                vx = min(vx, approach_cap)

            # ── Angular velocity — Pure Pursuit + low-pass smoothing ────────────
            wz_raw  = max(-10.0, min(10.0, vx * kappa))
            wz      = 0.1 * prev_wz + 0.9 * wz_raw   # smooth: 10% old, 90% new
            prev_wz = wz

            self._publish_cmd(vx, wz)
            self._publish_ros_pose(rx, ry, ryaw)
            iter_cnt += 1
            if iter_cnt % 200 == 0:
                progress = min(100, int(ptr / len(pts) * 100))
                aim_tag = f'AIM({lx:.1f},{ly:.1f})'
                if aim_override:
                    aim_tag += ' [TARGET]'
                self.get_logger().info(
                    f'    pos=({rx:.1f},{ry:.1f})  {aim_tag}'
                    f'  ptr={ptr}/{len(pts)}'
                    f'  vx={vx:.2f}  wz={wz:.2f}  '
                    f'turn={math.degrees(max_turn):.0f}°@{turn_dist}m  {progress}% path')

            # ── Stuck guard ───────────────────────────────────────────────────
            if last_ok is None:
                last_ok = (rx, ry, time.time())
            elif math.hypot(rx - last_ok[0], ry - last_ok[1]) > STUCK_M:
                last_ok = (rx, ry, time.time())
            elif time.time() - last_ok[2] > STUCK_S:
                self.get_logger().warn('  Stuck — backing up to recover')
                self._publish_cmd(-2.0, 0.0)
                time.sleep(1.5)
                self._publish_cmd(0.0, 0.0)
                time.sleep(0.3)
                # Resync ptr to nearest path cell after backing up
                rx_r, ry_r, ryaw_r = self._get_pose()
                lo, hi = max(0, ptr - 30), min(len(pts), ptr + 50)
                min_d, best = float('inf'), ptr
                for j in range(lo, hi):
                    d = math.hypot(pts[j][0] - rx_r, pts[j][1] - ry_r)
                    if d < min_d:
                        min_d, best = d, j
                ptr = best
                # Rotate toward upcoming path direction, not the final goal
                tgt_idx = min(ptr + 8, len(pts) - 1)
                tgt_yaw = math.atan2(pts[tgt_idx][1] - ry_r, pts[tgt_idx][0] - rx_r)
                angle_err = self._wrap(tgt_yaw - ryaw_r)
                rot_dir = 1.0 if angle_err > 0 else -1.0
                rot_time = min(3.0, abs(angle_err) / (2.5 * 0.71))
                self._publish_cmd(0.0, rot_dir * 2.5)
                time.sleep(rot_time)
                self._publish_cmd(0.0, 0.0)
                last_ok = None

            time.sleep(0.02)   # 50 Hz control loop

        self._publish_cmd(0.0, 0.0)
        self.get_logger().info('  Path complete.')
        return actual, simple_m

    # ─────────────────────────────────────────────────────────────────────────
    # ROS 2 publishers
    # ─────────────────────────────────────────────────────────────────────────
    def _publish_grid(self, inflated):
        msg                     = OccupancyGrid()
        msg.header.frame_id     = 'map'
        msg.header.stamp        = self.get_clock().now().to_msg()
        msg.info.resolution     = float(MESH)
        msg.info.width          = NX
        msg.info.height         = NY
        msg.info.origin.orientation.w = 1.0
        data = []
        for ri in range(NY):
            for ci in range(NX):
                data.append(100 if inflated[ci][ri] else 0)
        msg.data = data
        self._grid_pub.publish(msg)

    def _publish_path(self, name: str, path):
        pub = self._path_pubs.get(name)
        if pub is None:
            return
        msg                 = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        for ci, ri in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x  = ci * MESH + MESH / 2.0
            ps.pose.position.y  = ri * MESH + MESH / 2.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Matplotlib plots
    # ─────────────────────────────────────────────────────────────────────────
    def _draw_field(self, ax, grid):
        # ── Soccer pitch markings (matches the SDF world) ────────────────
        LW = 2.5     # line width for boundary/markings — wide for visibility
        # Outer boundary rectangle
        ax.add_patch(mp.Rectangle((0, 0), 120, 90, fill=False,
                                  edgecolor='white', linewidth=LW, zorder=1))
        # Halfway line (vertical at X = 60)
        ax.plot([60, 60], [0, 90], color='white', linewidth=LW, zorder=1)
        # Centre circle (radius 9.15 m, FIFA spec)
        ax.add_patch(plt.Circle((60, 45), 9.15, fill=False,
                                edgecolor='white', linewidth=LW, zorder=1))
        ax.plot(60, 45, marker='o', markersize=3, color='white', zorder=1)
        # Left penalty box (16.5 × 40.32 from goal line)
        ax.add_patch(mp.Rectangle((0, 24.84), 16.5, 40.32, fill=False,
                                  edgecolor='white', linewidth=LW, zorder=1))
        # Left 6-yard box (5.5 × 18.32)
        ax.add_patch(mp.Rectangle((0, 35.84), 5.5, 18.32, fill=False,
                                  edgecolor='white', linewidth=LW, zorder=1))
        # Right penalty box
        ax.add_patch(mp.Rectangle((103.5, 24.84), 16.5, 40.32, fill=False,
                                  edgecolor='white', linewidth=LW, zorder=1))
        # Right 6-yard box
        ax.add_patch(mp.Rectangle((114.5, 35.84), 5.5, 18.32, fill=False,
                                  edgecolor='white', linewidth=LW, zorder=1))

        # ── Obstacles ────────────────────────────────────────────────────
        for name, ox, oy, sx, sy in OBS:
            ax.add_patch(mp.Rectangle(
                (ox - sx / 2, oy - sy / 2), sx, sy,
                color='#C62828', edgecolor='#8B0000',
                linewidth=0.8, zorder=2))
            ax.text(ox, oy, name, ha='center', va='center',
                    fontsize=7, fontweight='bold', color='white', zorder=3)

        # ── Targets ──────────────────────────────────────────────────────
        # Build reverse lookup: input index → TSP visit position (1-based)
        tgt_clrs = ['#2288FF', '#FF8800', '#AA00FF', '#FFCC00']
        visit_pos = {}   # input index → visit order (1,2,3,4)
        if self._tsp_order:
            for rank, idx in enumerate(self._tsp_order):
                visit_pos[idx] = rank + 1
        ordinal = ['1st', '2nd', '3rd', '4th']
        for i, (tc, clr) in enumerate(
                zip([self._m2cell(tx, ty) for tx, ty in TARGET_POS], tgt_clrs)):
            ax.add_patch(plt.Circle((tc[0] + 0.5, tc[1] + 0.5), 1.2,
                                    color=clr, ec='black', lw=0.7, zorder=6))
            rank = visit_pos.get(i)
            label = f'T{i+1}\n{ordinal[rank-1]}' if rank else f'T{i+1}'
            ax.text(tc[0] + 0.5, tc[1] + 0.5, label,
                    ha='center', va='center', fontsize=6, fontweight='bold',
                    color='white', zorder=7, linespacing=1.1)

    def _save_planned_plot(self, grid, inflated, results, target_cells, start_cell):
        styles = {
            'A*':     dict(color='red',        lw=2.4),
            'BFS':    dict(color='limegreen',  lw=2.4),
            'RRT':    dict(color='darkorange', lw=2.4),
            'Theta*': dict(color='cyan',       lw=2.8),
        }

        # ── Per-algorithm 2×2 plot (each algo gets its own panel) ─────────
        tsp_str = ('Start → ' +
                   ' → '.join(f'T{i+1}' for i in self._tsp_order) +
                   ' → Start') if self._tsp_order else ''
        fig, axes = plt.subplots(2, 2, figsize=(22, 18))
        fig.suptitle(f'SEN771 Planned Paths — One Panel per Algorithm\n'
                     f'TSP optimal visit order: {tsp_str}',
                     fontsize=15, fontweight='bold')
        algo_order = ['A*', 'BFS', 'RRT', 'Theta*']
        for ax, name in zip(axes.flat, algo_order):
            ax.set_xlim(-1, NX + 1)
            ax.set_ylim(-1, NY + 1)
            ax.set_aspect('equal')
            ax.set_facecolor('#E8F5E9')
            ax.set_title(
                f"{name}  ({len(results[name]['path'])} cells, "
                f"{results[name]['nodes']} nodes, {results[name]['time']:.3f}s)",
                fontsize=13, fontweight='bold')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            self._draw_field(ax, grid)
            p = results[name]['path']
            ax.plot([c[0] + 0.5 for c in p], [c[1] + 0.5 for c in p],
                    zorder=4, label='Planned path', **styles[name])
            # Mark first and last cell so the return-to-start is obvious
            ax.plot(p[0][0]  + 0.5, p[0][1]  + 0.5, 'o', ms=14, color='lime',
                    mec='black', mew=1.5, zorder=10, label='Start cell')
            ax.plot(p[-1][0] + 0.5, p[-1][1] + 0.5, '^', ms=14, color='red',
                    mec='black', mew=1.5, zorder=10, label='Final cell')
            sc = start_cell
            ax.add_patch(mp.Rectangle((sc[0], sc[1]), 1, 1,
                                      color='magenta', zorder=8))
            ax.text(sc[0] + 0.5, sc[1] + 0.5, 'S',
                    ha='center', va='center', fontsize=9, fontweight='bold',
                    color='white', zorder=9)
            ax.legend(loc='upper right', fontsize=10)
            ax.grid(True, alpha=0.12)

        plt.tight_layout()
        out_dir = os.path.expanduser('~/ros2_ws')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'paths_planned.png')
        plt.savefig(out, dpi=110, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Planned paths plot → {out}')

    def _save_comparison_plot(self, grid, results, actual_paths, start_cell, run_specs):
        labels = [s[0] for s in run_specs if actual_paths.get(s[0])]
        if not labels:
            return
        # 2×2 layout for 4 panels → bigger, easier to read than 1×4 strip
        n = len(labels)
        rows, cols = (2, 2) if n == 4 else (1, n)
        fig, axes = plt.subplots(rows, cols, figsize=(cols * 11, rows * 9), squeeze=False)
        tsp_str = ('Start → ' +
                   ' → '.join(f'T{i+1}' for i in self._tsp_order) +
                   ' → Start') if self._tsp_order else ''
        fig.suptitle(f'SEN771 — Planned (colour) vs Actual (black) — ● start  ▲ end\n'
                     f'TSP visit order: {tsp_str}',
                     fontsize=14, fontweight='bold')
        plan_clr = {'A*': 'red', 'BFS': 'limegreen',
                    'RRT': 'darkorange', 'Theta*': 'cyan'}
        spec_map = {s[0]: s for s in run_specs}

        for ax, label in zip(axes.flat, labels):
            _, algo, _ = spec_map[label]
            ax.set_xlim(-1, NX + 1)
            ax.set_ylim(-1, NY + 1)
            ax.set_aspect('equal')
            ax.set_facecolor('#E8F5E9')
            ax.set_title(label, fontsize=14, fontweight='bold')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            self._draw_field(ax, grid)

            p = results[algo]['path']
            ax.plot([c[0] + 0.5 for c in p], [c[1] + 0.5 for c in p],
                    color=plan_clr.get(algo, 'red'),
                    lw=2.5, alpha=0.55, label='Planned', zorder=3)

            act = actual_paths[label]
            if act:
                ax.plot([a[0] for a in act], [a[1] for a in act],
                        color='black', lw=1.8, alpha=0.85, label='Actual', zorder=4)
                # Start marker (green circle), end marker (red triangle) — large
                ax.plot(act[0][0],  act[0][1],  'o', ms=14, color='lime',
                        mec='black', mew=1.5, zorder=10, label='Actual start')
                ax.plot(act[-1][0], act[-1][1], '^', ms=14, color='red',
                        mec='black', mew=1.5, zorder=10, label='Actual end')
                # Distance from end to start position for the report
                d = math.hypot(act[-1][0] - act[0][0], act[-1][1] - act[0][1])
                ax.text(0.02, 0.02, f'End → start gap: {d:.2f} m',
                        transform=ax.transAxes, fontsize=10, fontweight='bold',
                        bbox=dict(boxstyle='round', fc='white', alpha=0.85),
                        zorder=11)

            sc = start_cell
            ax.add_patch(mp.Rectangle((sc[0], sc[1]), 1, 1,
                                      color='magenta', zorder=8))
            ax.text(sc[0] + 0.5, sc[1] + 0.5, 'S',
                    ha='center', va='center', fontsize=9, fontweight='bold',
                    color='white', zorder=9)
            ax.legend(fontsize=10, loc='upper right')
            ax.grid(True, alpha=0.12)

        # Hide unused subplots if any
        for ax in axes.flat[len(labels):]:
            ax.axis('off')

        plt.tight_layout()
        out_dir = os.path.expanduser('~/ros2_ws')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'paths_comparison.png')
        plt.savefig(out, dpi=110, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Comparison plot → {out}')

    # ─────────────────────────────────────────────────────────────────────────
    # Simplification plot — raw planned path vs simplified waypoints
    # ─────────────────────────────────────────────────────────────────────────
    def _save_simplification_plot(self, grid, results, simplified_paths, run_specs):
        """Shows raw planned path (all grid cells) vs simplified waypoints
        (after line-of-sight shortcutting) for each run variant."""
        labels = [s[0] for s in run_specs if simplified_paths.get(s[0])]
        if not labels:
            return
        plan_clr = {'A*': 'red', 'BFS': 'limegreen',
                    'RRT': 'darkorange', 'Theta*': 'cyan'}
        spec_map  = {s[0]: s for s in run_specs}

        fig, axes = plt.subplots(2, 2, figsize=(22, 18), squeeze=False)
        fig.suptitle('SEN771 — Path Simplification: Raw Planned vs Simplified Waypoints',
                     fontsize=15, fontweight='bold')

        for ax, label in zip(axes.flat, labels):
            _, algo, smooth = spec_map[label]
            raw  = results[algo]['path']
            simp = simplified_paths[label]

            ax.set_xlim(-1, NX + 1)
            ax.set_ylim(-1, NY + 1)
            ax.set_aspect('equal')
            ax.set_facecolor('#E8F5E9')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            self._draw_field(ax, grid)

            # Raw planned path
            ax.plot([c[0] + 0.5 for c in raw], [c[1] + 0.5 for c in raw],
                    color=plan_clr.get(algo, 'grey'), lw=1.5, alpha=0.4,
                    label=f'Raw ({len(raw)} cells)', zorder=3)

            # Simplified waypoints
            sx = [p[0] for p in simp]
            sy = [p[1] for p in simp]
            ax.plot(sx, sy, color='navy', lw=2.2, alpha=0.9,
                    label=f'Simplified ({len(simp)} waypoints)', zorder=4)
            ax.scatter(sx, sy, color='navy', s=60, zorder=5)

            reduction = (1 - len(simp) / max(len(raw), 1)) * 100
            mode = 'smoothed' if smooth else 'raw (no simplify)'
            ax.set_title(
                f'{label}  [{mode}]\n'
                f'{len(raw)} cells → {len(simp)} waypoints  '
                f'({reduction:.0f}% reduction)',
                fontsize=12, fontweight='bold')
            ax.legend(fontsize=10, loc='upper right')
            ax.grid(True, alpha=0.12)

        for ax in axes.flat[len(labels):]:
            ax.axis('off')

        plt.tight_layout()
        out_dir = os.path.expanduser('~/ros2_ws')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'paths_simplification.png')
        plt.savefig(out, dpi=110, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Simplification plot → {out}')

    # ─────────────────────────────────────────────────────────────────────────
    # Efficiency plot — runtime, nodes, path length, deviation comparison
    # ─────────────────────────────────────────────────────────────────────────
    def _save_efficiency_plot(self, results, actual_paths, simplified_paths, run_specs):
        """Bar charts comparing: planning time, nodes explored, path length,
        actual distance driven, and mean deviation from planned path."""

        def _path_len_m(pts):
            if len(pts) < 2:
                return 0.0
            return sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
                       for i in range(len(pts)-1))

        def _mean_deviation(actual, planned_cells):
            if not actual or not planned_cells:
                return 0.0
            planned_m = [(c[0]*MESH + MESH/2, c[1]*MESH + MESH/2)
                         for c in planned_cells]
            total = 0.0
            for ax_, ay_ in actual:
                d = min(math.hypot(ax_ - px, ay_ - py) for px, py in planned_m)
                total += d
            return total / len(actual)

        def _max_deviation(actual, planned_cells):
            if not actual or not planned_cells:
                return 0.0
            planned_m = [(c[0]*MESH + MESH/2, c[1]*MESH + MESH/2)
                         for c in planned_cells]
            return max(
                min(math.hypot(ax_ - px, ay_ - py) for px, py in planned_m)
                for ax_, ay_ in actual)

        algo_names  = ['A*', 'BFS', 'RRT', 'Theta*']
        label_names = [s[0] for s in run_specs]
        colors      = ['#e74c3c', '#2ecc71', '#e67e22', '#1abc9c']

        # ── Metrics per planning algorithm ───────────────────────────────────
        plan_times   = [results[a]['time']          for a in algo_names]
        plan_nodes   = [results[a]['nodes']          for a in algo_names]
        plan_cells   = [len(results[a]['path'])      for a in algo_names]
        plan_len_m   = [_path_len_m(
                            [(c[0]*MESH + MESH/2, c[1]*MESH + MESH/2)
                             for c in results[a]['path']])
                        for a in algo_names]

        # ── Metrics per run variant ───────────────────────────────────────────
        actual_len   = [_path_len_m(actual_paths.get(l, []))   for l in label_names]
        mean_dev     = [_mean_deviation(actual_paths.get(l, []),
                                        results[run_specs[i][1]]['path'])
                        for i, l in enumerate(label_names)]
        max_dev      = [_max_deviation(actual_paths.get(l, []),
                                       results[run_specs[i][1]]['path'])
                        for i, l in enumerate(label_names)]
        simp_wpts    = [len(simplified_paths.get(l, [])) for l in label_names]

        fig, axes = plt.subplots(3, 2, figsize=(18, 20))
        fig.suptitle('SEN771 — Runtime & Path Efficiency Analysis',
                     fontsize=16, fontweight='bold')
        x4  = range(len(algo_names))
        x4r = range(len(label_names))

        # 1. Planning time
        ax = axes[0, 0]
        bars = ax.bar(x4, plan_times, color=colors)
        ax.set_xticks(x4); ax.set_xticklabels(algo_names, fontsize=12)
        ax.set_title('Planning Time (seconds)', fontsize=13, fontweight='bold')
        ax.set_ylabel('Time (s)')
        for bar, v in zip(bars, plan_times):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                    f'{v:.3f}s', ha='center', va='bottom', fontsize=11, fontweight='bold')
        ax.grid(axis='y', alpha=0.3)

        # 2. Nodes explored
        ax = axes[0, 1]
        bars = ax.bar(x4, plan_nodes, color=colors)
        ax.set_xticks(x4); ax.set_xticklabels(algo_names, fontsize=12)
        ax.set_title('Nodes Explored', fontsize=13, fontweight='bold')
        ax.set_ylabel('Nodes')
        for bar, v in zip(bars, plan_nodes):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 50,
                    f'{v:,}', ha='center', va='bottom', fontsize=10, fontweight='bold')
        ax.grid(axis='y', alpha=0.3)

        # 3. Planned path length (metres)
        ax = axes[1, 0]
        bars = ax.bar(x4, plan_len_m, color=colors)
        ax.set_xticks(x4); ax.set_xticklabels(algo_names, fontsize=12)
        ax.set_title('Planned Path Length (metres)', fontsize=13, fontweight='bold')
        ax.set_ylabel('Distance (m)')
        for bar, v in zip(bars, plan_len_m):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{v:.1f} m', ha='center', va='bottom', fontsize=11, fontweight='bold')
        ax.grid(axis='y', alpha=0.3)

        # 4. Actual distance driven vs simplified waypoints
        ax = axes[1, 1]
        run_colors = ['#c0392b', '#27ae60', '#8e44ad', '#2980b9']
        bars = ax.bar(x4r, actual_len, color=run_colors)
        ax.set_xticks(x4r)
        ax.set_xticklabels(label_names, fontsize=10, rotation=10, ha='right')
        ax.set_title('Actual Distance Driven (metres)', fontsize=13, fontweight='bold')
        ax.set_ylabel('Distance (m)')
        for bar, v in zip(bars, actual_len):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{v:.1f} m', ha='center', va='bottom', fontsize=11, fontweight='bold')
        ax.grid(axis='y', alpha=0.3)

        # 5. Mean deviation: actual vs planned
        ax = axes[2, 0]
        bars = ax.bar(x4r, mean_dev, color=run_colors, label='Mean deviation')
        ax.bar(x4r, max_dev, color=run_colors, alpha=0.35, label='Max deviation')
        ax.set_xticks(x4r)
        ax.set_xticklabels(label_names, fontsize=10, rotation=10, ha='right')
        ax.set_title('Actual vs Planned Deviation (metres)', fontsize=13, fontweight='bold')
        ax.set_ylabel('Deviation (m)')
        for bar, v in zip(bars, mean_dev):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                    f'{v:.2f} m', ha='center', va='bottom', fontsize=10, fontweight='bold')
        ax.legend(fontsize=11)
        ax.grid(axis='y', alpha=0.3)

        # 6. Simplified waypoints count
        ax = axes[2, 1]
        bars = ax.bar(x4r, simp_wpts, color=run_colors)
        ax.set_xticks(x4r)
        ax.set_xticklabels(label_names, fontsize=10, rotation=10, ha='right')
        ax.set_title('Simplified Waypoints Count', fontsize=13, fontweight='bold')
        ax.set_ylabel('Waypoints')
        for bar, v in zip(bars, simp_wpts):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                    str(v), ha='center', va='bottom', fontsize=12, fontweight='bold')
        ax.grid(axis='y', alpha=0.3)

        plt.tight_layout()
        out_dir = os.path.expanduser('~/ros2_ws')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'paths_efficiency.png')
        plt.savefig(out, dpi=110, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Efficiency plot → {out}')

    # ─────────────────────────────────────────────────────────────────────────
    # Results summary table
    # ─────────────────────────────────────────────────────────────────────────
    def _save_results_table(self, results, actual_paths, simplified_paths, run_specs):
        """Saves a single image with two tables:
        1. Planning algorithm metrics (wall time, CPU time, memory, nodes, path length)
        2. Run variant metrics (actual distance, mean/max deviation, waypoints)
        """

        def _path_len_m(pts):
            if len(pts) < 2:
                return 0.0
            return sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])
                       for i in range(len(pts)-1))

        def _mean_dev(actual, planned_cells):
            if not actual or not planned_cells:
                return 0.0
            pm = [(c[0]*MESH + MESH/2, c[1]*MESH + MESH/2) for c in planned_cells]
            return sum(min(math.hypot(ax-px, ay-py) for px, py in pm)
                       for ax, ay in actual) / len(actual)

        def _max_dev(actual, planned_cells):
            if not actual or not planned_cells:
                return 0.0
            pm = [(c[0]*MESH + MESH/2, c[1]*MESH + MESH/2) for c in planned_cells]
            return max(min(math.hypot(ax-px, ay-py) for px, py in pm)
                       for ax, ay in actual)

        algo_names  = ['A*', 'BFS', 'RRT', 'Theta*']
        label_names = [s[0] for s in run_specs]

        # ── Table 1: Planning algorithm metrics ──────────────────────────────
        t1_headers = ['Algorithm', 'Wall Time (s)', 'CPU Time (s)',
                      'Peak Memory (KB)', 'Nodes Explored',
                      'Path Cells', 'Path Length (m)']
        t1_rows = []
        for a in algo_names:
            r   = results[a]
            plen = _path_len_m([(c[0]*MESH + MESH/2, c[1]*MESH + MESH/2)
                                 for c in r['path']])
            t1_rows.append([
                a,
                f"{r['time']:.3f}",
                f"{r['cpu_time']:.3f}",
                f"{r['peak_mem_kb']:.1f}",
                f"{r['nodes']:,}",
                f"{len(r['path']):,}",
                f"{plen:.1f}",
            ])

        # ── Table 2: Run variant metrics ─────────────────────────────────────
        t2_headers = ['Run Variant', 'Algo', 'Simplified\nWaypoints',
                      'Actual Driven (m)', 'Mean Deviation (m)', 'Max Deviation (m)']
        t2_rows = []
        for i, (label, algo, smooth) in enumerate(run_specs):
            act  = actual_paths.get(label, [])
            plan = results[algo]['path']
            t2_rows.append([
                label,
                algo,
                str(len(simplified_paths.get(label, []))),
                f"{_path_len_m(act):.1f}",
                f"{_mean_dev(act, plan):.2f}",
                f"{_max_dev(act, plan):.2f}",
            ])

        # ── Draw ──────────────────────────────────────────────────────────────
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 8))
        fig.suptitle('SEN771 AGVC — Full Results Summary Table',
                     fontsize=15, fontweight='bold', y=1.01)
        fig.patch.set_facecolor('#f0f0f0')

        for ax, headers, rows, title in [
            (ax1, t1_headers, t1_rows, 'Planning Algorithm Metrics'),
            (ax2, t2_headers, t2_rows, 'Run Variant Metrics'),
        ]:
            ax.axis('off')
            ax.set_title(title, fontsize=13, fontweight='bold', pad=8)
            tbl = ax.table(
                cellText=rows,
                colLabels=headers,
                cellLoc='center',
                loc='center',
            )
            tbl.auto_set_font_size(False)
            tbl.set_fontsize(11)
            tbl.scale(1, 2.0)

            # Header row styling
            for j in range(len(headers)):
                tbl[0, j].set_facecolor('#2c3e50')
                tbl[0, j].set_text_props(color='white', fontweight='bold')

            # Alternating row colours
            row_colors = ['#ffffff', '#eaf2fb']
            for i in range(1, len(rows) + 1):
                for j in range(len(headers)):
                    tbl[i, j].set_facecolor(row_colors[(i - 1) % 2])

        plt.tight_layout()
        out_dir = os.path.expanduser('~/ros2_ws')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'results_table.png')
        plt.savefig(out, dpi=120, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Results table → {out}')


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SEN771PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
