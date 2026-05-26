#!/usr/bin/env python3
"""
SEN771 Robot Control Debug Node
================================
Runs four scripted maneuvers and measures what the robot ACTUALLY does vs what was commanded.
Compares ground-truth position (Gazebo) vs wheel odometry (drift test).

Tests
-----
  T1 – Straight east: vx=2.0 m/s, wz=0 for 5 s
  T2 – Stop accuracy: vx=2.5 m/s for 3 s then hard stop, record coast
  T3 – Clockwise arc: vx=1.5 m/s, wz=-0.8 rad/s for 8 s
  T4 – Counter-clockwise arc: vx=1.5 m/s, wz=+0.8 rad/s for 8 s

Output
------
  ~/generated/debug_control.png  — trajectory plots (GT=blue, odom=red)
  Log lines with real-time factor, distance errors, stopping distance
"""

import math, time, threading, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

CMD_TOPIC     = '/sen771_robot/cmd_vel'
ODOM_TOPIC    = '/sen771_robot/odometry'
GT_POSE_TOPIC = '/model/sen771_robot/pose'
OUT_PATH      = os.path.expanduser('~/generated/debug_control.png')


class DebugNode(Node):

    def __init__(self):
        super().__init__('sen771_debug')
        self._lock       = threading.Lock()
        self._gt  = (3.0, 45.0, 0.0)   # (x, y, yaw) — ground truth
        self._odom= (3.0, 45.0, 0.0)   # (x, y, yaw) — wheel odometry
        self._gt_ok   = False
        self._odom_ok = False

        self._cmd_pub  = self.create_publisher(Twist, CMD_TOPIC, 10)
        self._gt_sub   = self.create_subscription(Pose,     GT_POSE_TOPIC, self._gt_cb,   10)
        self._odom_sub = self.create_subscription(Odometry, ODOM_TOPIC,    self._odom_cb, 10)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _gt_cb(self, msg: Pose):
        q   = msg.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        with self._lock:
            self._gt    = (msg.position.x, msg.position.y, yaw)
            self._gt_ok = True

    def _odom_cb(self, msg: Odometry):
        q   = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        with self._lock:
            self._odom    = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self._odom_ok = True

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _send(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _pose(self):
        with self._lock:
            return self._gt, self._odom

    def _record(self, vx, wz, duration, dt=0.05):
        """Run (vx, wz) for `duration` seconds, recording GT and odom at `dt` Hz."""
        gt_log, odom_log = [], []
        t0 = time.time()
        while time.time() - t0 < duration and rclpy.ok():
            self._send(vx, wz)
            gt, odom = self._pose()
            gt_log.append(gt[:2])
            odom_log.append(odom[:2])
            time.sleep(dt)
        self._send(0.0, 0.0)
        return gt_log, odom_log

    # ── Main test sequence ────────────────────────────────────────────────────
    def _run(self):
        log = self.get_logger()
        log.info('Waiting for Gazebo ground truth ...')
        while rclpy.ok() and not (self._gt_ok and self._odom_ok):
            time.sleep(0.1)
        time.sleep(2.0)   # let physics settle after spawn
        log.info('=== Control Debug Starting ===')

        tests = []

        # ── T1: Straight east ──────────────────────────────────────────────
        vx, wz, dur = 2.0, 0.0, 5.0
        log.info(f'T1  Straight  vx={vx}  wz={wz}  dur={dur}s')
        gt_start, odom_start = self._pose()
        t_wall = time.time()
        gt_log, odom_log = self._record(vx, wz, dur)
        elapsed = time.time() - t_wall
        gt_end, odom_end = self._pose()
        dist_gt   = math.hypot(gt_end[0]-gt_start[0],   gt_end[1]-gt_start[1])
        dist_odom = math.hypot(odom_end[0]-odom_start[0],odom_end[1]-odom_start[1])
        sim_rtf   = dist_gt / (vx * dur) if dist_gt > 0 else 0.0
        log.info(f'     GT dist={dist_gt:.2f}m  odom dist={dist_odom:.2f}m  '
                 f'commanded={vx*dur:.1f}m  RTF≈{sim_rtf:.2f}x')
        tests.append(('T1: Straight  vx=2  wz=0  5s', gt_log, odom_log,
                       vx, wz, dur, sim_rtf))
        time.sleep(1.5)

        # ── T2: Stop accuracy ──────────────────────────────────────────────
        vx2, dur2 = 2.5, 3.0
        log.info(f'T2  Stop accuracy  vx={vx2}  dur={dur2}s then hard stop')
        gt_pre, _ = self._pose()
        gt_run, _ = self._record(vx2, 0.0, dur2)
        gt_stop, _ = self._pose()   # position at cmd=0
        # Record 2 s of coast
        gt_coast, _ = self._record(0.0, 0.0, 2.0)
        gt_final, _ = self._pose()
        coast = math.hypot(gt_final[0]-gt_stop[0], gt_final[1]-gt_stop[1])
        log.info(f'     Stop pos=({gt_stop[0]:.2f},{gt_stop[1]:.2f})  '
                 f'coast after cmd=0: {coast:.3f}m  (ideal=0)')
        tests.append(('T2: Stop accuracy  vx=2.5  3s+coast', gt_run+gt_coast,
                       [], vx2, 0.0, dur2, sim_rtf))
        time.sleep(1.5)

        # ── T3: Clockwise arc ──────────────────────────────────────────────
        vx3, wz3, dur3 = 1.5, -0.8, 8.0
        r3 = abs(vx3 / wz3)
        log.info(f'T3  CW arc  vx={vx3}  wz={wz3}  dur={dur3}s  '
                 f'expected radius={r3:.2f}m')
        gt3s, _ = self._pose()
        gt3_log, odom3_log = self._record(vx3, wz3, dur3)
        gt3e, _ = self._pose()
        arc_dist = math.hypot(gt3e[0]-gt3s[0], gt3e[1]-gt3s[1])
        log.info(f'     Start=({gt3s[0]:.1f},{gt3s[1]:.1f}) '
                 f'End=({gt3e[0]:.1f},{gt3e[1]:.1f}) dist={arc_dist:.2f}m')
        tests.append((f'T3: CW arc  vx={vx3}  wz={wz3}  {dur3}s  r≈{r3:.1f}m',
                       gt3_log, odom3_log, vx3, wz3, dur3, sim_rtf))
        time.sleep(1.5)

        # ── T4: Counter-clockwise arc ──────────────────────────────────────
        vx4, wz4, dur4 = 1.5, +0.8, 8.0
        r4 = abs(vx4 / wz4)
        log.info(f'T4  CCW arc  vx={vx4}  wz={wz4}  dur={dur4}s  '
                 f'expected radius={r4:.2f}m')
        gt4s, _ = self._pose()
        gt4_log, odom4_log = self._record(vx4, wz4, dur4)
        gt4e, _ = self._pose()
        arc_dist4 = math.hypot(gt4e[0]-gt4s[0], gt4e[1]-gt4s[1])
        log.info(f'     Start=({gt4s[0]:.1f},{gt4s[1]:.1f}) '
                 f'End=({gt4e[0]:.1f},{gt4e[1]:.1f}) dist={arc_dist4:.2f}m')
        tests.append((f'T4: CCW arc  vx={vx4}  wz=+{wz4}  {dur4}s  r≈{r4:.1f}m',
                       gt4_log, odom4_log, vx4, wz4, dur4, sim_rtf))

        # ── Summary ───────────────────────────────────────────────────────────
        log.info('=== Summary ===')
        log.info(f'  Simulation real-time factor : {sim_rtf:.2f}×')
        log.info(f'  T1 GT dist vs commanded     : {dist_gt:.2f}m vs {vx*dur:.1f}m '
                 f'(error {abs(dist_gt - vx*dur*sim_rtf):.2f}m)')
        log.info(f'  T2 coast after cmd=0        : {coast:.3f}m')
        log.info(f'  T1 odom vs GT drift         : {abs(dist_odom-dist_gt):.3f}m after 5s straight')

        # ── Plot ──────────────────────────────────────────────────────────────
        fig, axes = plt.subplots(1, 4, figsize=(22, 6))
        fig.suptitle('SEN771 Robot Control Debug — Ground Truth (blue) vs Odometry (red dashed)',
                     fontsize=13, fontweight='bold')

        all_logs = [
            ('T1: Straight  vx=2  wz=0  5s',      gt_log,   odom_log),
            ('T2: Stop accuracy  vx=2.5  3s+coast',gt_run+gt_coast, []),
            (f'T3: CW arc  vx={vx3}  wz={wz3}',   gt3_log,  odom3_log),
            (f'T4: CCW arc  vx={vx4}  wz=+{wz4}', gt4_log,  odom4_log),
        ]

        for ax, (title, gt_l, odom_l) in zip(axes, all_logs):
            if gt_l:
                gx = [p[0] for p in gt_l]
                gy = [p[1] for p in gt_l]
                ax.plot(gx, gy, 'b-', lw=2.5, label='Ground truth (GT)', zorder=4)
                ax.plot(gx[0], gy[0], 'go', ms=10, zorder=6, label='Start')
                ax.plot(gx[-1], gy[-1], 'bs', ms=8, zorder=6, label='End GT')
            if odom_l:
                ox = [p[0] for p in odom_l]
                oy = [p[1] for p in odom_l]
                ax.plot(ox, oy, 'r--', lw=1.8, alpha=0.75, label='Odometry', zorder=3)
                ax.plot(ox[-1], oy[-1], 'r^', ms=8, zorder=5, label='End odom')
            ax.set_title(title, fontsize=9, fontweight='bold')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_aspect('equal')
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.25)

        # Add sim RTF annotation to T1
        axes[0].text(0.05, 0.95,
                     f'Sim RTF ≈ {sim_rtf:.2f}×\nGT={dist_gt:.2f}m  cmd={vx*dur:.1f}m',
                     transform=axes[0].transAxes, fontsize=8,
                     va='top', bbox=dict(boxstyle='round', fc='wheat', alpha=0.8))

        plt.tight_layout()
        os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
        plt.savefig(OUT_PATH, dpi=110, bbox_inches='tight')
        plt.close()
        log.info(f'Plot saved → {OUT_PATH}')
        log.info('=== Debug complete — check the plot ===')


def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
