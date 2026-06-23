#!/usr/bin/env python3
"""
pure_pursuit_follower: Bypass Nav2 — direct /pct_path → /cmd_vel via pure pursuit.

Architecture (Plan A):
  PCT planner → /pct_path (nav_msgs/Path, frame_id=map)
       ↓
  TF: map → base_link  (robot pose lookup)
       ↓
  pure_pursuit_follower (this node)
       ↓
  /cmd_vel (geometry_msgs/Twist)

No Nav2 ControllerServer, no local costmap, no /scan required.

Algorithm:
  1. Find closest path point to robot.
  2. Walk forward from that point accumulating distance to find lookahead point.
  3. Transform lookahead point to robot frame.
  4. Pure pursuit curvature:  κ = 2·ly / L²  (ly = lateral offset, L = lookahead)
  5. ω = v·κ, clamped to max_angular_vel.
  6. Slow down linear velocity when yaw error to lookahead is large.
  7. Stop when within goal_tolerance of final waypoint.
  8. Safety: TF loss / empty path / path timeout → zero velocity.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quaternion(q):
    """Extract yaw from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitFollower(Node):
    """Direct pure pursuit tracker — no Nav2 dependency."""

    def __init__(self):
        super().__init__('pure_pursuit_follower')

        # ── parameters ──────────────────────────────────────────────────
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('lookahead_dist', 0.5)
        self.declare_parameter('min_lookahead_dist', 0.3)
        self.declare_parameter('max_lookahead_dist', 0.8)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('min_linear_vel', 0.05)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('slowdown_yaw_error', 0.7)  # rad
        self.declare_parameter('path_timeout', 2.0)        # seconds
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate', 20.0)       # Hz

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.min_lookahead = self.get_parameter('min_lookahead_dist').value
        self.max_lookahead = self.get_parameter('max_lookahead_dist').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.slowdown_yaw_error = self.get_parameter('slowdown_yaw_error').value
        self.path_timeout = self.get_parameter('path_timeout').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_rate = self.get_parameter('control_rate').value

        # ── TF ──────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── current path ────────────────────────────────────────────────
        self.current_path = None       # nav_msgs/Path
        self.last_path_time = None     # rostime of last received path

        # ── publishers / subscribers ─────────────────────────────────────
        self.path_sub = self.create_subscription(
            Path, '/pct_path', self._path_cb, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

        # ── control loop timer ───────────────────────────────────────────
        dt = 1.0 / self.control_rate
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'PurePursuitFollower started: '
            f'global={self.global_frame}, robot={self.robot_frame}, '
            f'lookahead={self.lookahead_dist}m, '
            f'v_max={self.max_linear_vel}m/s, '
            f'ω_max={self.max_angular_vel}rad/s, '
            f'goal_tol={self.goal_tolerance}m')

    # ── callback ────────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        """Receive new /pct_path."""
        if not msg.poses:
            self.get_logger().warn('Received empty /pct_path — ignoring.')
            return

        if msg.header.frame_id != self.global_frame:
            self.get_logger().error(
                f'Path frame_id "{msg.header.frame_id}" != '
                f'expected "{self.global_frame}". REJECTING.')
            return

        self.current_path = msg
        self.last_path_time = self.get_clock().now()
        self.get_logger().info(
            f'New path: {len(msg.poses)} waypoints, '
            f'frame={msg.header.frame_id}')

    # ── control loop ────────────────────────────────────────────────────────

    def _control_loop(self):
        """Main timer callback: compute and publish cmd_vel."""
        now = self.get_clock().now()

        # ── safety checks ───────────────────────────────────────────────
        if self.current_path is None:
            self._publish_zero('no path received yet')
            return

        # path timeout
        if self.last_path_time is not None:
            elapsed = (now - self.last_path_time).nanoseconds * 1e-9
            if elapsed > self.path_timeout:
                self.get_logger().warn(
                    f'Path timeout ({elapsed:.1f}s > {self.path_timeout}s). '
                    'Stopping.', throttle_duration_sec=2.0)
                self._publish_zero('path timeout')
                return

        # empty path
        if not self.current_path.poses:
            self._publish_zero('empty path')
            return

        # ── get robot pose in path frame via TF ─────────────────────────
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup {self.global_frame}→{self.robot_frame} '
                f'failed: {e}', throttle_duration_sec=2.0)
            self._publish_zero('TF lookup failed')
            return

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        ryaw = yaw_from_quaternion(t.transform.rotation)

        # ── extract path as list of (x, y) ──────────────────────────────
        poses = self.current_path.poses
        path_points = [(p.pose.position.x, p.pose.position.y) for p in poses]

        # ── find closest point index ────────────────────────────────────
        closest_idx = 0
        closest_dist_sq = float('inf')
        for i, (px, py) in enumerate(path_points):
            dx = rx - px
            dy = ry - py
            d2 = dx * dx + dy * dy
            if d2 < closest_dist_sq:
                closest_dist_sq = d2
                closest_idx = i

        # ── find lookahead point ────────────────────────────────────────
        lookahead_idx = self._find_lookahead(path_points, closest_idx, rx, ry)
        if lookahead_idx is None:
            lookahead_idx = len(path_points) - 1

        lx, ly = path_points[lookahead_idx]

        # ── check goal reached ──────────────────────────────────────────
        goal_x, goal_y = path_points[-1]
        dist_to_goal = math.sqrt((rx - goal_x)**2 + (ry - goal_y)**2)
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(
                f'Goal reached: {dist_to_goal:.2f}m < '
                f'{self.goal_tolerance}m. Stopping.',
                throttle_duration_sec=1.0)
            self._publish_zero('goal reached')
            self.current_path = None  # clear path so we don't keep tracking
            return

        # ── transform lookahead to robot frame ──────────────────────────
        dx = lx - rx
        dy = ly - ry
        # rotate by -yaw into robot frame
        cos_yaw = math.cos(-ryaw)
        sin_yaw = math.sin(-ryaw)
        lx_robot = dx * cos_yaw - dy * sin_yaw
        ly_robot = dx * sin_yaw + dy * cos_yaw

        # ── pure pursuit curvature ──────────────────────────────────────
        # κ = 2 * lateral_offset / lookahead²
        L = self.lookahead_dist
        curvature = 2.0 * ly_robot / (L * L)

        # ── angular velocity ────────────────────────────────────────────
        angular_vel = self.max_linear_vel * curvature
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        # ── linear velocity with yaw-error slowdown ─────────────────────
        yaw_error = math.atan2(ly_robot, lx_robot)  # angle to lookahead in robot frame
        abs_yaw_err = abs(yaw_error)
        if abs_yaw_err > self.slowdown_yaw_error:
            # linear scale from min_linear_vel to max_linear_vel
            ratio = min(1.0, (abs_yaw_err - self.slowdown_yaw_error) / (math.pi - self.slowdown_yaw_error))
            linear_vel = self.max_linear_vel - ratio * (self.max_linear_vel - self.min_linear_vel)
        else:
            linear_vel = self.max_linear_vel

        # ── publish ─────────────────────────────────────────────────────
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().debug(
            f'cmd_vel: v={linear_vel:.2f}, ω={angular_vel:.2f}, '
            f'closest_idx={closest_idx}, lookahead_idx={lookahead_idx}, '
            f'dist_to_goal={dist_to_goal:.2f}',
            throttle_duration_sec=1.0)

    # ── lookahead search ────────────────────────────────────────────────────

    def _find_lookahead(self, path_points, start_idx, rx, ry):
        """
        Walk forward from start_idx, accumulating Euclidean distance.
        Return index of the first point whose cumulative distance >= lookahead_dist.
        If we run out of path, return the last index.
        """
        # try adaptive lookahead: scale with distance to goal
        goal_x, goal_y = path_points[-1]
        dist_to_goal = math.sqrt((rx - goal_x)**2 + (ry - goal_y)**2)
        L = self.lookahead_dist
        L = max(self.min_lookahead, min(L, dist_to_goal * 0.5))
        L = min(self.max_lookahead, L)

        accumulated = 0.0
        prev_x, prev_y = path_points[start_idx]

        for i in range(start_idx + 1, len(path_points)):
            px, py = path_points[i]
            segment = math.sqrt((px - prev_x)**2 + (py - prev_y)**2)
            accumulated += segment
            if accumulated >= L:
                return i
            prev_x, prev_y = px, py

        # fell off end of path — use last point
        return len(path_points) - 1

    # ── safety ──────────────────────────────────────────────────────────────

    def _publish_zero(self, reason=''):
        """Publish zero Twist and log reason."""
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        if reason:
            self.get_logger().debug(f'Zero velocity ({reason})',
                                    throttle_duration_sec=1.0)


# ─── main ───────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pure pursuit follower stopped by user.')
    finally:
        # publish zero velocity on shutdown
        node._publish_zero('shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
