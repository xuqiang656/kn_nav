#!/usr/bin/env python3
"""
pct_nav2_bridge: Subscribe /pct_path → dispatch FollowPath action to Nav2 controller.

Architecture:
  PCT planner → /pct_path (nav_msgs/Path)
       ↓
  bridge_node (this file)
       ↓
  Nav2 ControllerServer (FollowPath action)
       ↓
  /cmd_vel → robot

Key behaviours:
  - New /pct_path cancels any in-flight FollowPath goal before sending the new one.
  - Empty path is ignored; frame_id is verified against 'map'.
  - On goal abort/reject, zero velocity is published.
  - Path segmentation for cross-floor (stair) scenarios is supported.
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener


# ─── path segment classification ────────────────────────────────────────────

class SegmentType(Enum):
    FLAT = 0
    STAIR_ENTRY = 1
    STAIR = 2
    STAIR_EXIT = 3


def classify_segment_z(poses, stair_dz_threshold=0.15, flat_window=5):
    """
    Simple z-gradient classifier: groups consecutive poses by dz.
    Returns list of (SegmentType, start_idx, end_idx).
    """
    if len(poses) < 2:
        return [(SegmentType.FLAT, 0, len(poses) - 1)] if poses else []

    segments = []
    n = len(poses)
    i = 0
    while i < n - 1:
        # look ahead to determine segment kind
        dz = abs(poses[i + 1].pose.position.z - poses[i].pose.position.z)
        if dz > stair_dz_threshold:
            # entering stair region
            start = i
            while i < n - 1 and abs(poses[i + 1].pose.position.z - poses[i].pose.position.z) > stair_dz_threshold * 0.5:
                i += 1
            segments.append((SegmentType.STAIR, start, i))
        else:
            start = i
            while i < n - 1 and abs(poses[i + 1].pose.position.z - poses[i].pose.position.z) <= stair_dz_threshold:
                i += 1
            segments.append((SegmentType.FLAT, start, i))
    else:
        # catch trailing single pose
        if segments and segments[-1][2] < n - 1:
            segments.append((SegmentType.FLAT, segments[-1][2] + 1, n - 1))
        elif not segments:
            segments.append((SegmentType.FLAT, 0, n - 1))

    return segments


# ─── bridge node ────────────────────────────────────────────────────────────

class PctNav2Bridge(Node):
    """Bridge between PCT global planner and Nav2 controller server."""

    def __init__(self):
        super().__init__('pct_nav2_bridge')

        # --- parameters ---
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'goal_checker')
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('zero_vel_on_failure', True)
        self.declare_parameter('clip_nearby', False)
        self.declare_parameter('clip_radius_m', 0.3)
        self.declare_parameter('segment_cross_floor', False)
        self.declare_parameter('segment_stair_dz', 0.15)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value
        self.path_frame = self.get_parameter('path_frame').value
        self.zero_on_fail = self.get_parameter('zero_vel_on_failure').value
        self.clip_nearby = self.get_parameter('clip_nearby').value
        self.clip_radius = self.get_parameter('clip_radius_m').value
        self.segment_cross_floor = self.get_parameter('segment_cross_floor').value
        self.stair_dz = self.get_parameter('segment_stair_dz').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- latest odom (for clipping closest point) ---
        self.latest_odom = None
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # --- PCT path subscriber ---
        self.path_sub = self.create_subscription(
            Path, '/pct_path', self._path_cb, 10)

        # --- FollowPath action client ---
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._goal_handle = None
        self._goal_future = None
        self._result_future = None
        self._pending_path = None  # path waiting to be sent after current goal resolves
        self._segment_queue = []   # remaining segments (for cross-floor mode)
        self._active = False

        # --- zero-velocity publisher (safety fallback) ---
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # --- wait for action server ---
        server_timeout_s = 10.0
        if not self._action_client.wait_for_server(timeout_sec=server_timeout_s):
            self.get_logger().warn(
                f'FollowPath action server not available after {server_timeout_s}s. '
                'Will keep waiting...')
        else:
            self.get_logger().info('FollowPath action server connected.')

        self.get_logger().info('pct_nav2_bridge started.')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _path_cb(self, msg: Path):
        """New /pct_path received."""
        if not msg.poses:
            self.get_logger().warn('Received empty /pct_path — ignoring.')
            self._publish_zero_velocity()
            return

        # verify frame — reject if mismatch (Nav2 controller needs correct frame)
        if msg.header.frame_id != self.path_frame:
            self.get_logger().error(
                f'Path frame_id "{msg.header.frame_id}" != '
                f'expected "{self.path_frame}". REJECTING path. '
                'Publishing zero velocity for safety.')
            self._publish_zero_velocity()
            return

        self.get_logger().info(
            f'Received /pct_path: {len(msg.poses)} waypoints, '
            f'frame={msg.header.frame_id}')

        # Cancel current goal if active
        if self._active and self._goal_handle is not None:
            self.get_logger().info('Cancelling previous FollowPath goal...')
            self._pending_path = msg
            self._segment_queue.clear()  # discard old segments
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
        else:
            self._send_path(msg)

    def _cancel_done_callback(self, future):
        """Called when cancellation is complete; send the pending path."""
        cancel_response = future.result()
        if cancel_response is not None and len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Previous goal successfully cancelled.')
        else:
            self.get_logger().warn('Previous goal cancellation failed or already completed.')
        self._active = False
        self._goal_handle = None

        if self._pending_path is not None:
            path = self._pending_path
            self._pending_path = None
            self._send_path(path)

    # ── path sending ────────────────────────────────────────────────────────

    def _send_path(self, path_msg: Path):
        """Prepare and send a FollowPath goal (or segment queue)."""
        clipped = self._clip_path(path_msg)
        if clipped is None or not clipped.poses:
            self.get_logger().warn('Path empty after clipping — skipping.')
            return

        # Cross-floor segmentation: split into flat/stair segments, send sequentially
        if self.segment_cross_floor:
            segments = self.segment_path(clipped)
            if len(segments) > 1:
                self.get_logger().info(
                    f'Cross-floor mode: {len(segments)} segments. '
                    f'Sending segment 1/{len(segments)}.')
                self._segment_queue = segments[1:]  # remaining segments
                self._send_goal(segments[0][1])
                return
            # else: single segment, fall through to direct send

        self._send_goal(clipped)

    def _send_goal(self, path_msg: Path):
        """Send a single FollowPath goal to the action server."""
        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f'Sending FollowPath goal: {len(path_msg.poses)} poses, '
            f'controller={self.controller_id}, '
            f'goal_checker={self.goal_checker_id}')

        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_callback)
        self._active = True

    def _get_robot_pose_in_frame(self, target_frame: str, timeout: float = 0.2):
        """
        Look up the robot's current pose in target_frame via TF.
        Returns (x, y) or None if the transform is unavailable.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                'base_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout))
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:
            self.get_logger().debug(
                f'TF lookup base_link→{target_frame} failed: {e}')
            return None

    def _clip_path(self, path_msg: Path):
        """
        Optionally clip the portion of the path behind the robot.
        Uses TF to transform robot pose into the path's frame, avoiding
        frame-mismatch errors between /odom and /pct_path (map).
        Returns a new Path message (or the original if clipping is disabled).
        """
        if not self.clip_nearby:
            return path_msg

        # Use TF to get robot pose in the same frame as the path
        robot_xy = self._get_robot_pose_in_frame(path_msg.header.frame_id)
        if robot_xy is None:
            self.get_logger().warn(
                'Cannot clip path: TF lookup failed. Sending full path.')
            return path_msg

        robot_x, robot_y = robot_xy

        # find index of the closest point
        best_idx = 0
        best_dist = float('inf')
        for i, pose_stamped in enumerate(path_msg.poses):
            dx = pose_stamped.pose.position.x - robot_x
            dy = pose_stamped.pose.position.y - robot_y
            d = math.sqrt(dx * dx + dy * dy)
            if d < best_dist:
                best_dist = d
                best_idx = i

        # keep from best_idx onward; skip points within clip_radius
        start = best_idx
        for i in range(best_idx, len(path_msg.poses)):
            p = path_msg.poses[i]
            dx = p.pose.position.x - robot_x
            dy = p.pose.position.y - robot_y
            if math.sqrt(dx * dx + dy * dy) > self.clip_radius:
                start = max(best_idx, i)
                break

        clipped = Path()
        clipped.header = path_msg.header
        clipped.poses = path_msg.poses[start:]
        if len(clipped.poses) < len(path_msg.poses):
            self.get_logger().debug(
                f'Clipped {len(path_msg.poses) - len(clipped.poses)} '
                f'waypoints (behind robot).')
        return clipped

    def _goal_response_callback(self, future):
        """Handle FollowPath goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected by controller server.')
            self._active = False
            self._goal_handle = None
            if self.zero_on_fail:
                self._publish_zero_velocity()
            return

        self.get_logger().info('FollowPath goal accepted.')
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _feedback_cb(self, feedback_msg):
        """Optional feedback logging (distance remaining, etc.)."""
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f'Feedback: distance_remaining={fb.distance_to_goal:.2f}, '
            f'speed={fb.speed:.2f}',
            throttle_duration_sec=2.0)

    def _result_callback(self, future):
        """Handle FollowPath result."""
        self._active = False
        self._goal_handle = None
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('FollowPath succeeded — path complete.')
            # Check for remaining cross-floor segments
            if self._segment_queue:
                seg_type, next_seg = self._segment_queue.pop(0)
                remaining = len(self._segment_queue) + 1
                self.get_logger().info(
                    f'Segment done. Sending next ({seg_type.name}), '
                    f'{remaining} remaining.')
                self._send_goal(next_seg)
                return
            # ControllerServer's publish_zero_velocity handles stopping.
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('FollowPath was cancelled.')
            self._segment_queue.clear()  # abort remaining segments
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('FollowPath ABORTED — publishing zero velocity.')
            self._segment_queue.clear()  # abort remaining segments
            if self.zero_on_fail:
                self._publish_zero_velocity()
        else:
            self.get_logger().warn(f'FollowPath ended with status={status}.')
            self._segment_queue.clear()

        # If there's another pending path, send it
        if self._pending_path is not None:
            path = self._pending_path
            self._pending_path = None
            self._send_path(path)

    # ── safety ──────────────────────────────────────────────────────────────

    def _publish_zero_velocity(self):
        """Publish zero Twist to stop the robot."""
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        self.get_logger().info('Published zero velocity (safety stop).')

    # ── path segmentation (cross-floor) ─────────────────────────────────────

    def segment_path(self, path_msg: Path):
        """
        Split path into flat/stair segments.
        Returns list of (segment_type, sub_path).
        """
        if not self.segment_cross_floor or len(path_msg.poses) < 2:
            return [(SegmentType.FLAT, path_msg)]

        segs = classify_segment_z(path_msg.poses, stair_dz_threshold=self.stair_dz)
        result = []
        for seg_type, start, end in segs:
            sub = Path()
            sub.header = path_msg.header
            sub.poses = path_msg.poses[start:end + 1]
            result.append((seg_type, sub))
            self.get_logger().info(
                f'Segment [{start}:{end}] type={seg_type.name} '
                f'({len(sub.poses)} poses)')
        return result


# ─── main ───────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PctNav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bridge node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
