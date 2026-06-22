#!/usr/bin/env python3
import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener
import tf_transformations


ROBOT_FRAME = 'base'
GOAL_THRES_POS = 0.2
GOAL_THRES_ANG = 0.2
FACE_GOAL_DIST = 1.0


def getYaw(quat):
    _, _, yaw = tf_transformations.euler_from_quaternion(quat)
    return yaw


def getConstrainedYaw(yaw):
    while yaw > math.pi:
        yaw -= 2 * math.pi
    while yaw < -math.pi:
        yaw += 2 * math.pi
    return yaw


def getAngleError(target, current):
    return getConstrainedYaw(target - current)


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_pid')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_twist = self.create_publisher(TwistStamped, '/path_planning_and_following/twist', 1)
        self.pub_path = self.create_publisher(Path, '/art_planner/followed_path', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sub = self.create_subscription(Path, '/art_planner/path', self.pathCallback, 1)
        self.current_pose = None
        self.goal_pose = None
        self.fixed_frame = None
        self.path = None
        self.path_ros = None
        self.gain_pid_pos = [2, 0.0, 0.0]
        self.gain_pid_ang = [5, 0.0, 0.0]
        self.i = [0, 0, 0]
        self.timer = self.create_timer(0.1, self.computeAndPublishTwist)

    def pathCallback(self, path_msg):
        self.fixed_frame = path_msg.header.frame_id
        self.path = []
        self.path_ros = path_msg
        self.goal_pose = None

        if len(path_msg.poses) > 1:
            for ros_pose in path_msg.poses:
                pos = ros_pose.pose.position
                rot = ros_pose.pose.orientation
                yaw = getYaw([rot.x, rot.y, rot.z, rot.w])
                self.path.append([pos.x, pos.y, yaw])

            self.removePathNodesBeforeIndex(1)
            self.get_logger().info('Got path: ' + str(self.path))
            self.i = [0, 0, 0]
        else:
            self.get_logger().warn('Path message is too short')

    def updateCurrentPose(self):
        if self.fixed_frame is None:
            self.get_logger().warn('Fixed frame not set.')
            return

        try:
            transform = self.tf_buffer.lookup_transform(self.fixed_frame, ROBOT_FRAME, rclpy.time.Time())
        except TransformException:
            self.get_logger().warn('TF lookup of pose failed')
            return

        trans = transform.transform.translation
        rot = transform.transform.rotation
        self.current_pose = [trans.x, trans.y, getYaw([rot.x, rot.y, rot.z, rot.w])]

    def publishPath(self):
        if self.fixed_frame is not None:
            msg = Path()
            msg.header.frame_id = self.fixed_frame
            if self.path_ros is not None:
                msg.poses = self.path_ros.poses
            self.pub_path.publish(msg)

    def removePathNodesBeforeIndex(self, index):
        self.path = self.path[index:]
        self.path_ros.poses = self.path_ros.poses[index:]

    def updateCurrentGoalPose(self):
        if self.goal_pose is not None:
            dx = self.goal_pose[0] - self.current_pose[0]
            dy = self.goal_pose[1] - self.current_pose[1]
            dist = (dx**2 + dy**2)**0.5
            dyaw = getAngleError(self.goal_pose[2], self.current_pose[2])
            if dist < GOAL_THRES_POS and abs(dyaw) < GOAL_THRES_ANG:
                if len(self.path) > 1:
                    self.removePathNodesBeforeIndex(1)
                else:
                    self.path = None
                    self.path_ros = None
                    self.publishPath()
                self.goal_pose = None

        if self.goal_pose is None and self.current_pose is not None and self.path is not None:
            largest_valid_index = 0
            for i in range(len(self.path) - 1):
                path_segment = np.array([self.path[i + 1][0] - self.path[i][0],
                                         self.path[i + 1][1] - self.path[i][1]])
                robot_from_node = np.array([self.current_pose[0] - self.path[i][0],
                                            self.current_pose[1] - self.path[i][1]])
                dist_along_path = robot_from_node.dot(path_segment)
                if dist_along_path > 0 and i + 1 > largest_valid_index:
                    largest_valid_index = i + 1
                else:
                    break

            self.removePathNodesBeforeIndex(largest_valid_index)
            self.goal_pose = self.path[0]
            self.publishPath()

    def getYawTarget(self):
        dx = self.goal_pose[0] - self.current_pose[0]
        dy = self.goal_pose[1] - self.current_pose[1]
        dist = (dx**2 + dy**2)**0.5

        if dist < FACE_GOAL_DIST:
            return self.goal_pose[2]

        yaw_target = math.atan2(dy, dx)
        error = getAngleError(yaw_target, self.current_pose[2])
        if abs(error) > math.pi * 0.5:
            yaw_target = getConstrainedYaw(yaw_target + math.pi)
        return yaw_target

    def computeAndPublishTwist(self):
        self.updateCurrentPose()
        if self.path is None or self.current_pose is None:
            return

        self.updateCurrentGoalPose()
        if self.goal_pose is None:
            return

        msg = TwistStamped()
        msg.header.frame_id = ROBOT_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()

        yaw = self.current_pose[2]
        yaw_target = self.getYawTarget()

        dx = self.goal_pose[0] - self.current_pose[0]
        dy = self.goal_pose[1] - self.current_pose[1]
        dyaw = getAngleError(yaw_target, self.current_pose[2])

        dlon = math.cos(yaw) * dx + math.sin(yaw) * dy
        dlat = -math.sin(yaw) * dx + math.cos(yaw) * dy

        self.i[0] += dlon
        self.i[1] += dlat
        self.i[2] += dyaw

        msg.twist.linear.x = dlon * self.gain_pid_pos[0] + self.i[0] * self.gain_pid_pos[1]
        msg.twist.linear.y = dlat * self.gain_pid_pos[0] + self.i[1] * self.gain_pid_pos[1]
        msg.twist.angular.z = dyaw * self.gain_pid_ang[0] + self.i[2] * self.gain_pid_ang[1]

        self.pub_twist.publish(msg)


def main():
    rclpy.init()
    follower = PathFollower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
