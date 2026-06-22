#!/usr/bin/env python3
from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from art_planner_msgs.action import PlanToGoal



class PathToGoalClient(Node):

    def sendGoal(self, msg):
        goal = PlanToGoal.Goal()
        goal.goal = msg

        self.client.send_goal_async(goal)

    def goalCallback(self, msg):
        self.sendGoal(msg)

    def clickedPointCallback(self, msg):
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose.position = msg.point
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            '收到点目标: frame=%s, x=%.3f, y=%.3f, z=%.3f'
            % (msg.header.frame_id, msg.point.x, msg.point.y, msg.point.z))
        self.sendGoal(goal_pose)

    def __init__(self):
        super().__init__('path_to_goal_client')
        self.sub = self.create_subscription(PoseStamped, '/goal', self.goalCallback, 1)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clickedPointCallback, 1)

        self.client = ActionClient(self, PlanToGoal, '/art_planner/plan_to_goal')

        print('Waiting for plan_to_goal server to appear...')
        self.client.wait_for_server()
        print('Found server.')



if __name__ == '__main__':
    rclpy.init()
    client = PathToGoalClient()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()
