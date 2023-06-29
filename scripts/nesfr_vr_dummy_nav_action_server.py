#!/bin/python3

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NesfrVRDummyNavActionServer(Node):

    def __init__(self):
        super().__init__('dummy_nav_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback)
        self.get_logger().info('Dummy Nav Action Server get started')


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = NavigateToPose.Feedback()

        for i in range(1, 10):
            feedback_msg.current_pose = goal_handle.request.pose
            self.get_logger().info('Feedback to Client: {}'.format(i))
            self.get_logger().info('                       {}'.format(feedback_msg.current_pose))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = NavigateToPose.Result()
        self.get_logger().info('End...')
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = NesfrVRDummyNavActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info(' shutting down by KeyboardInterrupt')

    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
