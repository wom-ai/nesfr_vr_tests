#!/bin/python3

#
# reference
#  - https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/action/server.py
#  - https://github.com/ros2/ros2cli/blob/rolling/ros2action/ros2action/verb/send_goal.py
#  - https://github.com/ROBOTIS-GIT/turtlebot3/blob/galactic-devel/turtlebot3_example/turtlebot3_example/turtlebot3_patrol_server/turtlebot3_patrol_server.py
#  - https://docs.ros2.org/foxy/api/rclpy/api/execution_and_callbacks.html#module-rclpy.callback_groups
#
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action import GoalResponse
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('Dummy Nav Action Server get started')

    def goal_callback(self, goal_request):
        self.get_logger().info('Goal callback...')
        self.get_logger().info('goal_request={}'.format(goal_request))
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        self.get_logger().info('Canceling...')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = NavigateToPose.Feedback()

        for i in range(1, 20):
            time.sleep(1)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToPose.Result()
                return result

            feedback_msg.current_pose = goal_handle.request.pose
            self.get_logger().info('Feedback to Client: {}'.format(i))
            self.get_logger().info('                       position   ={}'.format(feedback_msg.current_pose.pose.position))
            self.get_logger().info('                       orientation={}'.format(feedback_msg.current_pose.pose.orientation))
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = NavigateToPose.Result()
        self.get_logger().info('End...')
        return result

def main(args=None):
    rclpy.init(args=args)

    action_server = NesfrVRDummyNavActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(action_server, executor=executor)
        #rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info(' shutting down by KeyboardInterrupt')

    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
