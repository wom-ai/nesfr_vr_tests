#!/bin/python3

import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32

from wom_ros_interfaces.msg import DriveMode
from wom_ros_interfaces.srv import ChangeDriveMode
from enum import IntEnum

drive_mode_str = {
        DriveMode.OFF:                  "Off",
        DriveMode.IDLE:                 "Idle",
        DriveMode.MANUAL:               "Manual",
        DriveMode.PROTECTIVE_MANUAL:    "Protective Manual",
        DriveMode.ASSISTED_MANUAL:      "Assisted Manual",
        DriveMode.SELF_DRIVING:           "Autonomous",
    }

class WarningLevel(IntEnum):
    NONE                    = 0
    WARNING_ONLY            = 1
    CONTROL_WITH_WARNING    = 2
    MAX                     = 3

class NesfrVRDummyStateMachine(Node):

    def __init__(self):
        super().__init__('dummy_state_machine')

        self.drive_mode_pub_            = self.create_publisher(DriveMode, "drive_mode", 1);
        self.change_drive_mode_srv_     = self.create_service(ChangeDriveMode, "change_drive_mode", self.change_drive_mode);
        self.timer_                     = self.create_timer(0.1, self.publish_drive_mode);

        self.drive_mode_ = DriveMode.OFF
        self.get_logger().info('Dummy State Machine get started')

    def publish_drive_mode(self):
        #self.get_logger().info('publish_drive_mode() publish {}'.format(drive_mode_str[self.drive_mode_]))
        msg = DriveMode()
        msg.mode = self.drive_mode_
        self.drive_mode_pub_.publish(msg)

    def change_drive_mode(self, request, response):

        if request.mode.mode == DriveMode.OFF:
            response.success = True
            response.message = "Driving is off now!"
            self.drive_mode_ = request.mode.mode
        elif request.mode.mode == DriveMode.IDLE:
            response.success = True
            response.message = "Driving is idel now!"
            self.drive_mode_ = request.mode.mode
        elif request.mode.mode == DriveMode.MANUAL:
            response.success = True
            response.message = "Driving is in manual mode now!"
            self.drive_mode_ = request.mode.mode
        elif request.mode.mode == DriveMode.PROTECTIVE_MANUAL:
            response.success = True
            response.message = "Driving is in protective manual mode now!"
            self.drive_mode_ = request.mode.mode
        elif request.mode.mode == DriveMode.ASSISTED_MANUAL:
            response.success = True
            response.message = "Driving is in assisted manual mode now!"
            self.drive_mode_ = request.mode.mode
        elif request.mode.mode == DriveMode.SELF_DRIVING:
            response.success = True
            response.message = "Driving is in self driving mode now!"
            self.drive_mode_ = request.mode.mode
        else:
            response.success = False
            response.message = "Mode not implemented yet!"

        self.get_logger().info(response.message)
        self.get_logger().info('change_drive_mode({}) return {}'.format(request, response))
        return response

def main(args=None):
    rclpy.init(args=args)

    node = NesfrVRDummyStateMachine()

    #
    # reference: https://github.com/ros2/demos/blob/humble/demo_nodes_py/demo_nodes_py/topics/talker.py
    #
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().info(' shutting down by KeyboardInterrupt')

    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
