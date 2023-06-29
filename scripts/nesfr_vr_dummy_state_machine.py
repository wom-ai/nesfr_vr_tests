#!/bin/python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from bstar_ros.msg import NesfrSystemState
from bstar_ros.msg import CollisionWarning

from enum import IntEnum

class RobotStateCmd(IntEnum):
    MANUAL               =  0
    PROTECTIVE_MANUAL    =  1
    AUTONOMOUS           =  2
    TRACKING             =  3
    MAX_NUM              =  4
    NONE                 = 255

robot_state_cmd_str = {
        RobotStateCmd.MANUAL:               "Manual",
        RobotStateCmd.PROTECTIVE_MANUAL:    "Protective Manual",
        RobotStateCmd.AUTONOMOUS:           "Autonomous",
        RobotStateCmd.TRACKING:             "Tracking",
    }

class RobotSystemState(IntEnum):
    OFF                 = 0
    IDLE                = 1
    MANUAL              = 2
    PROTECTIVE_MANUAL   = 3
    AUTONOMOUS          = 4
    EMERGENCY           = 5
    TRACKING            = 6
    MAX                 = 7

robot_system_state_str = {
        RobotSystemState.OFF:       "Off",
        RobotSystemState.IDLE:      "Idle",
        RobotSystemState.MANUAL:    "Manual",
        RobotSystemState.PROTECTIVE_MANUAL: "Protective Manual",
        RobotSystemState.AUTONOMOUS:"Autonomous",
        RobotSystemState.EMERGENCY: "Emergency",
        RobotSystemState.TRACKING:  "Tracking",
    }

class WarningLevel(IntEnum):
    NONE                    = 0
    WARNING_ONLY            = 1
    CONTROL_WITH_WARNING    = 2
    MAX                     = 3

class NesfrVRDummyStateMachine(Node):

    def __init__(self):
        super().__init__('dummy_state_machine')

        self._system_state_publisher = self.create_publisher(NesfrSystemState, 'system_state', 10)
        self._collision_warning_publisher = self.create_publisher(CollisionWarning, 'collision_warning', 10)
        self._subscription = self.create_subscription(Int32, 'state', self._listener_callback,1)

        self._seed = 0

        self.get_logger().info('Dummy State Machine get started')
        self._state = RobotStateCmd.MANUAL

    def _listener_callback(self, msg):
        state = msg.data
        #self.get_logger().info('_listener_callback() I heard: {}'.format(msg.data))

        out_msg = NesfrSystemState()

        is_state_changed = False
        if self._state != state:
            self.get_logger().info('state changes {} -> {}'.format(robot_state_cmd_str[self._state], robot_state_cmd_str[state]))
            self._state = state
            time.sleep(2.0)
            is_state_changed = True

        system_state = RobotSystemState.OFF
        warning_level = WarningLevel.NONE
        if state == RobotStateCmd.MANUAL:
            system_state = RobotSystemState.MANUAL
        elif state == RobotStateCmd.PROTECTIVE_MANUAL:
            system_state = RobotSystemState.PROTECTIVE_MANUAL
            warning_level = (self._seed%WarningLevel.MAX)
            if is_state_changed: self._seed += 1
        elif state == RobotStateCmd.AUTONOMOUS:
            system_state = RobotSystemState.AUTONOMOUS
        elif state == RobotStateCmd.TRACKING:
            system_state = RobotSystemState.TRACKING
        out_msg.system_state = int(system_state)
        out_msg.state_msg = "this is dummy msg"
        self._system_state_publisher.publish(out_msg)


        out_msg = CollisionWarning()
        out_msg.warning_level = int(warning_level)
        self._collision_warning_publisher.publish(out_msg)

import sys
def main(args=None):
    rclpy.init(args=args)

    print("args={}".format(args))
    node = NesfrVRDummyStateMachine()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(' shutting down by KeyboardInterrupt')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
