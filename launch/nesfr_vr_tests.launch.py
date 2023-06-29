import launch
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.substitutions import FindPackageShare
import os

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'


def generate_launch_description():
    dummy_state_machine_node = launch_ros.actions.Node(
        package='nesfr_vr_tests',
        namespace=LaunchConfiguration('namespace'),
        executable='nesfr_vr_dummy_state_machine.py',
        name='dummy_state_machine',
        arguments=['--ros-args', '--log-level', [LaunchConfiguration('namespace'), '.dummy_state_machine:=info'],],
        output='both')

    dummy_nav_action_server_node = launch_ros.actions.Node(
        package='nesfr_vr_tests',
        namespace=LaunchConfiguration('namespace'),
        executable='nesfr_vr_dummy_nav_action_server.py',
        name='dummy_nav_action_server',
        arguments=['--ros-args', '--log-level', [LaunchConfiguration('namespace'), '.dummy_nav_action_server:=debug'],],
        output='both')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='namespace', default_value='',
                                             description='namespace'),
        dummy_state_machine_node,
        dummy_nav_action_server_node,
    ])
