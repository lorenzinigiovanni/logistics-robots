from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction)

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Top-level namespace')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            name='nav2_waypoint_follower_client',
            package='turtlebot3_nav',
            executable='follow_waypoints_client',
            namespace=namespace,
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
