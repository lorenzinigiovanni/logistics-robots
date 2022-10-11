import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gui = LaunchConfiguration('gui')
    
    world_file_name = 'povo.world'
    robots = [
        {'name': 'robot_0', 'x_pose': 68.0, 'y_pose': -20.0, 'z_pose': 0.5,
         'roll': 0.0, 'pitch': 0.0, 'yaw': 1.57},
        {'name': 'robot_1', 'x_pose': 75.0, 'y_pose': 20.0, 'z_pose': 0.5,
         'roll': 0.0, 'pitch': 0.0, 'yaw': 0}
    ]

    simulation_dir = get_package_share_directory('simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    world = os.path.join(simulation_dir, 'worlds', world_file_name)
    launch_file_dir = os.path.join(simulation_dir, 'launch')

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable gazebo visualization')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    ld = LaunchDescription()

    ld.add_action(gui_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    for i, robot in enumerate(robots):
        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'robot_name': robot['name'],
                'namespace': 'tb3_{}'.format(i),
                'use_sim_time': 'True',
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                'roll': TextSubstitution(text=str(robot['roll'])),
                'pitch': TextSubstitution(text=str(robot['pitch'])),
                'yaw': TextSubstitution(text=str(robot['yaw'])),
            }.items()
        )

        ld.add_action(spawn_turtlebot_cmd)

    return ld
