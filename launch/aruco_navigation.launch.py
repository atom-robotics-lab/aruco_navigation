import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_mr_robot_desc = get_package_share_directory('aruco_navigation')

    # launch GZ Sim with empty world
    gz_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args' : pkg_mr_robot_desc + '/worlds/aruco_marker_world.sdf'
                }.items()          
            )
    
    # spawn robot with rviz
    # robot = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(pkg_mr_robot_desc, 'launch', 'robot.launch.py')
    #             ),
    #             launch_arguments={
    #                 'rviz': 'true',
    #                 'with_bridge': 'true'
    #             }.items()
    #         )

    return LaunchDescription([
        gz_sim,
        # robot
    ])