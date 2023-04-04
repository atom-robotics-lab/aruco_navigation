import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

	# get the required paths of packages & files
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
	pkg_mr_robot_desc = get_package_share_directory('mr_robot_description')
	xacro_path = pkg_mr_robot_desc + '/urdf/mr_robot.xacro'
	
	# TODO: take rviz config file as launch arg 
	# use this one as default
	rviz_config = pkg_mr_robot_desc + '/config/urdf.rviz'
	bridge_config = pkg_mr_robot_desc + '/config/bridge.yaml'

	# launch configs to use launch args
	use_sim_time = LaunchConfiguration('use_sim_time')
	with_rviz = LaunchConfiguration('rviz')
	with_bridge = LaunchConfiguration('with_bridge')

	# create urdf from xacro 
	robot_xacro_config = xacro.process_file(xacro_path)
	robot_urdf = robot_xacro_config.toxml()

	# spawn robot in gz sim using urdf
	spawn_robot = Node(package='ros_gz_sim', executable='create',
				arguments=['-name', 'mr_robot',
					'-x', '5.04',
					'-y', '1.95',
					'-z', '0.50',
					'-Y', '-1.57',
					'-string', robot_urdf],
				output='screen')

	# launch rviz node if rviz parameter was set to true
	rviz = Node(package='rviz2',
				executable='rviz2',
				name='rviz',
#				output='screen',
				arguments=['-d' + rviz_config],
				condition=IfCondition(with_rviz))
	
	# robot state publisher node
	state_publisher = Node(package='robot_state_publisher', 
				executable='robot_state_publisher',
				output='screen',
				parameters = [
					{'ignore_timestamp': False},
					{'use_sim_time': use_sim_time},
					{'use_tf_static': True},
					{'robot_description': robot_urdf}],
				arguments = [robot_urdf])

	# parameter bridge node to bridge different gz and tos 2 topics
	ros_gz_bridge = Node(package="ros_gz_bridge", 
				executable="parameter_bridge",
                parameters = [
                    {'config_file': bridge_config}],
				condition=IfCondition(with_bridge)
                )

	
	lidar_stf = Node(package='tf2_ros', executable='static_transform_publisher',
            name = 'lidar_stf',
                arguments = [
                    '0', '0', '0', '0', '0', '0', '1',
                    'lidar_1',
                    'mr_robot/base_link/front_rplidar'
            ])

	# use_sim_time launch argument
	arg_use_sim_time = DeclareLaunchArgument('use_sim_time',
											default_value='true',
											description="Enable sim time from /clock")
	
	# argument to specify if rviz needs to be launched
	arg_with_rviz = DeclareLaunchArgument('rviz', default_value='false',
											description="Set true to launch rviz")

	# argument to specify if bridge needs to be launched
	arg_with_bridge = DeclareLaunchArgument('with_bridge', default_value='true',
											description="Set true to bridge ROS 2 & Gz topics")
	
	return LaunchDescription([
		arg_use_sim_time,
		arg_with_rviz,
		arg_with_bridge,
		spawn_robot,
		ros_gz_bridge,
		rviz,
		state_publisher,
		lidar_stf
	])