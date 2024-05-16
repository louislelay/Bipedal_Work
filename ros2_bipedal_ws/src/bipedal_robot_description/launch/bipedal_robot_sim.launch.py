import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

	package_name='bipedal_robot_description'

	rsp = IncludeLaunchDescription(
				PythonLaunchDescriptionSource([os.path.join(
					get_package_share_directory(package_name),'launch','bipedal_robot.launch.py'
				)]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
	)

	gazebo = IncludeLaunchDescription(
				PythonLaunchDescriptionSource([os.path.join(
					get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
			 )

	spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
						arguments=['-topic', 'robot_description',
								   '-entity', 'my_bot'],
						output='screen')

	pos_cont_spawner = Node(
		package="controller_manager",
		executable="spawner.py",
		arguments=["position_controller"],
	)

	vel_cont_spawner = Node(
		package="controller_manager",
		executable="spawner.py",
		arguments=["velocity_controller"],
	)


	# Launch them all!
	return LaunchDescription([
		rsp,
		gazebo,
		spawn_entity,
		pos_cont_spawner,
		vel_cont_spawner,
	])

