#								sensors.launch.py							#
#	launch file to start wheel_odom_node and imu_node
#
#	copied skeleton from chatgpt, just tweaked as required
#

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
		# static publisher for imu transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_transform',
            arguments=[
                '0.0', '0.0', '0.0',  # x, y, z (m) - imu centred on chass
                '0.0', '0.0', '0.0',  # yaw, pitch, roll (rad)
                'base_link',
                'imu_link'
            ],
			output='screen',
        ),

		# imu handler node
		Node(
            package='sensors',
            executable='imu_handler.py',
            name='imu_handler',
            output='screen',
        ),

        # wheel odometry 
        Node(
            package='sensors',
            executable='wheel_odom_node',
            name='wheel_odom',
            output='screen',
        ),


    ])

