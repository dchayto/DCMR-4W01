#								control.launch.py							#
#	launch file to start mec_controller_node, and due_interface_node
#	
#	note: kb_handler_node needs to be started separately in a new terminal!!
#

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # due hardware interface
        Node(
            package='control',
            executable='due_interface_node',
            name='due_hw_interface',
            output='screen',
        ),

        # mecanum controller
        Node(
            package='control',
            executable='mecanum_controller_node',
            name='mec_controller',
			parameters=[	# had ki_x at 5.0e-4 before - retuning P
			{'Kp_x': 5.0e-2},{'Ki_x': 2.0e-4},{'Kd_x': 0.0},{'max_ix': 1e-7},
			{'Kp_y': 1.0e-1}, {'Ki_y': 0.0}, {'Kd_y': 0.0}, {'max_iy': 5e-3},
			{'Kp_w': 0.65}, {'Ki_w': 0.0}, {'Kd_w': 0.0}, {'max_iw': 5e-6}
			],
            output='screen',
        ),
    ])
