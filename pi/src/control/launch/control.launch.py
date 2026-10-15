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
			parameters=[	# had ki_w = 5e-2, kd_w = 1e-6	
			{'Kp_x': 8.0e-2},{'Ki_x': 1.8e-2},{'Kd_x': 1e-2},{'max_ix': 1.0},
			{'Kp_y': 0.0e-2},{'Ki_y': 5.0e-2},{'Kd_y': 0.0},{'max_iy': 1.0},
			{'Kp_w': 0.85},{'Ki_w': 3.5e-1},{'Kd_w': 2e-1},{'max_iw': 1.0}
			],
            output='screen',
        ),
    ])
