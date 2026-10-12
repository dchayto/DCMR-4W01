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
            output='screen',
        ),
    ])
