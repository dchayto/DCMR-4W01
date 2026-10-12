#								robot.launch.py								#
#	launch file to bringup robot. needs to start control.launch.py, 
#	sensors.launch.py, and ekf.launch.py
#
#	copied skeleton from chatgpt, just tweaking as required
#

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    control_share = get_package_share_directory('control')
    sensors_share = get_package_share_directory('sensors')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    control_share,
                    'launch',
                    'control.launch.py'
                )
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    sensors_share,
                    'launch',
                    'sensors.launch.py'
                )
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    sensors_share,
                    'launch',
                    'ekf.launch.py'
                )
            )
        ),
    ])
