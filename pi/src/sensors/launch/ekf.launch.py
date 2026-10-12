#								ekf.launch.py								#
#	launch file to start robot_localization ekf. should be considering
#	wheel odom body twist (TwistWithCovarianceStamped) from odom handler,
#	and imu data (Imu) from imu handler. 
#
#	copied skeleton from chatgpt, then tweaked as required

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sensors_share = get_package_share_directory('sensors')
    ekf_config = os.path.join(
        sensors_share,
        'config',
        'odom_ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='robot_odom_ekf',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
