from launch import LaunchDescription
from launch.actions import ExecuteProcess
from datetime import datetime


def generate_launch_description():

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"rosbag_{timestamp}"

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-a',
                '--storage', 'mcap',
                '-o', bag_name
            ],
            output='screen'
        )
    ])
    