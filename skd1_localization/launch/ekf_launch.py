from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters = [{'use_sim_time': use_sim_time},
                          PathJoinSubstitution([FindPackageShare('skd1_localization'), 'config', 'ekf.yaml'])],
            remappings=[("odometry/filtered", "odometry/filtered"),
                            ("imu/data", "phidgets_373223/imu/data")],
           )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        ekf_node,
])
