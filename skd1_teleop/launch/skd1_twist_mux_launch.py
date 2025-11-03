from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    twist_mux_params = PathJoinSubstitution(
        [FindPackageShare("skd1_teleop"), 'config', 'twist_mux.yaml']
    )

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': use_sim_time, 'use_stamped': True}],
            # Default publisher for twist_mux is /cmd_vel_out
            # Rename so it matches diff_drive_controller topic
            remappings=[('/cmd_vel_out','/skd1_base_controller/cmd_vel')]
        )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        twist_mux,
    ])