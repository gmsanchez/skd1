from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("skd1_description"), 'urdf', 'skd1.urdf.xacro']
    )

    # https://articulatedrobotics.xyz/mobile-robot-12a-ros2-control-extra/
    robot_description = ParameterValue(Command(['xacro ', urdf_path, ' is_sim:=', use_sim_time]), value_type=str)
    
    # parameters =[{'robot_description': ParameterValue(  robot_description, value_type=str)  }]

    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{ 'robot_description': robot_description,
                        'use_sim_time': use_sim_time }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        node_robot_state_publisher,
    ])