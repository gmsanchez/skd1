from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    ARGUMENTS = [
        DeclareLaunchArgument('use_sim_time', default_value='False',
            description='Use simulation (Gazebo) clock if true'),
    ]
    
    skd1_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("skd1_description"),
                "launch",
                "skd1_description_launch.py"
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
        
    config_skd1_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("skd1_control"),
        "config",
        "diff_drive_controller.yaml"],
    )

    # Sacado del launch de https://github.dev/husarion/rosbot_hardware_interfaces
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'is_sim': use_sim_time}, config_skd1_velocity_controller],
        arguments=['--ros-args', '--log-level', 'WARN']
    )
    
    # Sacado del launch de https://github.dev/husarion/rosbot_hardware_interfaces
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            # '--ros-args', '--log-level', 'debug',
        ],
    )

    # Sacado del launch de https://github.dev/husarion/rosbot_hardware_interfaces
    spawn_skd1_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "skd1_base_controller",
            "--controller-manager",
            "/controller_manager",
            # '--ros-args', '--log-level', 'debug',
        ],
    )
    
    # Sacado del launch de https://github.dev/husarion/rosbot_hardware_interfaces
    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_skd1_velocity_controller],
            )
        )
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(skd1_description)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(delay_robot_controller_spawner_after_joint_state_broadcaster_spawner)
    
    return ld
