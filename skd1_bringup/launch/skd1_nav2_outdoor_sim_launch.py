from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():

    # Hardcoded true per request -- this bringup is sim-only.
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    ARGUMENTS = [
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('skd1_navigation'), 'config', 'nav2_outdoor_params.yaml']
            ),
            description='Full path to param yaml file to load for Nav2 (GPS/EKF-only, outdoor)',
        ),
    ]

    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # ---- Gazebo simulation bringup ----
    # NOTE: confirm this is the actual launch filename/package in skd1_simulation.
    skd1_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('skd1_gz'), 'launch', 'skd1_gz_launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- Twist mux (teleop / nav cmd_vel arbitration) ----
    teleop_twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('skd1_teleop'), 'launch', 'skd1_twist_mux_launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- Dual EKF + navsat_transform (GPS/IMU/odom fusion) ----
    # NOTE: confirm this is the actual launch filename in skd1_localization --
    # this is the combined dual_ekf_navsat setup you already have configured
    # and tested outside this file.
    dual_ekf_navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('skd1_localization'), 'launch', 'dual_ekf_navsat_launch.py']
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- Nav2 (stock bringup launch + our GPS/EKF-only params file) ----
    nav2_outdoor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('skd1_navigation'), 'launch', 'navigation_outdoor_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )
    
    custom_nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('skd1_navigation'),
                   'launch',
                   'custom_navigation_launch.py']),
            ),
            launch_arguments = {'use_sim_time': use_sim_time,
                                'params_file': nav2_params_file}.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(skd1_gz_launch)
    ld.add_action(teleop_twist_mux_launch)
    ld.add_action(dual_ekf_navsat_launch)
    ld.add_action(custom_nav2_launch)
    # ld.add_action(nav2_outdoor_launch)
    return ld