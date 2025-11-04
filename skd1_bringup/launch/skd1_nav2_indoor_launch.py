# import os

# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():

   use_sim_time = LaunchConfiguration('use_sim_time', default='False')
   use_mag = LaunchConfiguration('use_mag', default='False')

   ARGUMENTS = [
      DeclareLaunchArgument('slam_param_files', default_value=PathJoinSubstitution([FindPackageShare('skd1_navigation'), 'config', 'mapper_params_online_async.yaml']),
            description='Full path to param yaml file to load for SLAM Online Async'),
      DeclareLaunchArgument('nav2_params_file', default_value=PathJoinSubstitution([FindPackageShare('skd1_navigation'), 'config', 'nav2_params.yaml']),
            description='Full path to param yaml file to load for Nav2'),
      # DeclareLaunchArgument('amcl_params_file', default_value=PathJoinSubstitution([FindPackageShare('skd1_navigation'), 'config', 'nav2_params.yaml']),
      #       description='Full path to param yaml file to load for AMCL'),
      # DeclareLaunchArgument('amcl_map_file', default_value=PathJoinSubstitution([FindPackageShare('skd1_navigation'), 'maps', 'turtlebot3_world.yaml']),
      #       description='Full path to map yaml file to load for AMCL')
   ]

   slam_params_file = LaunchConfiguration('slam_param_files')
   nav2_params_file = LaunchConfiguration('nav2_params_file')
   # amcl_params_file = LaunchConfiguration('amcl_params_file')
   # amcl_map_file = LaunchConfiguration('amcl_map_file')

   # Cargar:
   # - skd1_description (XACRO > robot_state_publisher)
   # - skd1_control = joint_state_broadcaster + skd1_base_controller
   skd1_control_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("skd1_control"),
            "launch",
            "skd1_control_launch.py"],
         )
      ),
   )

   teleop_joy_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("skd1_teleop"),
             "launch",
             "skd1_teleop_twist_joy_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   teleop_twist_mux_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("skd1_teleop"),
             "launch",
             "skd1_twist_mux_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   phidgets_spatial_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("phidgets_spatial_bringup"),
             "launch",
             "spatial_imu_filter_component_launch.py"],
         )
      ),
      launch_arguments= {'use_mag': use_mag}.items(),
   )

   ekf_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("skd1_localization"),
             "launch",
             "ekf_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   rplidar_c1_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("rplidar_c1_bringup"),
             "launch",
             "sllidar_c1_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   range_filter_laser_filter = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("rplidar_c1_bringup"),
             "launch",
             "range_filter_laser_filter_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   # nav2_amcl_localization_launch = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource(
   #       PathJoinSubstitution(
   #          [FindPackageShare("nav2_bringup"),
   #           "launch",
   #           "localization_launch.py"],
   #       )
   #    ),
   #    launch_arguments= {'use_sim_time': use_sim_time,
   #                      'params_file': amcl_params_file,
   #                      'map': amcl_map_file}.items(),
   # )

   online_async_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
         PathJoinSubstitution(
            [FindPackageShare("slam_toolbox"),
               "launch",
               "online_async_launch.py"],
         )
      ),
      launch_arguments= {'use_sim_time': use_sim_time,
                         'slam_params_file': slam_params_file}.items(),
      # launch_arguments= {'use_sim_time': use_sim_time}.items(),
   )

   nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('skd1_navigation'),
               'launch',
               'navigation_launch.py']),
        ),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'params_file': nav2_params_file}.items()
    )

   ld = LaunchDescription(ARGUMENTS)
   ld.add_action(skd1_control_launch)
   ld.add_action(teleop_twist_mux_launch)
   ld.add_action(teleop_joy_launch)
   ld.add_action(phidgets_spatial_launch)
   ld.add_action(rplidar_c1_launch)
   ld.add_action(range_filter_laser_filter)
   ld.add_action(ekf_launch)
   ld.add_action(online_async_launch)
   # # ld.add_action(nav2_amcl_localization_launch)
   ld.add_action(nav2_launch)

   return ld
