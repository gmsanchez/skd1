# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    skd1_localization_dir = get_package_share_directory('skd1_localization')

    parameters_file_dir = os.path.join(skd1_localization_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_output_final_position = DeclareLaunchArgument(
        'output_final_position',
        default_value='false')

    declare_output_location = DeclareLaunchArgument(
        'output_location',
        default_value=os.path.expanduser('~/dual_ekf_navsat_example_debug.txt'))

    gps_gyro_heading_node = launch_ros.actions.Node(
        package='outdoor_nav_tools',
        executable='gps_gyro_heading_node',
        name='gps_gyro_heading_node',
        output='screen',
        parameters=[{
            'gps_topic': 'ardusimple/fix',
            'imu_topic': 'phidgets_373223/imu/data',
            'min_gps_distance': 0.3,
            'complementary_alpha': 0.98,
            'frame_id' : 'phidgets_imu_link',
            'use_sim_time': use_sim_time,
        }],
    )
    
    gps_only_heading_node = launch_ros.actions.Node(
        package='outdoor_nav_tools',
        executable='gps_only_heading_node',
        name='gps_only_heading_node',
        output='screen',
        parameters=[{
            'gps_topic': 'ardusimple/fix',
            'min_gps_distance': 0.3,
            'frame_id' : 'phidgets_imu_link',
            'use_sim_time': use_sim_time,
        }],
    )

    ekf_filter_node_odom = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, parameters_file_path],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    ekf_filter_node_map = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, parameters_file_path],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, parameters_file_path],
        remappings=
            [
                # ('imu', 'phidgets_373223/imu/data'),
                ('imu', 'gps_gyro_heading_node/heading_imu'),
                # ('imu', 'gps_only_heading_node/heading_imu'),
                ('gps/fix', 'ardusimple/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global')
            ]
    )
    

    return LaunchDescription([
        declare_use_sim_time,
        declare_output_final_position,
        declare_output_location,
        gps_gyro_heading_node,
        gps_only_heading_node,
        ekf_filter_node_odom,
        ekf_filter_node_map,
        navsat_transform_node,
    ])