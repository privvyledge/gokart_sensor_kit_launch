"""
Launches a single LIVOX HAP LIDAR.

Todo:
    * pass configuration options as parameters
    * Pass the publish_frequency as a parameter. Test
    * Pass the frame_id as a parameter. Test
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import launch

################### user configure parameters for ros2 start ###################
xfer_format = 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src = 0  # 0-lidar, others-Invalid data src
publish_freq = 50.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type = 0
frame_id = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'


def generate_launch_description():
    package_directory = get_package_share_directory('common_sensor_launch')

    lidar_config = os.path.join(
            package_directory, 'config', 'lidar_livox_hap_config.json'
    )

    publish_freq_la = DeclareLaunchArgument('publish_freq_la',
                                            default_value=str(publish_freq),
                                            description='LIDAR publish frequency. 5.0, 10.0, 20.0, 50.0, etc. Max 100.')

    lidar_frame_id_la = DeclareLaunchArgument('lidar_publish_frame_id',
                                              default_value=frame_id,
                                              description='Frame ID to publish pointclouds and IMU.')

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": lidar_config},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    # lidar_publish_freq = LaunchConfiguration('lidar_publish_freq')
    # lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    #
    # lidar_publish_freq_la = DeclareLaunchArgument('lidar_publish_freq',
    #                                               default_value=TextSubstitution(text=str(publish_freq)),
    #                                               description='LIDAR publish frequency. 5.0, 10.0, 20.0, 50.0, etc. Max 100.')
    #
    # lidar_frame_id_la = DeclareLaunchArgument('lidar_frame_id',
    #                                           default_value=frame_id,
    #                                           description='Frame ID to publish pointclouds and IMU.')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        )

    return LaunchDescription([
        publish_freq_la,
        lidar_frame_id_la,
        livox_driver,
    ])
