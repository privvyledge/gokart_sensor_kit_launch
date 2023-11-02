"""
Launches a single LIVOX HAP LIDAR.

Todo:
    * Pass the publish_frequency as a parameter. Test
    * Pass the frame_id as a parameter. Test
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 50.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'lidar_livox_hap_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    package_directory = get_package_share_directory('sensor_kit_launch')

    lidar_config = os.path.join(
        package_directory, 'config', 'lidar_livox_hap_config.json'
    )

    publish_freq_la = DeclareLaunchArgument('lidar_publish_freq',
                                            default_value=publish_freq,
                                            description='LIDAR publish frequency. 5.0, 10.0, 20.0, 50.0, etc. Max 100.')

    lidar_frame_id_la = DeclareLaunchArgument('lidar_publish_frame_id',
                                              default_value=frame_id,
                                              description='Frame ID to publish pointclouds and IMU.')
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            livox_ros2_params,
            # Override parameters
            {'publish_freq': publish_freq_la,
             'frame_id': lidar_frame_id_la}
        ]
        )

    return LaunchDescription([
        publish_freq_la,
        lidar_frame_id_la,
        livox_driver,
    ])

