"""
Source: https://github.com/stereolabs/zed-ros2-wrapper/tree/master/zed_wrapper/launch
"""

# Copyright 2022 Stereolabs
#
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Camera model (force value). Todo: load as launch arguments or use the config file instead
    camera_name = 'front'
    camera_model = 'zed2i'

    ros_params_override_path = os.path.join(
        get_package_share_directory('common_sensor_launch'),
        'config',
        'camera_' + camera_model + '.yaml'
    )

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory('zed_wrapper'),
                '/launch/include/zed_camera.launch.py'
            ]),
            launch_arguments={
                'camera_name': camera_name,
                'camera_model': camera_model,
                'ros_params_override_path': ros_params_override_path,
            }.items()
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)

    return ld
