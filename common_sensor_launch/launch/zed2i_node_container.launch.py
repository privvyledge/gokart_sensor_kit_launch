#!/usr/bin/env python3
"""
See https://github.com/stereolabs/zed-ros2-wrapper/issues/126
"""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'common.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
)


def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    arr = str.split(',')

    return arr


def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    """
    Zed configuration setup
    """
    wrapper_dir = get_package_share_directory('zed_wrapper')

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    node_name = LaunchConfiguration('node_name')

    config_common_path = LaunchConfiguration('config_path')

    zed_id = LaunchConfiguration('zed_id')
    serial_number = LaunchConfiguration('serial_number')

    base_frame = LaunchConfiguration('base_frame')
    cam_pose = LaunchConfiguration('cam_pose')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    ros_params_override_path = LaunchConfiguration('ros_params_override_path')

    gnss_fusion_enabled = LaunchConfiguration('gnss_fusion_enabled')
    gnss_fix_topic = LaunchConfiguration('gnss_fix_topic')
    gnss_frame = LaunchConfiguration('gnss_frame')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)

    if (camera_name_val == ""):
        camera_name_val = camera_model_val

    config_camera_path = os.path.join(
            get_package_share_directory('zed_wrapper'),
            'config',
            camera_model_val + '.yaml'
    )

    # Convert 'cam_pose' parameter
    cam_pose_str = cam_pose.perform(context)
    cam_pose_array = parse_array_param(cam_pose_str)

    nodes = []

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    ego_box_crop_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", camera_name_val+"/point_cloud/cloud_registered"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    mirror_info = get_vehicle_mirror_info(context)
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    mirror_box_crop_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    pointcloud_interpolator_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    ring_outlier_filter_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "outlier_filtered/pointcloud"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    # todo: add more filters
    nodes.append(ego_box_crop_node)
    nodes.append(mirror_box_crop_node)  # todo: might remove
    nodes.append(pointcloud_interpolator_node)  # todo: might remove
    # nodes.append(ring_outlier_filter_node)  # todo: remove

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    component_loader = LoadComposableNodes(
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    # Start the ZED driver.
    # Robot State Publisher node
    rsp_node = Node(
            condition=IfCondition(publish_urdf),
            package='robot_state_publisher',
            # plugin="robot_state_publisher::RobotStatePublisher",
            namespace=camera_name_val,
            executable='robot_state_publisher',
            name='zed_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(
                        [
                            'xacro', ' ', xacro_path, ' ',
                            'camera_name:=', camera_name_val, ' ',
                            'camera_model:=', camera_model_val, ' ',
                            'base_frame:=', base_frame, ' ',
                            'gnss_frame:=', gnss_frame, ' ',
                            'cam_pos_x:=', cam_pose_array[0], ' ',
                            'cam_pos_y:=', cam_pose_array[1], ' ',
                            'cam_pos_z:=', cam_pose_array[2], ' ',
                            'cam_roll:=', cam_pose_array[3], ' ',
                            'cam_pitch:=', cam_pose_array[4], ' ',
                            'cam_yaw:=', cam_pose_array[5]
                        ])
            }]
    )

    driver_component = ComposableNode(
        package="zed_components",
        plugin="stereolabs::ZedCamera",
        # node is created in a global context, need to avoid name clash
        name=camera_name_val,
        parameters=[
                # YAML files
                config_common_path,  # Common parameters
                config_camera_path,  # Camera related parameters
                # Overriding
                {
                    'general.camera_name': camera_name_val,
                    'general.camera_model': camera_model_val,
                    'general.svo_file': svo_path,
                    # 'depth.quality': 0,
                    'pos_tracking.base_frame': base_frame,
                    'general.zed_id': zed_id,
                    'general.serial_number': serial_number,
                    'pos_tracking.publish_tf': publish_tf,
                    'pos_tracking.publish_map_tf': publish_map_tf,
                    'sensors.publish_imu_tf': publish_imu_tf,
                    'gnss_fusion.gnss_fusion_enabled': gnss_fusion_enabled,
                    'gnss_fusion.gnss_fix_topic': gnss_fix_topic,
                    'gnss_fusion.gnss_frame': gnss_frame,
                },
                ros_params_override_path,
            ],

        # remappings=[
        #     (camera_name_val+"/point_cloud/cloud_registered", "pointcloud_raw_ex"),
        #     # ("/livox/points_ex", "pointcloud_raw_ex"),
        # ],
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    driver_component_loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component],
        target_container=target_container,
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    return [container, component_loader, driver_component_loader, rsp_node]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # Common launch arguments
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")

    # Zed launch arguments
    add_launch_arg("camera_name", default_value=TextSubstitution(text=""),
                   description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`. Leave empty to use the camera model as camera name.')
    add_launch_arg("camera_model", default_value="zed2i", description="The model of the camera. Using a wrong camera model can disable camera features. Valid models: `zed`, `zedm`, `zed2`, `zed2i`.")
    add_launch_arg('node_name', default_value='zed_node',
                description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`')
    add_launch_arg("config_path", default_value=TextSubstitution(text=default_config_common), description="Path to the YAML configuration file for the camera.")

    add_launch_arg("base_frame", "base_link", "Name of the base link frame.")
    # add_launch_arg("frame_id", "zed_link", "frame id")  # change in config file

    add_launch_arg(
            'zed_id',
            default_value='0',
            description='The index of the camera to be opened. To be used in multi-camera rigs.')
    add_launch_arg(
            'serial_number',
            default_value='0',
            description='The serial number of the camera to be opened. To be used in multi-camera rigs. Has priority with respect to `zed_id`.')
    add_launch_arg(
            'publish_urdf',
            default_value='true',
            description='Enable URDF processing and starts Robot State Published to propagate static TF.')
    add_launch_arg(
            'publish_tf',
            default_value='true',
            description='Enable publication of the `odom -> base_link` TF.')
    add_launch_arg(
            'publish_map_tf',
            default_value='true',
            description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.')
    add_launch_arg(
            'publish_imu_tf',
            default_value='true',
            description='Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.')
    add_launch_arg(
            'xacro_path',
            default_value=TextSubstitution(text=default_xacro_path),
            description='Path to the camera URDF file as a xacro file.')
    add_launch_arg(
            'ros_params_override_path',
            default_value='',
            description='The path to an additional parameters file to override the defaults')
    add_launch_arg(
            'svo_path',
            default_value=TextSubstitution(text="live"),
            description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.')
    add_launch_arg(
            'base_frame',
            default_value='base_link',
            description='Name of the base link frame.')
    add_launch_arg(
            'gnss_fusion_enabled',
            default_value='true',
            description='Whether to fuse "sensor_msg/NavSatFix" message information into pose data')
    add_launch_arg(
            'gnss_fix_topic',
            default_value='',
            description='Name of the GNSS topic of type NavSatFix to subscribe [Default: "/gps/fix"]')

    add_launch_arg(
            'gnss_frame',
            default_value='',
            description='Name of the GNSS link frame. Leave empty if not used. Remember to set the transform `base_link` -> `gnss_frame` in the URDF file.')

    add_launch_arg(
            'cam_pose',
            default_value='[0.0,0.0,0.0,0.0,0.0,0.0]',
            description='Pose of the camera with respect to the base frame (i.e. `base_link`): [x,y,z,r,p,y]. Note: Orientation in rad.)')

    # Autoware launch arguments
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("setup_sensor", "True", "configure sensor")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg(
            "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )

    add_launch_arg("use_pointcloud_container", "false")
    add_launch_arg("container_name", "zed_node_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        [SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1')]
        + launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
