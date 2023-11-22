#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    #
    # yolo
    #
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8m.pt",
        description="Model name or path")

    tracker = LaunchConfiguration("tracker")
    tracker_cmd = DeclareLaunchArgument(
        "tracker",
        default_value="bytetrack.yaml",
        description="Tracker name or path")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)")

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Wheter to start darknet enabled")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/kitti/camera_color_left/image_raw",
        description="Name of the input image topic")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes")

    #
    # NODES
    #
    detector_node_cmd = Node(
        package="yolov8_ros",
        executable="yolov8_node",
        name="yolov8_node",
        namespace=namespace,
        parameters=[{"model": model,
                     "device": device,
                     "enable": enable,
                     "threshold": threshold}],
        remappings=[("image_raw", input_image_topic)]
    )

    tracking_node_cmd = Node(
        package="yolov8_ros",
        executable="tracking_node",
        name="tracking_node",
        namespace=namespace,
        parameters=[{"tracker": tracker}],
        remappings=[("image_raw", input_image_topic)]
    )

    debug_node_cmd = Node(
        package="yolov8_ros",
        executable="debug_node",
        name="debug_node",
        namespace=namespace,
        remappings=[("image_raw", input_image_topic),
                    ("detections", "tracking")]
    )

    ld = LaunchDescription()

    ld.add_action(model_cmd)
    ld.add_action(tracker_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detector_node_cmd)
    ld.add_action(tracking_node_cmd)
    ld.add_action(debug_node_cmd)

    lidar_detection_node = Node(
        package='lidar_detection',
        executable='lidar_detection_node',
        parameters=[],
        arguments=[],
        output="screen",
    )

    preprocessing_node = Node(
        package='preprocessing',
        executable='preprocessing_node',
        parameters=[],
        arguments=[],
        output="screen",
    )

    lidar_clustering_node = Node(
        package='lidar_clustering',
        executable='lidar_clustering_node',
        parameters=[],
        arguments=[],
        output="screen",
    )

    lidar_detection_node = Node(
        package='lidar_detection',
        executable='lidar_detection_node',
        parameters=[],
        arguments=[],
        output="screen",
    )

    comparison_node = Node(
        package='comparison',
        executable='comparison_node',
        parameters=[],
        arguments=[],
        output="screen",
    )

    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', [os.path.join(pkg_dir, 'config', 'config_file.rviz')]]
        )

    # create and return launch description object
    return LaunchDescription(
        [
            ld,
            preprocessing_node,
            lidar_clustering_node,
            lidar_detection_node,
            comparison_node,
            # rviz_node,
        ]
    )