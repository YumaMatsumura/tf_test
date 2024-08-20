#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Set launch params
    use_composition = LaunchConfiguration('use_composition')
    use_namespace = LaunchConfiguration('use_namespace')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed nodes',
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to use namespace',
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    # Load nodes
    load_nodes = GroupAction(
        [
            PushROSNamespace(condition=IfCondition(use_namespace), namespace='tf_test'),
            Node(
                name='tf2_ros',
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '0.5', '0', '0', '0', '0', '0', 'base_link', 'lidar_link',
                ],
                remappings=remappings,
                output='screen',
            ),
            Node(
                condition=UnlessCondition(use_composition),
                name='tf_listener_test_node',
                package='tf_test',
                executable='tf_listener_test',
                remappings=remappings,
                output='screen',
            ),
            ComposableNodeContainer(
                condition=IfCondition(use_composition),
                name='tf_listener_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        name='tf_listener_test_node',
                        package='tf_test',
                        plugin='tf_test::TfListenerTest',
                        remappings=remappings,
                    ),
                ],
                output='screen',
            ),
        ]
    )

    return LaunchDescription(
        [
            declare_use_composition_cmd,
            declare_use_namespace_cmd,
            load_nodes,
        ]
    )
