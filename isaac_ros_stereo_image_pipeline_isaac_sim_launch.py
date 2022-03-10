# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    left_image_format_converter_node = ComposableNode(
        name='left_image_format_converter_node',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ImageFormatConverterNode',
        '''remappings=[('/image_raw', '/rgb_left'),
                    ('/image', '/stereo_camera/left/image')],'''
        remappings=[('/image_raw', '/rgb_left/resized'),
                    ('/image', '/stereo_camera/left/image')],
        parameters=[{'encoding_desired': 'mono8'}]
    )

    right_image_format_converter_node = ComposableNode(
        name='right_image_format_converter_node',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ImageFormatConverterNode',
        '''remappings=[('/image_raw', '/rgb_right'),
                    ('/image', '/stereo_camera/right/image')],'''
        remappings=[('/image_raw', '/rgb_right/resized'),
                    ('/image', '/stereo_camera/right/image')],
        parameters=[{'encoding_desired': 'mono8'}]
    )

    left_resize_node = ComposableNode(
        #namespace='',
        name='left_isaac_ros_resize',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'use_relative_scale': False,
            'height': 352,
            'width': 640,
        }],
        remappings=[('/image', '/rgb_left'), ('/camera_info', '/camera_info_left'), ('/resized/image', '/rgb_left/resized'), ('/resized/camera_info', '/camera_info_left/resized')]
    )

    right_resize_node = ComposableNode(
        #namespace='',
        name='right_isaac_ros_resize',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'use_relative_scale': False,
            'height': 352,
            'width': 640,
        }],
        remappings=[('/image', '/rgb_right'), ('/camera_info', '/camera_info_right'), ('/resized/image', '/rgb_right/resized'), ('/resized/camera_info', '/camera_info_right/resized')]
    )


    disparity_node = ComposableNode(
        name='disparity',
        package='isaac_ros_stereo_image_proc',
        plugin='isaac_ros::stereo_image_proc::DisparityNode',
        parameters=[{
                'backends': 'CUDA',
                'scale': 1.0 / 32.0,
                'max_disparity': 256,
        }],
        remappings=[('/left/image_rect', '/stereo_camera/left/image'),
                    ('/left/camera_info', '/camera_info_left/resized'),
                    ('/right/image_rect', '/stereo_camera/right/image'),
                    ('/right/camera_info', '/camera_info_right/resized')])

    pointcloud_node = ComposableNode(
        package='isaac_ros_stereo_image_proc',
        plugin='isaac_ros::stereo_image_proc::PointCloudNode',
        parameters=[{
                'use_color': True,
        }],
        remappings=[('/left/image_rect_color', '/rgb_left/resized'),
                    ('/left/camera_info', '/camera_info_left/resized'),
                    ('/right/camera_info', '/camera_info_right/resized')])

    container = ComposableNodeContainer(
        name='disparity_container',
        namespace='disparity',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[disparity_node, pointcloud_node,
                                      left_image_format_converter_node,
                                      right_image_format_converter_node,
                                      left_resize_node, right_resize_node],
        output='screen'
    )

    return (launch.LaunchDescription([container]))
