# Copyright (c) 2019 Intel Corporation
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

# /* Author: Gary Liu */

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # config the serial number and base frame id of each camera
    #t265_base_frame_id = LaunchConfiguration('base_frame_id', default='t265_link')
    #t265_base_frame_id = LaunchConfiguration('base_frame_id', default='odom_frame')
    t265_base_frame_id = LaunchConfiguration('base_frame_id', default='camera_pose_optical_frame')
    t265_serial_no = LaunchConfiguration('serial_no', default='948422110819')

    rgbd_base_frame_id = LaunchConfiguration('base_frame_id', default='d435_link')
    rgbd_serial_no = LaunchConfiguration('serial_no', default='834412071337')

    rviz_config_dir = os.path.join(get_package_share_directory('realsense_examples'), 'config', 'rs_cartographer_2.rviz')
    costmap_dir = os.path.join('/home/gewenhao/ros2_ws/src/realsense_image_raw/config','costmap.yaml')

    rviz_node = Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output = 'screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': 'false'}]
            )
    tf_node = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.03', '0', '0', '0', t265_base_frame_id, rgbd_base_frame_id]
            )

    # costmap_node = Node(
    #        node_namespace='/costmap_node',
    #        package='nav2_costmap_2d',
    #        node_executable='nav2_costmap_2d',
    #        arguments=['-d', costmap_dir],
    #        output='screen',
    #        parameters=[{'use_sim_time': 'true'}]
    #        )

    #tf2_node = Node(
    #        package='tf2_ros',
    #        node_executable='static_transform_publisher',
    #        output='screen',
    #        arguments=['0', '0', '0.03', '0', '0', '0', 'camera_pose_optical_frame', rgbd_base_frame_id]
    #       )
    #t265_node = Node(
    #   package='realsense_node',
    #    node_executable='realsense_node',
    #    node_namespace="/t265",
    #    output='screen',
    #    remappings=[('/t265/camera/odom/sample','/odom')],
    #    parameters=[{'serial_no':t265_serial_no ,
    #            'base_frame_id': t265_base_frame_id}]
    #    )
    t265_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/t265",
        #output='screen',
        remappings=[('/t265/camera/odom/sample', '/odom')],
        parameters=[{'serial_no': t265_serial_no,
                     'base_frame_id': t265_base_frame_id}]
    )
    
    rgbd_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/d435",
        #output='screen',
        parameters=[{'serial_no': rgbd_serial_no,
                     'base_frame_id': rgbd_base_frame_id,
                     'enable_pointcloud': 'true',
                     'dense_pointcloud': 'false'}]
    )

    map_process_node = Node(
        package='realsense_image_raw',
        node_executable='map_process',
        node_name='map_process',
        output='screen')

    # scan_node = Node(
    #         package='depthimage_to_laserscan',
    #         node_executable='depthimage_to_laserscan_node',
    #         node_name='scan',
    #         output='screen',
    #         parameters=[{'output_frame':'d435_link'}],
    #         remappings=[('depth','/d435/camera/depth/image_rect_raw'),
    #                     ('depth_camera_info', '/d435/camera/depth/camera_info')],
    #         )
    #return launch.LaunchDescription([map_process_node])
    return launch.LaunchDescription([rviz_node, tf_node, t265_node, rgbd_node, map_process_node])
    #return launch.LaunchDescription([rviz_node, tf_node, t265_node, rgbd_node, scan_node])
