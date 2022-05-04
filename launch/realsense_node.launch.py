import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    realsense_camera_node = Node(
        package='realsense2_camera',
        node_executable='realsense2_camera_node',
        namespace='camera',
        parameters=[{
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'color_width': 640,
                'color_height': 480,
                'enable_depth': False,
                'depth_width': 640,
                'depth_height': 480,
                'align_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.enable_auto_exposure': False,
                'depth_module.exposure': 18500,
                'depth_module.gain': 100,
                'infra_fps': 30.0,
                'color_fps': 30.0,
                'depth_fps': 30.0
        }]
    )
    ## Nodes to launch

    return LaunchDescription([realsense_camera_node])
