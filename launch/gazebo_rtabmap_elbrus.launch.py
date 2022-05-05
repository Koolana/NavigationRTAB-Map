import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time': True,
          'subscribe_depth':False,
          'subscribe_stereo': True,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'Grid/RayTracing': 'true',
          'Grid/3D': 'true',
          'RGBD/NeighborLinkRefining':'True',
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    # remappings=[
    #       ('rgb/image', '/camera/color/image_raw'),
    #       ('rgb/camera_info', '/camera/color/camera_info'),
    #       ('depth/image', '/camera/depth/image_rect_raw')]

    remappings=[
          ('left/image_rect', 'infra1/image_raw'),
          ('right/image_rect', 'infra2/image_raw'),
          ('left/camera_info', 'infra1/camera_info'),
          ('right/camera_info', 'infra2/camera_info')]

    urdf_file_name = 'urdf/newModel.xml'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('navigation_rtabmap'),
        urdf_file_name)

    # Launch arguments
    arg_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    arg_qos = DeclareLaunchArgument(
        'qos', default_value='2',
        description='QoS used for input sensor topics')

    arg_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Launch in localization mode.')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
        )
    ## Nodes to launch

    # SLAM mode:
    rtabmap_node = Node(
        name='rtabmap_node',
        condition=UnlessCondition(localization),
        package='rtabmap_ros', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d']) # This will delete the previous database (~/.ros/rtabmap.db)

    # Localization mode:
    rtabmap_node_localization = Node(
        name='rtabmap_node',
        condition=IfCondition(localization),
        package='rtabmap_ros', executable='rtabmap', output='screen',
        parameters=[parameters,
          {'Mem/IncrementalMemory':'False',
           'Mem/InitWMWithAllNodes':'True'}],
        remappings=remappings)

    rtabmap_viz_node = Node(
        name='rtabmap_viz_node',
        package='rtabmap_ros', executable='rtabmapviz', output='screen',
        parameters=[parameters],
        remappings=remappings)

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'use_sim_time': True,
                    'enable_rectified_pose': True,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'enable_localization_n_mapping': False,
                    'input_left_camera_frame': 'camera_infra1_frame',
                    'input_right_camera_frame': 'camera_infra2_frame'
                    }],

        remappings=[('stereo_camera/left/image', 'infra1/image_raw'),
                    ('stereo_camera/left/camera_info', 'infra1/camera_info'),
                    ('stereo_camera/right/image', 'infra2/image_raw'),
                    ('stereo_camera/right/camera_info', 'infra2/camera_info'),
                    ('visual_slam/tracking/odometry', 'odom')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    return LaunchDescription([arg_sim, arg_qos, arg_localization,
                              robot_state_publisher_node, joint_state_publisher_node,
                              rtabmap_node, rtabmap_node_localization,
                              visual_slam_launch_container,
                              rtabmap_viz_node])
