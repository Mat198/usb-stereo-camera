from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def stereo_pipeline(context):

    left_calibration_yaml = PathJoinSubstitution([
        FindPackageShare('usb_stereo_camera_driver'), 'config', 'stereo_left_camera_calibration.yaml'
    ])

    right_calibration_yaml = PathJoinSubstitution([
        FindPackageShare('usb_stereo_camera_driver'), 'config', 'stereo_right_camera_calibration.yaml'
    ])

    resolution = [
        int(LaunchConfiguration('resolution_width').perform(context)),
        int(LaunchConfiguration('resolution_height').perform(context))
    ]

    disparity_parameters_yaml = PathJoinSubstitution([
        FindPackageShare('usb_stereo_camera_driver'), 'config', 'disparity_parameters.yaml'
    ])

    stereo_pipeline_container = ComposableNodeContainer(
        name='stereo_pipeline',
        package='rclcpp_components',
        namespace="",
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_stereo_camera_driver',
                plugin='stereoCamera::UsbStereoCameraDriver',
                name='usb_stereo_camera_driver',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[
                    left_calibration_yaml,
                    right_calibration_yaml,
                    {
                        'camera_port': LaunchConfiguration('camera_port'),
                        'fps': LaunchConfiguration('fps'),
                        'resolution': resolution,
                        'left_camera': {'name': LaunchConfiguration('left_camera_name')},
                        'right_camera': {'name': LaunchConfiguration('right_camera_name')}
                    }
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='left_rectify_node',
                namespace=LaunchConfiguration('left_camera_name'),
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='right_rectify_node',
                namespace=LaunchConfiguration('right_camera_name'),
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                remappings=[
                    ('left/image_rect', [LaunchConfiguration('left_camera_name'), '/image_rect']),
                    ('left/camera_info', [LaunchConfiguration('left_camera_name'), '/camera_info']),
                    ('right/image_rect', [LaunchConfiguration('right_camera_name'), '/image_rect']),
                    ('right/camera_info', [LaunchConfiguration('right_camera_name'), '/camera_info']),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[disparity_parameters_yaml]
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                remappings=[
                    ('disparity', 'disparity'),
                    ('left/image_rect_color', [LaunchConfiguration('left_camera_name'), '/image_rect_color']),
                    ('left/camera_info', [LaunchConfiguration('left_camera_name'), '/camera_info']),
                    ('right/camera_info', [LaunchConfiguration('right_camera_name'), '/camera_info']),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[
                    {'approximate_sync': False},
                    {'use_color': True},
                    {'avoid_point_cloud_padding': False},
                ]
            ),
        ]
    )
    return [stereo_pipeline_container]

def generate_launch_description():

    camera_port_arg = DeclareLaunchArgument(
        'camera_port', default_value='0',
        description='USB port number for the stereo camera.'
    )

    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30',
        description='Frames per second for the stereo camera.'
    )

    resolution_width_arg = DeclareLaunchArgument(
        'resolution_width', default_value='1280',
        description='Output camera width in pixels.'
    )

    resolution_height_arg = DeclareLaunchArgument(
        'resolution_height', default_value='720',
        description='Output camera height in pixels.'
    )

    left_camera_name_arg = DeclareLaunchArgument(
        'left_camera_name', default_value='left_camera',
        description='Name of the left camera used for camera info frame_id.'
    )

    right_camera_name_arg = DeclareLaunchArgument(
        'right_camera_name', default_value='right_camera',
        description='Name of the right camera used for camera info frame_id.'
    )

    return LaunchDescription([
        camera_port_arg,
        fps_arg,
        resolution_width_arg,
        resolution_height_arg,
        left_camera_name_arg,
        right_camera_name_arg,

        OpaqueFunction(function=stereo_pipeline),
    ])
