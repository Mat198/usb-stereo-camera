from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    video_port_arg = DeclareLaunchArgument('video_port', default_value='0')

    stereo_pipeline = ComposableNodeContainer(
        name='stereo_pipeline',
        package='rclcpp_components',
        namespace="",
        executable='component_container',
        
        composable_node_descriptions=[
            ComposableNode(
                package='usb_stereo_camera_driver',
                plugin='stereoCamera::UsbStereoCameraDriver',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{
                    'video_port': LaunchConfiguration('video_port')
                }],
            ),
        ]
    )
    return LaunchDescription([
        video_port_arg,
        stereo_pipeline
    ])
