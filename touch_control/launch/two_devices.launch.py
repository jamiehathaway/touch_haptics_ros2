import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='touch_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # prefix=['gdbserver localhost:3000'],
        composable_node_descriptions=[
            ComposableNode(
                package='touch_control',
                plugin='touch_control::TouchControlNode',
                name='touch_control',
                namespace='touch',
                parameters=[{
                    'device_names': ['left', 'right'],  # Device name (if left blank the "Default Device" is used)
                    'prefixes': ['touch_left_', 'touch_right_'],  # Joint names prefix
                    # 'scheduler_rate': 1000,  # Can be 500 or 1000 Hz (default: 1000)
                    # 'publish_joint_states': True,  # Publishes joint states if True (default: False)
                }],
                # extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
