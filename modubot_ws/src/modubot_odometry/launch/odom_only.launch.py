from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modubot_odometry',
            executable='serial_odometry_node',
            name='serial_odometry_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'ticks_per_rev': 90.0,
                'wheel_radius': 0.05,
                'wheel_separation': 0.28,
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'publish_tf': True,
            }],
        ),
        # opcional: RViz com config
        # Node(package='rviz2', executable='rviz2', arguments=['-d', '/path/odom_view.rviz']),
    ])

