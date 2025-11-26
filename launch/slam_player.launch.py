from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    csv_arg = DeclareLaunchArgument('csv', default_value='')
    rviz_config = os.path.join(
        get_package_share_directory('pose_pubsub'),
        'config', 'rviz', 'slam_player.rviz'
    )
    return LaunchDescription([
        csv_arg,
        Node(
            package='pose_pubsub',
            executable='pose_publisher',
            name='pose_publisher',
            arguments=[LaunchConfiguration('csv')],  # positional CSV
            parameters=[{
                'frame_id': 'map',
                'topic_poses': '/poses',
                'topic_path': '/poses_path',
                'loop': True,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])