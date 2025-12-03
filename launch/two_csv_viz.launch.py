from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    csv1 = LaunchConfiguration('csv1')
    csv2 = LaunchConfiguration('csv2')
    stride = LaunchConfiguration('stride')

    return LaunchDescription([
        DeclareLaunchArgument('csv1', default_value=''),
        DeclareLaunchArgument('csv2', default_value=''),
        DeclareLaunchArgument('stride', default_value='20'),

        # Trajectory 1
        Node(
            package='pose_pubsub',
            executable='csv_pose_player',
            namespace='traj1',
            parameters=[{'file_path': csv1, 'pose_topic': 'poses', 'frame_id': 'map', 'publish_rate': 30.0}]
        ),
        Node(
            package='pose_pubsub',
            executable='pose_to_path',
            namespace='traj1',
            parameters=[{'pose_topic': 'poses', 'axis_stride': stride, 'frame_id': 'map'}]
        ),

        # Trajectory 2
        Node(
            package='pose_pubsub',
            executable='csv_pose_player',
            namespace='traj2',
            parameters=[{'file_path': csv2, 'pose_topic': 'poses', 'frame_id': 'map', 'publish_rate': 30.0}]
        ),
        Node(
            package='pose_pubsub',
            executable='pose_to_path',
            namespace='traj2',
            parameters=[{'pose_topic': 'poses', 'axis_stride': stride, 'frame_id': 'map'}]
        ),
    ])