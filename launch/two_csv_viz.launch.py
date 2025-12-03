from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    csv1 = LaunchConfiguration('csv1')
    csv2 = LaunchConfiguration('csv2')
    stride = LaunchConfiguration('stride')

    # NEW: per-stream start delays
    start_delay_traj1 = LaunchConfiguration('start_delay_traj1')
    start_delay_traj2 = LaunchConfiguration('start_delay_traj2')

    return LaunchDescription([
        DeclareLaunchArgument('csv1'),
        DeclareLaunchArgument('csv2'),
        DeclareLaunchArgument('stride', default_value='1'),
        DeclareLaunchArgument('start_delay_traj1', default_value='0.0'),
        DeclareLaunchArgument('start_delay_traj2', default_value='8.43'),

        Node(
            package='pose_pubsub',
            executable='csv_pose_player',
            namespace='traj1',
            parameters=[{
                'file_path': csv1,
                'pose_topic': 'poses',
                'frame_id': 'map',
                'start_delay_sec': start_delay_traj1,
            }],
        ),
        Node(
            package='pose_pubsub',
            executable='pose_to_path',
            namespace='traj1',
            parameters=[{
                'pose_topic': 'poses',
                'path_topic': 'poses_path_full',
                'frame_id': 'map',
            }],
        ),

        Node(
            package='pose_pubsub',
            executable='csv_pose_player',
            namespace='traj2',
            parameters=[{
                'file_path': csv2,
                'pose_topic': 'poses',
                'frame_id': 'map',
                'start_delay_sec': start_delay_traj2,
            }],
        ),
        Node(
            package='pose_pubsub',
            executable='pose_to_path',
            namespace='traj2',
            parameters=[{
                'pose_topic': 'poses',
                'path_topic': 'poses_path_full',
                'frame_id': 'map',
            }],
        ),
    ])