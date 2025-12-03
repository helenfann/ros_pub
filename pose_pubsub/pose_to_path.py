# Usage:
# ros2 run pose_pubsub csv_pose_player --ros-args \
#   -r __ns:=/traj2 \
#   -p file_path:=/home/helen/ros2_ws/src/pose_pubsub/data/all_gt_poses_aria_new.csv \
#   -p pose_topic:=poses -p frame_id:=map -p loop:=False
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseToPath(Node):
    def __init__(self):
        super().__init__('pose_to_path')
        self.declare_parameter('pose_topic', 'poses')                 # input PoseStamped
        self.declare_parameter('path_topic', 'poses_path_full')       # full path
        self.declare_parameter('path_topic_stride', 'poses_path_stride')  # stride path
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('stride', 10)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.path_topic_stride = self.get_parameter('path_topic_stride').value
        self.frame_id = self.get_parameter('frame_id').value
        self.stride = int(self.get_parameter('stride').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, qos)
        self.pub_full = self.create_publisher(Path, self.path_topic, qos)
        self.pub_stride = self.create_publisher(Path, self.path_topic_stride, qos)

        self.path = Path(); self.path.header.frame_id = self.frame_id
        self.path_stride = Path(); self.path_stride.header.frame_id = self.frame_id
        self.count = 0

        self.get_logger().info(f'pose_to_path: {self.get_namespace()}/{self.pose_topic} -> '
                               f'{self.get_namespace()}/{self.path_topic} (full), '
                               f'{self.get_namespace()}/{self.path_topic_stride} (stride={self.stride}), '
                               f'frame_id={self.frame_id}')

    def _on_pose(self, msg: PoseStamped):
        msg.header.frame_id = self.frame_id
        # Full path
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(msg)
        self.pub_full.publish(self.path)
        # Stride path (every Nth)
        if self.count % self.stride == 0:
            self.path_stride.header.stamp = msg.header.stamp
            self.path_stride.poses.append(msg)
            self.pub_stride.publish(self.path_stride)
        self.count += 1

def main():
    rclpy.init()
    node = PoseToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()