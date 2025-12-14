import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path

class PoseToPath(Node):
    def __init__(self):
        super().__init__('pose_to_path')
        self.declare_parameter('pose_topic', 'poses')                      # input PoseStamped
        self.declare_parameter('path_topic', 'poses_path_full')            # full path (lines)
        self.declare_parameter('path_topic_stride', 'poses_path_stride')   # stride path (every Nth as Path)
        self.declare_parameter('path_topic_stride_array', 'poses_stride_array')  # stride poses as PoseArray (axes)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('stride', 10)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.path_topic_stride = self.get_parameter('path_topic_stride').value
        self.path_topic_stride_array = self.get_parameter('path_topic_stride_array').value
        self.frame_id = self.get_parameter('frame_id').value
        self.stride = int(self.get_parameter('stride').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub = self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, qos)
        self.pub_full = self.create_publisher(Path, self.path_topic, qos)
        self.pub_stride = self.create_publisher(Path, self.path_topic_stride, qos)
        self.pub_stride_array = self.create_publisher(PoseArray, self.path_topic_stride_array, qos)

        self.path = Path(); self.path.header.frame_id = self.frame_id
        self.path_stride = Path(); self.path_stride.header.frame_id = self.frame_id
        self.pose_array = PoseArray(); self.pose_array.header.frame_id = self.frame_id
        self.count = 0

        self.get_logger().info(f'pose_to_path: {self.get_namespace()}/{self.pose_topic} -> '
                               f'{self.get_namespace()}/{self.path_topic} (full lines), '
                               f'{self.get_namespace()}/{self.path_topic_stride} (stride lines N={self.stride}), '
                               f'{self.get_namespace()}/{self.path_topic_stride_array} (stride axes), '
                               f'frame_id={self.frame_id}')

    def _on_pose(self, msg: PoseStamped):
        # Normalize frame
        msg.header.frame_id = self.frame_id

        # Full path (always append)
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = self.frame_id
        self.path.poses.append(msg)
        self.pub_full.publish(self.path)

        # Stride outputs
        if self.count % self.stride == 0:
            # Path (still published if you want lines every Nth; RViz can ignore it)
            self.path_stride.header.stamp = msg.header.stamp
            self.path_stride.header.frame_id = self.frame_id
            self.path_stride.poses.append(msg)
            self.pub_stride.publish(self.path_stride)

            # PoseArray (axes only in RViz)
            self.pose_array.header.stamp = msg.header.stamp
            self.pose_array.header.frame_id = self.frame_id
            self.pose_array.poses.append(msg.pose)
            self.pub_stride_array.publish(self.pose_array)

        self.count += 1

def main():
    rclpy.init()
    node = PoseToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()