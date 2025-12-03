import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path

class PoseToPath(Node):
    def __init__(self):
        super().__init__('pose_to_path')

        self.declare_parameter('pose_topic', 'poses')
        self.declare_parameter('axis_stride', 20)
        self.declare_parameter('frame_id', 'map')

        self.axis_stride = int(self.get_parameter('axis_stride').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.full_path = Path(); self.full_path.header.frame_id = self.frame_id
        self.stride_path = Path(); self.stride_path.header.frame_id = self.frame_id
        self.pose_array = PoseArray(); self.pose_array.header.frame_id = self.frame_id
        self.count = 0

        # relative topics so namespace isolates each instance
        self.sub = self.create_subscription(PoseStamped, pose_topic, self.cb, qos)
        self.pub_full = self.create_publisher(Path, 'poses_path_full', qos)
        self.pub_stride = self.create_publisher(Path, 'poses_path_stride', qos)
        self.pub_array = self.create_publisher(PoseArray, 'poses_stride_array', qos)

    def cb(self, msg: PoseStamped):
        if msg.header.frame_id != self.frame_id:
            msg.header.frame_id = self.frame_id

        self.full_path.header.stamp = msg.header.stamp
        self.full_path.poses.append(msg)
        self.pub_full.publish(self.full_path)

        if self.count % self.axis_stride == 0:
            self.stride_path.header.stamp = msg.header.stamp
            self.stride_path.poses.append(msg)
            self.pub_stride.publish(self.stride_path)

            p = Pose()
            p.position = msg.pose.position
            p.orientation = msg.pose.orientation
            self.pose_array.header.stamp = msg.header.stamp
            self.pose_array.poses.append(p)
            self.pub_array.publish(self.pose_array)

        self.count += 1

def main():
    rclpy.init()
    node = PoseToPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()