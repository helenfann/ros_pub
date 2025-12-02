import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path

class PoseToPath(Node):
    def __init__(self):
        super().__init__('pose_to_path')
        self.declare_parameter('axis_stride', 30)
        self.axis_stride = self.get_parameter('axis_stride').get_parameter_value().integer_value or 5

        # Use Reliable to match RViz subscribers
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.full_path = Path(); self.full_path.header.frame_id = 'map'
        self.stride_path = Path(); self.stride_path.header.frame_id = 'map'
        self.pose_array = PoseArray(); self.pose_array.header.frame_id = 'map'  # optional, if you still want it
        self.count = 0

        self.sub = self.create_subscription(PoseStamped, '/poses', self.cb, qos)
        self.pub_full = self.create_publisher(Path, '/poses_path_full', qos)
        self.pub_stride = self.create_publisher(Path, '/poses_path_stride', qos)
        self.pub_array = self.create_publisher(PoseArray, '/poses_stride_array', qos)

    def cb(self, msg: PoseStamped):
        # keep frames consistent
        if msg.header.frame_id != self.full_path.header.frame_id:
            msg.header.frame_id = self.full_path.header.frame_id

        # all poses for trajectory lines
        self.full_path.header.stamp = msg.header.stamp
        self.full_path.poses.append(msg)
        self.pub_full.publish(self.full_path)

        # every Nth pose for axes-only visualisation (publish Path)
        if self.count % self.axis_stride == 0:
            self.stride_path.header.stamp = msg.header.stamp
            self.stride_path.poses.append(msg)
            self.pub_stride.publish(self.stride_path)

            # optional PoseArray publish if you still use it elsewhere
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