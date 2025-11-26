#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.declare_parameter('topic', '/poses')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.sub = self.create_subscription(PoseStamped, topic, self.cb, 10)

    def cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self.get_logger().info(f"t=({msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}) "
                               f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
                               f"quat=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")

def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()