# Usage: ros2 run pose_pubsub csv_pose_player --ros-args -p file_path:=/home/helen/ros2_ws/src/pose_pubsub/data/all_gt_poses_aria_new.csv -p pose_topic:=/traj2/poses_path_full -p frame_id:=map -p publish_rate:=30.0


import csv
import math
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

def yaw_to_quat(yaw: float):
    # roll=pitch=0
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)

class CsvPosePlayer(Node):
    def __init__(self):
        super().__init__('csv_pose_player')

        self.declare_parameter('file_path', '')
        self.declare_parameter('pose_topic', 'poses')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 30.0)   # Hz
        self.declare_parameter('loop', False)

        self.file_path = self.get_parameter('file_path').value
        self.topic = self.get_parameter('pose_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate').value)
        self.loop = bool(self.get_parameter('loop').value)

        if not self.file_path:
            raise RuntimeError('csv_pose_player: parameter file_path is required')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PoseStamped, self.topic, qos)

        self.rows = self._load_csv(self.file_path)
        self.idx = 0
        self.timer = self.create_timer(1.0 / max(rate, 1e-3), self._tick)
        self.get_logger().info(f'Playing {len(self.rows)} poses from {self.file_path} on {self.get_namespace()}/{self.topic}')

    def _load_csv(self, path) -> List[Dict[str, str]]:
        # skipinitialspace handles headers like "x, y, z"
        with open(path, 'r', newline='') as f:
            r = csv.DictReader(f, skipinitialspace=True)
            rows = []
            for row in r:
                # Normalize keys and strip whitespace from values
                norm = { (k.strip() if k is not None else ''): (v.strip() if isinstance(v, str) else v)
                         for k, v in row.items() }
                rows.append(norm)
            return rows

    def _tick(self):
        if self.idx >= len(self.rows):
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info('Finished CSV')
                self.timer.cancel()
                return

        row = self.rows[self.idx]
        self.idx += 1

        def getf(key, default=0.0):
            v = row.get(key, '')
            return float(v) if v not in (None, '') else float(default)

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = getf('x')
        msg.pose.position.y = getf('y')
        msg.pose.position.z = getf('z', 0.0)

        if all(k in row and row[k] != '' for k in ('qx', 'qy', 'qz', 'qw')):
            msg.pose.orientation.x = getf('qx')
            msg.pose.orientation.y = getf('qy')
            msg.pose.orientation.z = getf('qz')
            msg.pose.orientation.w = getf('qw')
        else:
            # try yaw (radians) or yaw_deg
            if row.get('yaw_deg', '') != '':
                yaw = math.radians(getf('yaw_deg'))
            else:
                yaw = getf('yaw', getf('theta', 0.0))
            qx, qy, qz, qw = yaw_to_quat(yaw)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw


        self.pub.publish(msg)
        # Debug one-line confirmation
        self.get_logger().info(f'pub {self.topic}: x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f} z={msg.pose.position.z:.3f}')
def main():
    rclpy.init()
    node = CsvPosePlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()