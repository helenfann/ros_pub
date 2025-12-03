# Usage:
# ros2 run pose_pubsub csv_pose_player --ros-args \
#   -r __ns:=/traj2 \
#   -p file_path:=/home/helen/ros2_ws/src/pose_pubsub/data/all_gt_poses_aria_new.csv \
#   -p pose_topic:=poses -p frame_id:=map -p loop:=False

import csv
import math
import time
from typing import List, Dict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
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
        self.declare_parameter('loop', False)
        # NEW: fixed startup delay in wall time
        self.declare_parameter('start_delay_sec', 0.0)

        self.file_path = self.get_parameter('file_path').value
        self.topic = self.get_parameter('pose_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = bool(self.get_parameter('loop').value)
        self.start_delay = float(self.get_parameter('start_delay_sec').value)

        if not self.file_path:
            raise RuntimeError('csv_pose_player: parameter file_path is required')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PoseStamped, self.topic, qos)

        # Load CSV
        self.rows = self._load_csv(self.file_path)
        if not self.rows:
            raise RuntimeError(f'csv_pose_player: no rows parsed from {self.file_path}')
        first = self.rows[0]; last = self.rows[-1]
        self.get_logger().info(f'CSV first ts={first["sec"]}.{first["nsec"]:09d} '
                               f'last ts={last["sec"]}.{last["nsec"]:09d} rows={len(self.rows)}')

        # Build CSV-relative time (start at 0 for first row)
        t0 = self.rows[0]['sec'] + self.rows[0]['nsec'] / 1e9
        self.rel_times = [(r['sec'] + r['nsec'] / 1e9) - t0 for r in self.rows]

        # Start wall clock + delay
        self.start_wall = time.time() + max(0.0, self.start_delay)
        self.idx = 0
        self.timer = self.create_timer(0.001, self._tick)

        self.get_logger().info(f'Playing {len(self.rows)} poses from {self.file_path} on '
                               f'{self.get_namespace()}/{self.topic} frame_id={self.frame_id} '
                               f'start_delay_sec={self.start_delay:.3f}')

    def _load_csv(self, path) -> List[Dict]:
        rows = []
        with open(path, 'r', newline='') as f:
            first = f.readline()
            if not first:
                return rows
            if first.strip().startswith('#'):
                header = [h.strip() for h in first.lstrip('#').split(',')]
                reader = csv.DictReader(f, fieldnames=header, skipinitialspace=True)
            else:
                f.seek(0)
                reader = csv.DictReader(f, skipinitialspace=True)
            for row in reader:
                norm = {(k.strip() if k else ''): (row[k].strip() if isinstance(row[k], str) else row[k])
                        for k in row.keys()}
                try:
                    rec = {
                        'counter': int(norm.get('counter', norm.get('# counter', '0'))),
                        'sec': int(norm.get('sec', '0')),
                        'nsec': int(norm.get('nsec', '0')),
                        'x': float(norm.get('x', '0')),
                        'y': float(norm.get('y', '0')),
                        'z': float(norm.get('z', '0')),
                    }
                    if all(k in norm and norm[k] != '' for k in ('qx', 'qy', 'qz', 'qw')):
                        rec['qx'] = float(norm['qx']); rec['qy'] = float(norm['qy'])
                        rec['qz'] = float(norm['qz']); rec['qw'] = float(norm['qw'])
                    else:
                        # Optional yaw fallback
                        yaw = float(norm.get('yaw', norm.get('theta', '0')))
                        qx, qy, qz, qw = yaw_to_quat(yaw)
                        rec['qx'], rec['qy'], rec['qz'], rec['qw'] = qx, qy, qz, qw
                    rows.append(rec)
                except Exception:
                    continue
        return rows

    def _tick(self):
        if self.idx >= len(self.rows):
            if self.loop:
                self.start_wall = time.time() + max(0.0, self.start_delay)
                self.idx = 0
            else:
                self.get_logger().info('Finished CSV playback')
                self.timer.cancel()
                return

        t_target = self.rel_times[self.idx]
        now_rel = time.time() - self.start_wall
        if now_rel + 0.0005 < t_target:
            return

        r = self.rows[self.idx]
        self.idx += 1

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = Time(sec=r['sec'], nanosec=r['nsec'])
        msg.pose.position.x = r['x']; msg.pose.position.y = r['y']; msg.pose.position.z = r['z']
        msg.pose.orientation.x = r['qx']; msg.pose.orientation.y = r['qy']
        msg.pose.orientation.z = r['qz']; msg.pose.orientation.w = r['qw']

        self.pub.publish(msg)
        self.get_logger().info(f'pub {self.topic}: t={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} '
                               f'x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f}')


def main():
    rclpy.init()
    node = CsvPosePlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()