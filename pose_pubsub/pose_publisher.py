#!/usr/bin/env python3
import os
from dataclasses import dataclass
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time as TimeMsg

@dataclass
class PoseRow:
    counter: int
    sec: int
    nsec: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

def read_csv(path: str) -> List[PoseRow]:
    rows: List[PoseRow] = []
    with open(path, 'r') as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            parts = [p.strip() for p in s.split(',')]
            if len(parts) < 10:
                continue
            try:
                rows.append(PoseRow(
                    counter=int(parts[0]),
                    sec=int(parts[1]),
                    nsec=int(parts[2]),
                    x=float(parts[3]),
                    y=float(parts[4]),
                    z=float(parts[5]),
                    qx=float(parts[6]),
                    qy=float(parts[7]),
                    qz=float(parts[8]),
                    qw=float(parts[9]),
                ))
            except ValueError:
                continue
    return rows

class CsvPosePublisher(Node):
    def __init__(self):
        super().__init__('csv_pose_publisher')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('topic', '/poses')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('use_csv_time', True)
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('loop', False)

        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.use_csv_time = self.get_parameter('use_csv_time').get_parameter_value().bool_value
        self.playback_speed = max(1e-6, float(self.get_parameter('playback_speed').get_parameter_value().double_value))
        self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value

        if not self.csv_path or not os.path.exists(self.csv_path):
            raise FileNotFoundError(f"CSV not found: {self.csv_path}")
        self.rows = read_csv(self.csv_path)
        if not self.rows:
            raise RuntimeError("No valid rows in CSV.")

        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.idx = 0
        self.timer = None

        if self.use_csv_time:
            self._publish_current()
            self._schedule_next_by_csv_time()
        else:
            period = 1.0 / max(1e-3, self.rate)
            self.timer = self.create_timer(period, self._on_timer_fixed)

    def _make_msg(self, row: PoseRow) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        if self.use_csv_time:
            msg.header.stamp = TimeMsg(sec=int(row.sec), nanosec=int(row.nsec))
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = row.x
        msg.pose.position.y = row.y
        msg.pose.position.z = row.z
        msg.pose.orientation.x = row.qx
        msg.pose.orientation.y = row.qy
        msg.pose.orientation.z = row.qz
        msg.pose.orientation.w = row.qw
        return msg

    def _publish_current(self):
        row = self.rows[self.idx]
        self.pub.publish(self._make_msg(row))

    def _advance_index(self) -> bool:
        next_idx = self.idx + 1
        if next_idx >= len(self.rows):
            if self.loop:
                self.idx = 0
                return True
            return False
        self.idx = next_idx
        return True

    def _schedule_next_by_csv_time(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        cur = self.rows[self.idx]
        next_idx = self.idx + 1 if (self.idx + 1) < len(self.rows) else (0 if self.loop else None)
        if next_idx is None:
            self.get_logger().info("Reached end of CSV playback.")
            return
        nxt = self.rows[next_idx]
        dt = (nxt.sec - cur.sec) + (nxt.nsec - cur.nsec) / 1e9
        dt = max(0.0, float(dt)) / self.playback_speed
        delay = max(1e-4, dt)

        def _on_timer_once():
            if not self._advance_index():
                self.get_logger().info("Finished CSV playback.")
                if self.timer is not None:
                    self.timer.cancel()
                return
            self._publish_current()
            self._schedule_next_by_csv_time()

        self.timer = self.create_timer(delay, _on_timer_once)

    def _on_timer_fixed(self):
        self._publish_current()
        if not self._advance_index():
            self.get_logger().info("Finished CSV playback.")
            if self.timer is not None:
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CsvPosePublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()