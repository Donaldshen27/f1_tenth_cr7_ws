#!/usr/bin/env python3

"""Selects between camera-derived and prerecorded paths for pure pursuit."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32


@dataclass
class TimedPath:
    msg: Path
    stamp: Time


class LanePathMux(Node):
    """Publishes the most reliable path for the pure pursuit controller."""

    def __init__(self) -> None:
        super().__init__('lane_path_mux')

        self.declare_parameter('lane_path_topic', 'lane_center_path')
        self.declare_parameter('lane_confidence_topic', 'lane_confidence')
        self.declare_parameter('static_path_topic', 'waypoints_path')
        self.declare_parameter('selected_path_topic', 'selected_path')
        self.declare_parameter('vision_active_topic', 'vision_path_active')
        self.declare_parameter('min_lane_confidence', 0.01)  # Lowered - rely on min_lane_points instead
        self.declare_parameter('lane_path_timeout', 0.25)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('min_lane_points', 4)

        # Internal storage ----------------------------------------------------
        self.latest_lane_path: TimedPath | None = None
        self.latest_lane_confidence: float = 0.0
        self.latest_static_path: TimedPath | None = None

        # Publishers ----------------------------------------------------------
        selected_topic = self.get_parameter('selected_path_topic').value
        vision_topic = self.get_parameter('vision_active_topic').value
        self.selected_pub = self.create_publisher(Path, selected_topic, 10)
        self.vision_pub = self.create_publisher(Bool, vision_topic, 10)

        # Subscribers ---------------------------------------------------------
        self.create_subscription(
            Path,
            self.get_parameter('lane_path_topic').value,
            self.lane_path_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter('lane_confidence_topic').value,
            self.confidence_callback,
            10,
        )
        self.create_subscription(
            Path,
            self.get_parameter('static_path_topic').value,
            self.static_path_callback,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_selected_path)

        self.get_logger().info('Lane path multiplexer ready')

    # ------------------------------------------------------------------
    # Callbacks
    def lane_path_callback(self, msg: Path) -> None:
        self.latest_lane_path = TimedPath(msg=msg, stamp=Time.from_msg(msg.header.stamp))

    def confidence_callback(self, msg: Float32) -> None:
        self.latest_lane_confidence = float(msg.data)

    def static_path_callback(self, msg: Path) -> None:
        self.latest_static_path = TimedPath(msg=msg, stamp=Time.from_msg(msg.header.stamp))

    # ------------------------------------------------------------------
    def publish_selected_path(self) -> None:
        active_path, vision_active = self.choose_path()
        if active_path is None:
            self.get_logger().warn('No path available yet')
            return

        self.selected_pub.publish(active_path.msg)
        vision_msg = Bool()
        vision_msg.data = vision_active
        self.vision_pub.publish(vision_msg)

    def choose_path(self) -> tuple[TimedPath | None, bool]:
        min_conf = float(self.get_parameter('min_lane_confidence').value)
        lane_timeout = Duration(seconds=float(self.get_parameter('lane_path_timeout').value))
        min_points = int(self.get_parameter('min_lane_points').value)

        now = self.get_clock().now()

        # Only use lane path - no static waypoint fallback (for odom-free mode)
        if self.latest_lane_path is not None:
            path_age = now - self.latest_lane_path.stamp
            lane_valid = (
                self.latest_lane_confidence >= min_conf
                and len(self.latest_lane_path.msg.poses) >= min_points
                and path_age <= lane_timeout
            )
            if lane_valid:
                return self.latest_lane_path, True

        # No valid lane path - return None (car will stop safely)
        return None, False


def main(args=None):
    rclpy.init(args=args)
    node = LanePathMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
