#!/usr/bin/env python3

"""Multiplexes between lane following and cone corridor following for construction zones."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Path
from std_msgs.msg import Float32, String


class ZoneMode(Enum):
    """State machine modes for construction zone navigation."""
    LANE_FOLLOWING = "LANE"
    CONE_CORRIDOR = "CONE"


@dataclass
class TimedPath:
    msg: Path
    stamp: Time


class ConstructionZoneMux(Node):
    """Switches between lane and cone corridor paths with hysteresis-based mode transitions."""

    def __init__(self) -> None:
        super().__init__('construction_zone_mux')

        # Topic parameters -----------------------------------------------------
        self.declare_parameter('lane_path_topic', 'lane_center_path')
        self.declare_parameter('lane_confidence_topic', 'lane_confidence')
        self.declare_parameter('cone_path_topic', 'cone_corridor_path')
        self.declare_parameter('cone_confidence_topic', 'cone_confidence')
        self.declare_parameter('selected_path_topic', 'selected_path')
        self.declare_parameter('zone_mode_topic', 'zone_mode')

        # Transition parameters ------------------------------------------------
        self.declare_parameter('entry_threshold', 0.4)
        self.declare_parameter('exit_threshold', 0.2)
        self.declare_parameter('lane_exit_threshold', 0.04)
        self.declare_parameter('hysteresis_frames', 5)
        self.declare_parameter('require_lane_on_exit', True)
        self.declare_parameter('publish_rate_hz', 50.0)

        # State variables ------------------------------------------------------
        self.current_mode = ZoneMode.LANE_FOLLOWING
        self.entry_counter = 0  # Counts consecutive frames with high cone confidence
        self.exit_counter = 0   # Counts consecutive frames with low cone confidence

        # Latest data ----------------------------------------------------------
        self.latest_lane_path: TimedPath | None = None
        self.latest_lane_confidence: float = 0.0
        self.latest_cone_path: TimedPath | None = None
        self.latest_cone_confidence: float = 0.0

        # Publishers -----------------------------------------------------------
        selected_topic = self.get_parameter('selected_path_topic').value
        mode_topic = self.get_parameter('zone_mode_topic').value
        self.selected_pub = self.create_publisher(Path, selected_topic, 10)
        self.mode_pub = self.create_publisher(String, mode_topic, 10)

        # Subscribers ----------------------------------------------------------
        self.create_subscription(
            Path,
            self.get_parameter('lane_path_topic').value,
            self.lane_path_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter('lane_confidence_topic').value,
            self.lane_confidence_callback,
            10,
        )
        self.create_subscription(
            Path,
            self.get_parameter('cone_path_topic').value,
            self.cone_path_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter('cone_confidence_topic').value,
            self.cone_confidence_callback,
            10,
        )

        # Timer ----------------------------------------------------------------
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_selected_path)

        self.get_logger().info('Construction zone mux ready (starting in LANE mode)')

    # ------------------------------------------------------------------
    # Callbacks
    def lane_path_callback(self, msg: Path) -> None:
        self.latest_lane_path = TimedPath(msg=msg, stamp=Time.from_msg(msg.header.stamp))

    def lane_confidence_callback(self, msg: Float32) -> None:
        self.latest_lane_confidence = float(msg.data)

    def cone_path_callback(self, msg: Path) -> None:
        self.latest_cone_path = TimedPath(msg=msg, stamp=Time.from_msg(msg.header.stamp))

    def cone_confidence_callback(self, msg: Float32) -> None:
        self.latest_cone_confidence = float(msg.data)

    # ------------------------------------------------------------------
    # State machine logic
    def update_state_machine(self) -> None:
        """Update state machine based on current confidence values and hysteresis counters."""
        entry_thresh = float(self.get_parameter('entry_threshold').value)
        exit_thresh = float(self.get_parameter('exit_threshold').value)
        lane_exit_thresh = float(self.get_parameter('lane_exit_threshold').value)
        hysteresis = int(self.get_parameter('hysteresis_frames').value)
        require_lane = bool(self.get_parameter('require_lane_on_exit').value)

        if self.current_mode == ZoneMode.LANE_FOLLOWING:
            # Check for entry into cone corridor
            if self.latest_cone_confidence > entry_thresh:
                self.entry_counter += 1
                self.exit_counter = 0  # Reset exit counter
            else:
                self.entry_counter = 0

            # Transition to CONE mode after sustained high cone confidence
            if self.entry_counter >= hysteresis:
                self.current_mode = ZoneMode.CONE_CORRIDOR
                self.entry_counter = 0
                self.exit_counter = 0
                self.get_logger().info(
                    f'Entering CONE_CORRIDOR mode (cone_confidence={self.latest_cone_confidence:.2f})'
                )

        elif self.current_mode == ZoneMode.CONE_CORRIDOR:
            # Check for exit from cone corridor
            exit_condition = self.latest_cone_confidence < exit_thresh
            if require_lane:
                # Also require lane to be visible before exiting
                exit_condition = exit_condition and (self.latest_lane_confidence > lane_exit_thresh)

            if exit_condition:
                self.exit_counter += 1
                self.entry_counter = 0  # Reset entry counter
            else:
                self.exit_counter = 0

            # Transition to LANE mode after sustained low cone confidence (and visible lane)
            if self.exit_counter >= hysteresis:
                self.current_mode = ZoneMode.LANE_FOLLOWING
                self.entry_counter = 0
                self.exit_counter = 0
                self.get_logger().info(
                    f'Returning to LANE_FOLLOWING mode (cone_confidence={self.latest_cone_confidence:.2f}, lane_confidence={self.latest_lane_confidence:.2f})'
                )

    def publish_selected_path(self) -> None:
        """Publish the appropriate path based on current mode."""
        self.update_state_machine()

        # Select path based on current mode
        if self.current_mode == ZoneMode.LANE_FOLLOWING:
            selected_path = self.latest_lane_path
            path_type = "lane"
        else:  # CONE_CORRIDOR
            selected_path = self.latest_cone_path
            path_type = "cone"

            # Safety: If in CONE mode but lost both signals, stop (publish empty path)
            if (self.latest_cone_path is None or len(self.latest_cone_path.msg.poses) == 0) and \
               (self.latest_lane_path is None or len(self.latest_lane_path.msg.poses) == 0):
                self.get_logger().warn(
                    'In CONE mode but both paths unavailable - stopping for safety',
                    throttle_duration_sec=1.0
                )
                empty_path = Path()
                empty_path.header.frame_id = "base_link"
                empty_path.header.stamp = self.get_clock().now().to_msg()
                self.selected_pub.publish(empty_path)

                mode_msg = String()
                mode_msg.data = self.current_mode.value
                self.mode_pub.publish(mode_msg)
                return

        # Publish selected path
        if selected_path is not None and len(selected_path.msg.poses) > 0:
            self.selected_pub.publish(selected_path.msg)
            self.get_logger().debug(
                f'Publishing {path_type} path ({len(selected_path.msg.poses)} points)',
                throttle_duration_sec=2.0
            )
        else:
            # No valid path available - publish empty path (car will stop)
            self.get_logger().warn(
                f'No valid {path_type} path available in {self.current_mode.value} mode',
                throttle_duration_sec=1.0
            )
            empty_path = Path()
            empty_path.header.frame_id = "base_link"
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.selected_pub.publish(empty_path)

        # Publish current mode for debugging
        mode_msg = String()
        mode_msg.data = self.current_mode.value
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConstructionZoneMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
