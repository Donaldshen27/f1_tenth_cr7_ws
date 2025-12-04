#!/usr/bin/env python3

import os
import csv
import math
from typing import Dict, List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from math import cos, sin
from ament_index_python.packages import get_package_share_directory

class PurePursuit(Node):
    def __init__(self):
        super().__init__('vicon_pp_node')

        self.rate_hz = 50
        self.timer_period = 1.0 / self.rate_hz

        self.declare_parameter('look_ahead', 0.3)
        self.declare_parameter('wheelbase', 0.325)
        self.declare_parameter('curvature_gain', 0.5)
        self.declare_parameter('steering_limit', 0.3)
        self.declare_parameter('fast_speed', 1.0)
        self.declare_parameter('slow_speed', 0.4)
        self.declare_parameter('dynamic_path_timeout', 0.25)
        self.declare_parameter('use_path_topic', True)
        self.declare_parameter('selected_path_topic', 'selected_path')
        self.declare_parameter('static_waypoints_file', 'xyhead_demo_pp.csv')
        self.declare_parameter('log_debug', False)

        self.look_ahead = float(self.get_parameter('look_ahead').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.curvature_gain = float(self.get_parameter('curvature_gain').value)
        self.steering_limit = float(self.get_parameter('steering_limit').value)
        self.fast_speed = float(self.get_parameter('fast_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.dynamic_path_timeout = float(self.get_parameter('dynamic_path_timeout').value)
        self.use_path_topic = bool(self.get_parameter('use_path_topic').value)
        self.selected_path_topic = self.get_parameter('selected_path_topic').value
        self.static_waypoints_file = self.get_parameter('static_waypoints_file').value
        self.log_debug_enabled = bool(self.get_parameter('log_debug').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # degrees

        # Read static waypoints for fallback
        self.goal = 0
        self.static_path = self.load_waypoints_from_file(self.static_waypoints_file)
        self.dynamic_path: Optional[Dict[str, np.ndarray]] = None
        self.dynamic_path_stamp: Optional[Time] = None
        self.vision_path_active = False
        # Publisher
        self.ctrl_pub = self.create_publisher(
            AckermannDriveStamped,
            "drive",
            1
            )
        self.path_pub = self.create_publisher(Path,
            'waypoints_path',
            10
            )

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "base_link"
        self.drive_msg.drive.speed = self.fast_speed

        # Subscriber
        self.vicon_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        if self.use_path_topic:
            self.path_sub = self.create_subscription(
                Path,
                self.selected_path_topic,
                self.selected_path_callback,
                10
            )
            self.vision_flag_sub = self.create_subscription(
                Bool,
                'vision_path_active',
                self.vision_flag_callback,
                10
            )


        # Timer for control loop
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.waypoint_timer = self.create_timer(1.0, self.publish_waypoints)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.drive_msg.drive.speed = 1.0  # set constant speed only if you have odometry
        self.get_logger().warn(f"Received state: x={self.x}, y={self.y}, yaw={self.yaw}")

    def load_waypoints_from_file(self, filename: str) -> Optional[Dict[str, np.ndarray]]:
        package_share_directory = get_package_share_directory('f1tenth_control')
        filepath = os.path.join(package_share_directory, 'waypoints', filename)

        try:
            with open(filepath, encoding='utf-8') as f:
                path_points = [tuple(line) for line in csv.reader(f) if len(line) >= 3]
        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {filepath}')
            return None

        if not path_points:
            self.get_logger().error(f'Waypoint file {filepath} is empty')
            return None

        xs = np.array([float(point[0]) for point in path_points], dtype=np.float32)
        ys = np.array([float(point[1]) for point in path_points], dtype=np.float32)
        yaw_deg = np.array([float(point[2]) for point in path_points], dtype=np.float32)

        self.get_logger().info(f'Loaded {len(xs)} static waypoints from {filepath}')
        return {'x': xs, 'y': ys, 'yaw_deg': yaw_deg}

    def publish_waypoints(self):
        if self.static_path is None:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw_deg in zip(self.static_path['x'],
                                 self.static_path['y'],
                                 self.static_path['yaw_deg']):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            yaw_rad = math.radians(yaw_deg)
            pose.pose.orientation.z = sin(yaw_rad / 2.0)
            pose.pose.orientation.w = cos(yaw_rad / 2.0)

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().debug(f'Published {len(path_msg.poses)} waypoints to RViz.')

    def get_active_path(self) -> Optional[Dict[str, np.ndarray]]:
        if self.use_path_topic:
            # Camera-only mode: only use dynamic path, no static fallback
            if self.dynamic_path is not None and self.dynamic_path_stamp is not None:
                age = (self.get_clock().now() - self.dynamic_path_stamp).nanoseconds * 1e-9
                if age <= self.dynamic_path_timeout:
                    return self.dynamic_path
            # No valid dynamic path - return None (car will stop)
            return None
        # Static waypoint mode (use_path_topic=False)
        return self.static_path

    def select_goal_index(self, dx: np.ndarray, dy: np.ndarray, dists: np.ndarray) -> Optional[int]:
        if dists.size == 0:
            return None

        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        diffs = np.abs(dists - self.look_ahead)
        ordered_indices = np.argsort(diffs)

        for idx in ordered_indices:
            ahead = dx[idx] * heading[0] + dy[idx] * heading[1]
            if ahead > 0:
                return int(idx)

        return int(ordered_indices[0]) if ordered_indices.size > 0 else None

    def selected_path_callback(self, msg: Path) -> None:
        arrays = self.path_msg_to_arrays(msg)
        if arrays is None:
            return
        self.dynamic_path = arrays
        self.dynamic_path_stamp = Time.from_msg(msg.header.stamp)

    def vision_flag_callback(self, msg: Bool) -> None:
        self.vision_path_active = bool(msg.data)

    def path_msg_to_arrays(self, path_msg: Path) -> Optional[Dict[str, np.ndarray]]:
        if not path_msg.poses:
            self.get_logger().warn('Received empty path message')
            return None

        xs: List[float] = []
        ys: List[float] = []
        yaw_deg: List[float] = []
        for pose in path_msg.poses:
            xs.append(pose.pose.position.x)
            ys.append(pose.pose.position.y)
            quat = pose.pose.orientation
            yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
            yaw_deg.append(math.degrees(yaw))

        return {
            'x': np.array(xs, dtype=np.float32),
            'y': np.array(ys, dtype=np.float32),
            'yaw_deg': np.array(yaw_deg, dtype=np.float32)
        }

    def timer_callback(self):
        path = self.get_active_path()
        if path is None:
            self.get_logger().warn('No path available - stopping car')
            self.drive_msg.header.stamp = self.get_clock().now().to_msg()
            self.drive_msg.drive.steering_angle = 0.0
            self.drive_msg.drive.speed = 0.0
            self.ctrl_pub.publish(self.drive_msg)
            return

        path_x = path['x']
        path_y = path['y']

        if path_x.size == 0:
            self.get_logger().warn('Active path is empty - stopping car')
            self.drive_msg.header.stamp = self.get_clock().now().to_msg()
            self.drive_msg.drive.steering_angle = 0.0
            self.drive_msg.drive.speed = 0.0
            self.ctrl_pub.publish(self.drive_msg)
            return

        dx = path_x - self.x
        dy = path_y - self.y
        dists = np.hypot(dx, dy)

        goal_idx = self.select_goal_index(dx, dy, dists)
        if goal_idx is None:
            self.get_logger().warn('Unable to find lookahead point - stopping car')
            self.drive_msg.header.stamp = self.get_clock().now().to_msg()
            self.drive_msg.drive.steering_angle = 0.0
            self.drive_msg.drive.speed = 0.0
            self.ctrl_pub.publish(self.drive_msg)
            return

        L = float(max(dists[goal_idx], 1e-3))
        # Geometric pure pursuit: alpha is angle from vehicle heading to lookahead point
        alpha = math.atan2(dy[goal_idx], dx[goal_idx]) - self.yaw
        # Standard pure pursuit steering formula
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), L)
        f_delta = float(np.clip(steering, -self.steering_limit, self.steering_limit))

        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_msg.drive.steering_angle = f_delta
        self.drive_msg.drive.speed = self.fast_speed

        if self.log_debug_enabled:
            ct_error = math.sin(alpha) * L
            self.get_logger().debug(
                f"goal_idx={goal_idx} dist={L:.3f} alpha={math.degrees(alpha):.2f}deg "
                f"steer={math.degrees(f_delta):.2f}deg ct_error={ct_error:.3f}"
            )

        self.ctrl_pub.publish(self.drive_msg)
        # print("From timer_callback: Published drive message with steering angle:", f_delta_deg)


def main(args=None):
    rclpy.init(args=args)
    pp = PurePursuit()

    try:
        rclpy.spin(pp)
    except KeyboardInterrupt:
        pass

    pp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
