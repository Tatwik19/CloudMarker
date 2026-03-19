#!/usr/bin/env python3
"""
Lab 4 — Reactive Navigation (CHOICE A)

Choice A:
- Trigger: front_distance < safety_distance
- Action: randomly rotate left or right by a random angle in [90, 180] degrees
"""

import math
import random

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__("reactive_navigator")

        # Service gate (lab requirement)
        self.navigation_enabled = False
        self.create_service(SetBool, "toggle_navigation", self.toggle_callback)

        # Publishers (unstamped + stamped)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_stamped_pub = self.create_publisher(TwistStamped, "cmd_vel_stamped", 10)

        # LaserScan subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # Parameters
        self.declare_parameter("safety_distance", 0.5)
        self.declare_parameter("forward_speed", 0.05)
        self.declare_parameter("rotation_speed", 0.8)
        self.declare_parameter("debug_logging", True)
        self.declare_parameter("front_window_degrees", 30.0)
        self.declare_parameter("front_angle_offset_degrees", -30.0)

        # Internal state
        self.mode = "idle"  # idle | forward | rotating
        self.rot_end_time = None
        self.rot_twist = Twist()

        self.front_distance = float("inf")
        self.last_scan_time = None
        self.scan_timeout_sec = 1.0
        self._missing_scan_warned = False
        self._last_debug_log_ns = 0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("ReactiveNavigator ready (Choice A).")

    def toggle_callback(self, request, response):
        self.navigation_enabled = bool(request.data)

        if not self.navigation_enabled:
            self.mode = "idle"
            self.publish_stop()

        response.success = True
        response.message = f"Navigation set to {self.navigation_enabled}"
        self.get_logger().info(response.message)
        return response

    def normalize_angle_into_scan_range(self, msg: LaserScan, angle: float) -> float:
        if msg.angle_max < msg.angle_min:
            return angle

        while angle < msg.angle_min:
            angle += 2.0 * math.pi
        while angle > msg.angle_max:
            angle -= 2.0 * math.pi
        return angle

    def get_index_for_angle(self, msg: LaserScan, angle_rad: float) -> int:
        angle_rad = self.normalize_angle_into_scan_range(msg, angle_rad)
        idx = int((angle_rad - msg.angle_min) / msg.angle_increment)
        return max(0, min(idx, len(msg.ranges) - 1))

    def min_distance_in_window(self, msg: LaserScan, center_angle_rad: float, half_window_rad: float) -> float:
        center = self.normalize_angle_into_scan_range(msg, center_angle_rad)
        start = self.get_index_for_angle(msg, center - half_window_rad)
        end = self.get_index_for_angle(msg, center + half_window_rad)

        if end < start:
            start, end = end, start

        best = float("inf")
        for value in msg.ranges[start : end + 1]:
            if value is None:
                continue
            if math.isfinite(value) and value > 0.0:
                if value < best:
                    best = value
        return best

    def scan_callback(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()
        self._missing_scan_warned = False

        window = math.radians(float(self.get_parameter("front_window_degrees").value))
        front_offset = math.radians(float(self.get_parameter("front_angle_offset_degrees").value))

        self.front_distance = self.min_distance_in_window(msg, front_offset, window)

    def control_loop(self):
        if not self.navigation_enabled:
            return

        now = self.get_clock().now()

        if self.last_scan_time is None:
            if not self._missing_scan_warned:
                self.get_logger().warn("No LaserScan received yet on scan topic. Holding position.")
                self._missing_scan_warned = True
            self.publish_stop()
            return

        if (now - self.last_scan_time).nanoseconds > int(self.scan_timeout_sec * 1e9):
            if not self._missing_scan_warned:
                self.get_logger().warn("LaserScan timed out. Holding position until scan resumes.")
                self._missing_scan_warned = True
            self.publish_stop()
            return

        safety_distance = float(self.get_parameter("safety_distance").value)
        should_trigger_now = self.front_distance < safety_distance

        if bool(self.get_parameter("debug_logging").value):
            now_ns = now.nanoseconds
            if now_ns - self._last_debug_log_ns >= int(1e9):
                self.get_logger().info(
                    f"Debug scan: mode={self.mode} front={self.front_distance:.2f} trigger={should_trigger_now}"
                )
                self._last_debug_log_ns = now_ns

        if self.mode == "idle":
            self.mode = "forward"

        if self.mode == "rotating":
            if self.rot_end_time is not None and now < self.rot_end_time:
                self.publish_cmd(self.rot_twist)
                return
            self.publish_stop()
            self.mode = "forward"
            return

        # mode == forward
        if should_trigger_now:
            self.start_rotation_maneuver()
            return

        self.drive_forward()

    def start_rotation_maneuver(self):
        self.get_logger().info(f"Obstacle trigger: front={self.front_distance:.2f}")

        angular_speed = float(self.get_parameter("rotation_speed").value)
        if angular_speed <= 0.0:
            angular_speed = 0.8

        direction = random.choice([1.0, -1.0])
        target_angle = direction * math.radians(random.uniform(90.0, 180.0))
        duration_s = abs(target_angle) / abs(angular_speed)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed if target_angle >= 0.0 else -angular_speed

        self.rot_twist = twist
        self.rot_end_time = self.get_clock().now() + Duration(seconds=duration_s)
        self.mode = "rotating"

    def drive_forward(self):
        msg = Twist()
        msg.linear.x = float(self.get_parameter("forward_speed").value)
        msg.angular.z = 0.0
        self.publish_cmd(msg)

    def publish_stop(self):
        self.publish_cmd(Twist())

    def publish_cmd(self, twist: Twist):
        self.cmd_pub.publish(twist)

        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = twist
        self.cmd_stamped_pub.publish(stamped)


def main():
    rclpy.init()
    node = ReactiveNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()