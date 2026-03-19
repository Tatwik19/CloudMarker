

#!/usr/bin/env python3
"""
Lab 3 — Choice B: Triangle
- 3 equal sides: 0.8 m
- External turn: 120 deg
Publishes geometry_msgs/TwistStamped.

"""

from __future__ import annotations

import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


def _normalize_ns(ns: str) -> str:
    ns = (ns or "").strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def _default_cmd_vel_topic(robot_id: str, ros_namespace: str) -> str:
    robot_id = (robot_id or "").strip()
    if robot_id:
        # Accept "09" or "robot_09"
        if robot_id.startswith("robot_"):
            return f"/{robot_id}/cmd_vel"
        return f"/robot_{robot_id}/cmd_vel"

    ns = _normalize_ns(ros_namespace)
    if ns:
        return f"{ns}/cmd_vel"

    return "/cmd_vel"


class RobotDrive(Node):
    def __init__(self) -> None:
        super().__init__("robot_drive")

        # Parameters (default robot_id set to "09")
        self.declare_parameter("robot_id", "09")
        self.declare_parameter("cmd_vel_topic", "")
        self.declare_parameter("linear_speed", 0.25)    # m/s - increased from 0.2
        self.declare_parameter("angular_speed", 0.6)    # rad/s - increased from 0.5
        self.declare_parameter("distance_multiplier", 1.15)  # Compensate for slippage
        self.declare_parameter("angle_multiplier", 1.05)     # Fine-tune turns
        self.declare_parameter("start_delay_s", 1.0)    # give time to start recording
        self.declare_parameter("turn_left", True)       # True=left, False=right

        robot_id = str(self.get_parameter("robot_id").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        v = float(self.get_parameter("linear_speed").value)
        w = float(self.get_parameter("angular_speed").value)
        dist_mult = float(self.get_parameter("distance_multiplier").value)
        angle_mult = float(self.get_parameter("angle_multiplier").value)
        start_delay_s = float(self.get_parameter("start_delay_s").value)
        turn_left = bool(self.get_parameter("turn_left").value)

        if not cmd_vel_topic:
            cmd_vel_topic = _default_cmd_vel_topic(robot_id, os.environ.get("ROS_NAMESPACE", ""))

        self.publisher_ = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.get_logger().info(f"Publishing TwistStamped to: {cmd_vel_topic}")

        time.sleep(max(0.0, start_delay_s))
        self.get_logger().info("Starting Triangle Motion...")

        self.execute_triangle(v, w, dist_mult, angle_mult, turn_left)

        # Ensure stop at end
        self.stop()
        self.get_logger().info("Done.")
        rclpy.shutdown()

    def _msg(self, linear_x: float = 0.0, angular_z: float = 0.0) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        return msg

    def move_forward(self, duration_s: float, v: float) -> None:
        """Publish velocity commands continuously during movement"""
        rate = 10  # Hz - publish 10 times per second
        publish_period = 1.0 / rate
        elapsed = 0.0
        
        while elapsed < duration_s:
            self.publisher_.publish(self._msg(linear_x=v, angular_z=0.0))
            time.sleep(publish_period)
            elapsed += publish_period
        
        self.stop()

    def turn_robot(self, duration_s: float, w: float) -> None:
        """Publish turn commands continuously during rotation"""
        rate = 10  # Hz
        publish_period = 1.0 / rate
        elapsed = 0.0
        
        while elapsed < duration_s:
            self.publisher_.publish(self._msg(linear_x=0.0, angular_z=w))
            time.sleep(publish_period)
            elapsed += publish_period
        
        self.stop()

    def stop(self) -> None:
        """Send multiple stop commands to ensure robot stops"""
        for _ in range(5):
            self.publisher_.publish(self._msg(0.0, 0.0))
            time.sleep(0.1)

    def execute_triangle(self, v: float, w: float, dist_mult: float, angle_mult: float, turn_left: bool) -> None:
        side_m = 0.8
        turn_deg = 120.0

        # Apply multipliers for real-world calibration
        forward_t = (side_m * dist_mult) / v
        turn_rad = math.radians(turn_deg * angle_mult)
        turn_t = turn_rad / abs(w)

        signed_w = abs(w) if turn_left else -abs(w)

        self.get_logger().info(f"Triangle params: side={side_m}m, forward_t={forward_t:.2f}s, turn={turn_deg}°, turn_t={turn_t:.2f}s")

        for i in range(3):
            self.get_logger().info(f"Side {i+1}/3...")
            self.move_forward(forward_t, v)
            self.turn_robot(turn_t, signed_w)


def main() -> None:
    rclpy.init()
    RobotDrive()


if __name__ == "__main__":
    main()

