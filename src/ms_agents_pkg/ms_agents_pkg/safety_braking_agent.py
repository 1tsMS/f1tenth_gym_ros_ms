#!/usr/bin/env python3
"""Automatic Emergency Braking (AEB) Agent using Corridor-Filtered Time-to-Collision (TTC).

This agent drives the car forward and constantly monitors the LiDAR scan.
To prevent false alarms from side walls when driving close to track edges,
it converts LiDAR readings into Cartesian coordinates (x, y) and only evaluates
TTC for obstacles located inside the car's forward driving corridor (|y| <= corridor_width).
"""

import numpy as np
import rclpy
from rclpy.node import Node

# ROS 2 message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyBrakingAgent(Node):

    def __init__(self):
        super().__init__("safety_braking_agent")

        # ------------------------------------------------------
        # 1. ROS 2 Subscribers & Publishers
        # ------------------------------------------------------

        # Subscribe to LiDAR scans
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        # In F1TENTH Gym, ego vehicle odometry is published on /ego_racecar/odom
        self.odom_sub = self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.odom_callback,
            10
        )

        # Fallback subscription in case /odom is used directly
        self.odom_fallback_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # Publisher for drive commands
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10
        )

        # ------------------------------------------------------
        # 2. Parameters & State Variables
        # ------------------------------------------------------

        # Current forward speed of the car (in m/s)
        self.current_speed = 0.0

        # Normal test driving speed (m/s)
        self.drive_speed = 2.0

        # Safety threshold in seconds.
        # If TTC < ttc_threshold in the driving corridor, brakes engage.
        self.ttc_threshold = 0.70

        # Vehicle width filter (to ignore side walls parallel to the car)
        # F1TENTH car width is ~0.30m -> half-width is 0.15m + 0.05m safety margin = 0.20m
        self.corridor_half_width = 0.20  # meters (left & right of car centerline)

        # Flag to indicate if brakes have been triggered
        self.emergency_brake_engaged = False

        self.get_logger().info(
            f"Safety Braking Agent Initialized! Target Speed: {self.drive_speed} m/s | "
            f"TTC Threshold: {self.ttc_threshold}s | Corridor Half-Width: {self.corridor_half_width}m"
        )

    # ----------------------------------------------------------
    # Odometry Callback: updates current car speed
    # ----------------------------------------------------------
    def odom_callback(self, odom_msg: Odometry):
        # linear.x is the forward velocity in the car's body frame
        self.current_speed = float(odom_msg.twist.twist.linear.x)

    # ----------------------------------------------------------
    # LiDAR Callback: calculates Time-to-Collision (TTC)
    # ----------------------------------------------------------
    def scan_callback(self, scan_msg: LaserScan):

        # If brakes are already locked, keep sending 0 speed
        if self.emergency_brake_engaged:
            self.publish_drive(0.0, 0.0)
            return

        # 1. Convert LiDAR ranges to a numpy array
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        n_points = len(ranges)

        # 2. Compute angle for each beam: angle_i = min + i * inc
        angles = scan_msg.angle_min + np.arange(n_points) * scan_msg.angle_increment

        # 3. Filter out invalid measurements
        valid = (
            np.isfinite(ranges) &
            (ranges >= scan_msg.range_min) &
            (ranges <= scan_msg.range_max)
        )

        valid_ranges = ranges[valid]
        valid_angles = angles[valid]

        if len(valid_ranges) == 0:
            return

        # 4. Convert Polar (r, theta) -> Cartesian (x, y) coordinates
        # x = distance directly ahead (+x is forward)
        # y = distance to the side (+y is left, -y is right)
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)

        # 5. Filter: ONLY check points inside the vehicle's driving corridor
        # Points must be in front of the car (x > 0) and within the car's body width (|y| <= corridor_half_width)
        in_corridor = (x > 0.0) & (np.abs(y) <= self.corridor_half_width)

        if not np.any(in_corridor):
            # Nothing in the direct path of the vehicle -> safe to drive
            self.publish_drive(self.drive_speed, 0.0)
            return

        corridor_ranges = valid_ranges[in_corridor]
        corridor_angles = valid_angles[in_corridor]

        # 6. Determine effective forward speed
        effective_speed = max(self.current_speed, self.drive_speed, 0.5)

        # 7. Calculate Closing Speed for corridor beams:
        # closing_speed = speed * cos(beam_angle)
        closing_speeds = effective_speed * np.cos(corridor_angles)
        approaching = closing_speeds > 0.001

        if not np.any(approaching):
            self.publish_drive(self.drive_speed, 0.0)
            return

        # 8. Calculate Time-to-Collision (TTC): Distance / Closing Speed
        ttc_array = corridor_ranges[approaching] / closing_speeds[approaching]
        min_ttc = float(np.min(ttc_array))
        min_dist = float(np.min(corridor_ranges[approaching]))

        # Periodic status log
        self.get_logger().info(
            f"Speed: {effective_speed:.2f} m/s | Ahead Dist: {min_dist:.2f} m | Min TTC: {min_ttc:.2f} s",
            throttle_duration_sec=0.4
        )

        # ------------------------------------------------------
        # 9. Decision: Emergency Brake or Drive
        # ------------------------------------------------------
        if min_ttc < self.ttc_threshold:
            self.emergency_brake_engaged = True

            self.get_logger().error(
                f"🛑 EMERGENCY BRAKE TRIGGERED! Min TTC = {min_ttc:.3f}s (< {self.ttc_threshold}s) | Distance Ahead = {min_dist:.2f}m"
            )

            # Slam brakes
            self.publish_drive(0.0, 0.0)

        else:
            # Safe -> keep driving forward
            self.publish_drive(self.drive_speed, 0.0)

    # ----------------------------------------------------------
    # Helper to publish Ackermann drive commands
    # ----------------------------------------------------------
    def publish_drive(self, speed: float, steering_angle: float):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyBrakingAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
