#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyBrakingAgent(Node):

    def __init__(self):
        super().__init__("safety_braking_agent")

        # --------------------------------------------------
        # ROS setup
        # --------------------------------------------------

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.odom_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10
        )

        # --------------------------------------------------
        # Driving & Safety settings
        # --------------------------------------------------

        self.drive_speed = 2.0
        self.current_speed = 0.0

        # Desired stopping distance at nominal speed (in meters)
        self.min_stopping_distance = 0.80

        # Calculate TTC threshold automatically: TTC = Distance / Speed
        self.ttc_threshold = self.min_stopping_distance / self.drive_speed

        # Vehicle width corridor: only check obstacles in car's width path
        # Half width = 0.15m + 0.05m margin = 0.20m
        self.corridor_half_width = 0.20

        # State variable: True when stopped in front of obstacle
        self.is_stopped = False

        self.get_logger().info(
            f"Safety Braking Agent Initialized! Target Speed: {self.drive_speed} m/s | "
            f"Stop Distance: {self.min_stopping_distance} m (TTC: {self.ttc_threshold:.2f} s)"
        )

    # ------------------------------------------------------
    # Publish drive command
    # ------------------------------------------------------

    def publish_drive(self, speed, steering_angle):

        msg = AckermannDriveStamped()

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(msg)

    # ------------------------------------------------------
    # Clean LiDAR data and compute angles
    # ------------------------------------------------------

    def get_valid_scan(self, scan_msg):

        ranges = np.asarray(
            scan_msg.ranges,
            dtype=np.float32
        )

        n_points = len(ranges)

        angles = scan_msg.angle_min + np.arange(n_points) * scan_msg.angle_increment

        valid = np.isfinite(ranges)
        valid &= (ranges >= scan_msg.range_min)
        valid &= (ranges <= scan_msg.range_max)

        return ranges[valid], angles[valid]

    # ------------------------------------------------------
    # Calculate Time-to-Collision in driving corridor
    # ------------------------------------------------------

    def compute_min_ttc(self, valid_ranges, valid_angles):

        if len(valid_ranges) == 0:
            return None, None

        # Convert Polar (r, theta) -> Cartesian (x, y)
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)

        # Filter to points inside the vehicle corridor
        in_corridor = (x > 0.0) & (np.abs(y) <= self.corridor_half_width)

        if not np.any(in_corridor):
            return None, None

        corridor_ranges = valid_ranges[in_corridor]
        corridor_angles = valid_angles[in_corridor]

        effective_speed = max(self.current_speed, self.drive_speed, 0.5)

        # Closing speed towards obstacle
        closing_speeds = effective_speed * np.cos(corridor_angles)
        approaching = closing_speeds > 0.001

        if not np.any(approaching):
            return None, None

        ttc_array = corridor_ranges[approaching] / closing_speeds[approaching]

        min_ttc = float(np.min(ttc_array))
        min_dist = float(np.min(corridor_ranges[approaching]))

        return min_ttc, min_dist

    # ------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------

    def odom_callback(self, msg):

        self.current_speed = float(msg.twist.twist.linear.x)

    # ------------------------------------------------------
    # LiDAR callback
    # ------------------------------------------------------

    def scan_callback(self, msg):

        valid_ranges, valid_angles = self.get_valid_scan(msg)

        min_ttc, min_dist = self.compute_min_ttc(valid_ranges, valid_angles)

        # --------------------------------------------------
        # Case 1: Obstacle ahead inside safety threshold
        # --------------------------------------------------

        if min_ttc is not None and min_ttc < self.ttc_threshold:

            if not self.is_stopped:

                self.is_stopped = True

                self.get_logger().error(
                    f"🛑 BRAKING! Stopped at {min_dist:.2f} m ahead (Min TTC = {min_ttc:.2f} s)"
                )

                self.publish_drive(
                    0.0,
                    0.0
                )

            # While stationary in front of the wall, stop logging and stop sending commands
            return

        # --------------------------------------------------
        # Case 2: Path is clear (or car was repositioned)
        # --------------------------------------------------

        if self.is_stopped:

            self.is_stopped = False

            self.get_logger().info(
                "✅ Repositioned! Path ahead is clear, resuming drive..."
            )

        # Status log while driving
        if min_ttc is not None:

            self.get_logger().info(
                f"Speed: {self.current_speed:.2f} m/s | Ahead Dist: {min_dist:.2f} m | Min TTC: {min_ttc:.2f} s",
                throttle_duration_sec=0.4
            )

        self.publish_drive(
            self.drive_speed,
            0.0
        )

# ----------------------------------------------------------
# Main
# ----------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = SafetyBrakingAgent()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.publish_drive(
            0.0,
            0.0
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()

