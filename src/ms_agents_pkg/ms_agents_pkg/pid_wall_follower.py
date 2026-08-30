#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class PidWallFollower(Node):

    def __init__(self):
        super().__init__("pid_wall_follower")

        # --------------------------------------------------
        # ROS setup
        # --------------------------------------------------

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10
        )

        # --------------------------------------------------
        # Wall Following Settings
        # --------------------------------------------------

        # Which wall to follow: "right" or "left"
        self.wall_side = "left"

        # Desired distance to maintain from the wall (meters)
        self.target_distance = 0.80

        # Lookahead distance L for future prediction (meters)
        self.lookahead_dist = 0.80

        # Front obstacle / pocket threshold (meters)
        # If a wall is closer than this in front, turn sharply to escape pocket
        self.front_threshold = 1.30

        # Speeds
        self.straight_speed = 2.5
        self.turn_speed = 1.2

        # --------------------------------------------------
        # PID Controller Gains
        # --------------------------------------------------

        self.kp = 1.20
        self.kd = 0.08
        self.ki = 0.00

        # Controller state variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

        # Maximum steering angle limits (~24 degrees)
        self.max_steer = 0.4189

        self.get_logger().info(
            f"PID Wall Follower Initialized! Following {self.wall_side} wall at {self.target_distance} m | "
            f"Front Escape Threshold: {self.front_threshold} m"
        )

    # ------------------------------------------------------
    # Publish drive command
    # ------------------------------------------------------

    def publish_drive(self, speed, steering_angle):

        msg = AckermannDriveStamped()

        # Clamp steering to vehicle physical limits
        clamped_steer = float(np.clip(steering_angle, -self.max_steer, self.max_steer))

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = clamped_steer

        self.drive_pub.publish(msg)

    # ------------------------------------------------------
    # Get LiDAR distance at a specific angle (in radians)
    # ------------------------------------------------------

    def get_range_at_angle(self, scan_msg, target_angle_rad):

        idx = int(round((target_angle_rad - scan_msg.angle_min) / scan_msg.angle_increment))
        idx = max(0, min(len(scan_msg.ranges) - 1, idx))

        r = scan_msg.ranges[idx]

        if not np.isfinite(r) or r < scan_msg.range_min or r > scan_msg.range_max:

            start = max(0, idx - 5)
            end = min(len(scan_msg.ranges), idx + 6)
            window = np.array(scan_msg.ranges[start:end])
            valid_window = window[np.isfinite(window)]

            if len(valid_window) > 0:

                r = float(np.mean(valid_window))

            else:

                r = scan_msg.range_max

        return float(r)

    # ------------------------------------------------------
    # Get clearance directly ahead (±15 degrees)
    # ------------------------------------------------------

    def get_front_clearance(self, scan_msg):

        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
        n_points = len(ranges)
        angles = scan_msg.angle_min + np.arange(n_points) * scan_msg.angle_increment

        # Forward cone: -15 deg to +15 deg
        front_mask = np.abs(angles) <= np.radians(15.0)
        front_ranges = ranges[front_mask]

        valid = np.isfinite(front_ranges) & (front_ranges >= scan_msg.range_min)

        if np.any(valid):

            return float(np.min(front_ranges[valid]))

        return scan_msg.range_max

    # ------------------------------------------------------
    # Compute current and predicted distance to wall
    # ------------------------------------------------------

    def calculate_wall_distance(self, scan_msg):

        if self.wall_side == "right":

            b_angle = -np.pi / 2.0  # -90 deg (Right)
            a_angle = -np.pi / 4.0  # -45 deg (Right-Forward)

        else:

            b_angle = np.pi / 2.0   # +90 deg (Left)
            a_angle = np.pi / 4.0   # +45 deg (Left-Forward)

        theta = np.abs(a_angle - b_angle)  # 45 degrees

        b_dist = self.get_range_at_angle(scan_msg, b_angle)
        a_dist = self.get_range_at_angle(scan_msg, a_angle)

        # --------------------------------------------------
        # Step 1: Calculate orientation angle alpha
        # --------------------------------------------------

        numerator = (a_dist * np.cos(theta)) - b_dist
        denominator = a_dist * np.sin(theta)

        if np.abs(denominator) < 1e-4:

            alpha = 0.0

        else:

            alpha = np.arctan2(numerator, denominator)

        # --------------------------------------------------
        # Step 2: Current perpendicular distance D_t
        # --------------------------------------------------

        current_dist = b_dist * np.cos(alpha)

        # --------------------------------------------------
        # Step 3: Predicted lookahead distance D_{t+1}
        # --------------------------------------------------

        predicted_dist = current_dist + (self.lookahead_dist * np.sin(alpha))

        return current_dist, predicted_dist, alpha

    # ------------------------------------------------------
    # PID Controller: computes steering angle from error
    # ------------------------------------------------------

    def pid_control(self, error, dt):

        p_term = self.kp * error

        if dt > 0.0:

            d_term = self.kd * ((error - self.prev_error) / dt)

        else:

            d_term = 0.0

        self.integral += error * dt
        i_term = self.ki * self.integral

        steering = p_term + d_term + i_term

        if self.wall_side == "left":

            steering = -steering

        self.prev_error = error

        return float(steering)

    # ------------------------------------------------------
    # LiDAR callback
    # ------------------------------------------------------

    def scan_callback(self, msg):

        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is None:

            dt = 0.025

        else:

            dt = max(1e-4, current_time - self.prev_time)

        self.prev_time = current_time

        # 1. Check distance directly ahead
        front_dist = self.get_front_clearance(msg)

        # --------------------------------------------------
        # Front Wall / Pocket Escape Override
        # --------------------------------------------------

        if front_dist < self.front_threshold:

            # Facing a dead end / pocket wall! Steer hard AWAY from the followed wall
            if self.wall_side == "right":

                steering = self.max_steer   # Turn hard LEFT away from right pocket

            else:

                steering = -self.max_steer  # Turn hard RIGHT away from left pocket

            speed = self.turn_speed

            self.get_logger().warn(
                f"⚠️ CORNER / POCKET DETECTED! Front Dist = {front_dist:.2f} m (< {self.front_threshold} m) -> Escaping Pocket",
                throttle_duration_sec=0.4
            )

            self.publish_drive(
                speed,
                steering
            )

            return

        # --------------------------------------------------
        # Normal PID Wall Following
        # --------------------------------------------------

        current_dist, predicted_dist, alpha = self.calculate_wall_distance(msg)

        error = self.target_distance - predicted_dist

        steering = self.pid_control(error, dt)

        if np.abs(steering) > 0.15:

            speed = self.turn_speed

        else:

            speed = self.straight_speed

        self.get_logger().info(
            f"Wall Dist: {current_dist:.2f} m | Front: {front_dist:.2f} m | "
            f"Error: {error:+.2f} m | Steer: {np.degrees(steering):+.1f}° | Speed: {speed:.1f} m/s",
            throttle_duration_sec=0.4
        )

        self.publish_drive(
            speed,
            steering
        )

# ----------------------------------------------------------
# Main
# ----------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = PidWallFollower()

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
