#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SimpleLidarAvoider(Node):
    def __init__(self):
        super().__init__("simple_lidar_avoider")

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
        # Driving settings
        # --------------------------------------------------
        self.max_speed = 1.8
        self.min_speed = 0.5

        self.stop_distance = 1.0
        self.safe_distance = 2.0

        self.left_steer = 0.55
        self.right_steer = -0.55

        self.prev_steering = 0.0


    # ------------------------------------------------------
    # Helper function: publish command
    # ------------------------------------------------------
    def publish_drive(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)


    # ------------------------------------------------------
    # Helper function: clean LiDAR data
    # ------------------------------------------------------
    def get_valid_ranges(self, scan_msg):
        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)

        valid = np.isfinite(ranges)
        valid &= (ranges > 0.2)
        valid &= (ranges < 20.0)

        return ranges, valid


    # ------------------------------------------------------
    # Helper function: get a sector of lidar data
    # ------------------------------------------------------
    def get_sector(self, ranges, valid, start_index, end_index):
        start_index = max(0, start_index)
        end_index = min(len(ranges), end_index)

        sector_ranges = np.full(end_index - start_index, 20.0, dtype=np.float32)
        sector_valid = np.zeros(end_index - start_index, dtype=bool)

        if start_index < end_index:
            sector_ranges[:] = ranges[start_index:end_index]
            sector_valid[:] = valid[start_index:end_index]

        return sector_ranges, sector_valid


    # ------------------------------------------------------
    # Helper function: choose safest direction
    # ------------------------------------------------------
    def choose_direction(self, ranges, valid):
        if not np.any(valid):
            return "stop", 0.0

        center_index = len(ranges) // 2

        left_width = len(ranges) // 6
        right_width = len(ranges) // 6

        left_start = max(0, center_index - left_width)
        left_end = center_index

        right_start = center_index
        right_end = min(len(ranges), center_index + right_width)

        center_start = max(0, center_index - 20)
        center_end = min(len(ranges), center_index + 20)

        left_ranges, left_valid = self.get_sector(ranges, valid, left_start, left_end)
        right_ranges, right_valid = self.get_sector(ranges, valid, right_start, right_end)
        center_ranges, center_valid = self.get_sector(ranges, valid, center_start, center_end)

        left_open = np.min(np.where(left_valid, left_ranges, 100.0))
        right_open = np.min(np.where(right_valid, right_ranges, 100.0))
        center_open = np.min(np.where(center_valid, center_ranges, 100.0))

        # --------------------------------------------------
        # Emergency stop if obstacle is too close
        # --------------------------------------------------
        nearest = np.min(np.where(valid, ranges, 100.0))
        if nearest < self.stop_distance:
            if left_open > right_open:
                return "left", self.left_steer
            else:
                return "right", self.right_steer

        # --------------------------------------------------
        # Otherwise choose the direction with the most space
        # --------------------------------------------------
        if center_open >= left_open and center_open >= right_open:
            return "center", 0.0
        elif left_open >= right_open:
            return "left", self.left_steer
        else:
            return "right", self.right_steer


    # ------------------------------------------------------
    # Main callback
    # ------------------------------------------------------
    def scan_callback(self, msg):
        ranges, valid = self.get_valid_ranges(msg)

        if not np.any(valid):
            self.publish_drive(0.0, 0.0)
            return

        nearest = np.min(np.where(valid, ranges, 100.0))

        # --------------------------------------------------
        # If something is very close, stop and turn away
        # --------------------------------------------------
        if nearest < self.stop_distance:
            direction, steering = self.choose_direction(ranges, valid)

            if direction == "left":
                self.publish_drive(0.0, self.left_steer)
            elif direction == "right":
                self.publish_drive(0.0, self.right_steer)
            else:
                self.publish_drive(0.0, 0.0)
            return

        # --------------------------------------------------
        # Choose best direction
        # --------------------------------------------------
        direction, steering = self.choose_direction(ranges, valid)

        # --------------------------------------------------
        # Simple speed logic
        # --------------------------------------------------
        if direction == "center":
            speed = self.max_speed
        else:
            speed = self.max_speed * 0.7

        if abs(steering) > 0.3:
            speed = max(self.min_speed, speed * 0.7)

        # --------------------------------------------------
        # Smooth steering a little
        # --------------------------------------------------
        steering = 0.7 * self.prev_steering + 0.3 * steering
        self.prev_steering = steering

        self.publish_drive(speed, steering)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLidarAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()