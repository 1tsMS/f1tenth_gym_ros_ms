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

        self.max_speed = 2.0
        self.min_speed = 0.5

        # Start turning when something is closer than this
        self.front_trigger_distance = 1.0

        # Hard emergency distance
        self.stop_distance = 0.25

        self.left_steer = 0.55
        self.right_steer = -0.55

    # ------------------------------------------------------
    # Publish drive command
    # ------------------------------------------------------

    def publish_drive(self, speed, steering_angle):

        msg = AckermannDriveStamped()

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(msg)

    # ------------------------------------------------------
    # Clean LiDAR data
    # ------------------------------------------------------

    def get_valid_ranges(self, scan_msg):

        ranges = np.asarray(
            scan_msg.ranges,
            dtype=np.float32
        )

        valid = np.isfinite(ranges)

        valid &= ranges >= scan_msg.range_min
        valid &= ranges <= scan_msg.range_max

        return ranges, valid

    # ------------------------------------------------------
    # Choose direction
    # ------------------------------------------------------

    def choose_direction(self, ranges, valid):

        if not np.any(valid):
            return "stop", 0.0

        center = len(ranges) // 2

        # --------------------------------------------------
        # IMPORTANT:
        #
        # LaserScan angle increases counter-clockwise.
        #
        # negative angles = RIGHT
        # positive angles = LEFT
        #
        # Therefore:
        #
        # ranges BEFORE center = RIGHT
        # ranges AFTER center   = LEFT
        # --------------------------------------------------

        side_width = len(ranges) // 6

        # RIGHT
        right_start = max(0, center - side_width)
        right_end = center

        # LEFT
        left_start = center
        left_end = min(len(ranges), center + side_width)

        # --------------------------------------------------
        # Forward sector
        #
        # Only look at a narrow region directly ahead.
        # --------------------------------------------------

        front_width = 40

        front_start = max(0, center - front_width)
        front_end = min(len(ranges), center + front_width)

        right_ranges = ranges[right_start:right_end]
        right_valid = valid[right_start:right_end]

        left_ranges = ranges[left_start:left_end]
        left_valid = valid[left_start:left_end]

        front_ranges = ranges[front_start:front_end]
        front_valid = valid[front_start:front_end]

        # --------------------------------------------------
        # Calculate clearances
        #
        # Use MAXIMUM distance because we want to know
        # which direction has the most open space.
        # --------------------------------------------------

        right_clearance = np.max(
            np.where(
                right_valid,
                right_ranges,
                20.0
            )
        )

        left_clearance = np.max(
            np.where(
                left_valid,
                left_ranges,
                20.0
            )
        )

        # Closest obstacle directly ahead
        front_distance = np.min(
            np.where(
                front_valid,
                front_ranges,
                20.0
            )
        )

        # --------------------------------------------------
        # Debug information
        # --------------------------------------------------

        self.get_logger().info(
            f"front={front_distance:.2f} m | "
            f"left={left_clearance:.2f} m | "
            f"right={right_clearance:.2f} m"
        )

        # --------------------------------------------------
        # Nothing in front
        # --------------------------------------------------

        if front_distance > self.front_trigger_distance:

            return "center", 0.0

        # --------------------------------------------------
        # Obstacle in front
        #
        # Choose the side with more open space.
        # --------------------------------------------------

        if right_clearance > left_clearance:

            return "right", self.right_steer

        else:

            return "left", self.left_steer

    # ------------------------------------------------------
    # LiDAR callback
    # ------------------------------------------------------

    def scan_callback(self, msg):

        ranges, valid = self.get_valid_ranges(msg)

        if not np.any(valid):

            self.publish_drive(
                0.0,
                0.0
            )

            return

        # --------------------------------------------------
        # Find closest LiDAR return
        # --------------------------------------------------

        nearest = np.min(
            np.where(
                valid,
                ranges,
                20.0
            )
        )

        direction, steering = self.choose_direction(
            ranges,
            valid
        )

        # --------------------------------------------------
        # Emergency stop
        #
        # Only stop if something is extremely close.
        # --------------------------------------------------

        if nearest < self.stop_distance:

            self.get_logger().warn(
                f"EMERGENCY STOP - nearest={nearest:.2f} m"
            )

            self.publish_drive(
                0.0,
                0.0
            )

            return

        # --------------------------------------------------
        # Normal driving
        # --------------------------------------------------

        if direction == "center":

            speed = self.max_speed

        elif direction == "left":

            speed = self.max_speed * 0.7

        elif direction == "right":

            speed = self.max_speed * 0.7

        else:

            speed = 0.0
            steering = 0.0

        # --------------------------------------------------
        # Publish immediately.
        #
        # No steering smoothing for now.
        #
        # This makes debugging easier:
        #
        # right decision -> immediately -0.55
        # left decision  -> immediately +0.55
        # --------------------------------------------------

        self.publish_drive(
            speed,
            steering
        )

    # ------------------------------------------------------
    # Main
    # ------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = SimpleLidarAvoider()

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