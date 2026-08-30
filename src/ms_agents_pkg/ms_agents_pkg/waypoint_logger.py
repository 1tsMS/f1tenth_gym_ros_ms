#!/usr/bin/env python3
"""Waypoint Logger Node.

Subscribes to /ego_racecar/odom and records (x, y, speed) coordinates into a CSV file
whenever the car travels more than a set distance (e.g. 0.15m).
"""

import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class WaypointLogger(Node):

    def __init__(self):
        super().__init__("waypoint_logger")

        # --------------------------------------------------
        # ROS setup
        # --------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.odom_callback,
            10
        )

        # --------------------------------------------------
        # File & Logging settings
        # --------------------------------------------------

        # Save CSV file inside the package directory
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.csv_path = os.path.join(pkg_dir, "waypoints.csv")

        # Minimum distance traveled before saving a new point (meters)
        self.min_dist_threshold = 0.15

        # Last saved position
        self.last_x = None
        self.last_y = None
        self.points_logged = 0

        # Open file and write CSV header
        self.file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["x", "y", "speed"])

        self.get_logger().info(
            f"Waypoint Logger Started! Saving to: {self.csv_path}\n"
            f"Drive the car around the track once (e.g. with gap_follower or teleop)."
        )

    # ------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------

    def odom_callback(self, msg: Odometry):

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        speed = float(msg.twist.twist.linear.x)

        # First waypoint
        if self.last_x is None:

            self.last_x = x
            self.last_y = y
            self.writer.writerow([f"{x:.4f}", f"{y:.4f}", f"{speed:.2f}"])
            self.points_logged += 1

            return

        # Calculate distance traveled since last logged point
        dist = np.hypot(x - self.last_x, y - self.last_y)

        # Only record if the car has moved at least min_dist_threshold
        if dist >= self.min_dist_threshold:

            self.last_x = x
            self.last_y = y
            self.writer.writerow([f"{x:.4f}", f"{y:.4f}", f"{speed:.2f}"])
            self.file.flush()
            self.points_logged += 1

            self.get_logger().info(
                f"Logged {self.points_logged} waypoints | Current: x={x:.2f}, y={y:.2f}",
                throttle_duration_sec=1.0
            )

    # ------------------------------------------------------
    # Clean shutdown
    # ------------------------------------------------------

    def close_file(self):

        if self.file and not self.file.closed:

            self.file.close()

            self.get_logger().info(
                f"✅ Finished! Successfully saved {self.points_logged} waypoints to {self.csv_path}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = WaypointLogger()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.close_file()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()

