#!/usr/bin/env python3
"""Pure Pursuit Waypoint Tracking Agent with Dynamic Lookahead & Speed Profiling.

Implements the official F1TENTH Pure Pursuit geometric path-tracking algorithm:
1. Loads waypoints and speed profile from waypoints.csv.
2. Dynamically scales lookahead distance based on current velocity:
   L_look = clip(0.35 * speed, 0.75m, 1.30m).
3. Transforms the goal point into the car's local body frame.
4. Calculates the steering angle: delta = arctan( 2 * L_wheelbase * sin(alpha) / L_lookahead ).
5. Publishes drive commands to track the raceline with flawless precision.
"""

import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def get_waypoints_path():

    candidates = [
        "/sim_ws/src/ms_agents_pkg/waypoints.csv",
        "/home/ms/sim_ws/src/ms_agents_pkg/waypoints.csv",
        os.path.join(os.getcwd(), "src/ms_agents_pkg/waypoints.csv"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints.csv"),
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "waypoints.csv"),
        os.path.join(os.getcwd(), "waypoints.csv"),
    ]

    for path in candidates:

        if os.path.exists(path):

            return path

    return candidates[0]


class PurePursuitAgent(Node):

    def __init__(self):
        super().__init__("pure_pursuit_agent")

        # --------------------------------------------------
        # ROS setup
        # --------------------------------------------------

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
        # Pure Pursuit Parameters
        # --------------------------------------------------

        # Wheelbase of F1TENTH car (meters)
        self.wheelbase = 0.33

        # Lookahead limits (meters)
        self.min_lookahead = 0.65
        self.max_lookahead = 1.25
        self.lookahead_ratio = 0.25  # L = ratio * speed

        # Default speed (m/s)
        self.default_speed = 3.5

        # Maximum steering angle limits (~24 degrees)
        self.max_steer = 0.4189

        # Steering smoothing filter
        self.smoothing_alpha = 0.70
        self.prev_steering = 0.0

        # --------------------------------------------------
        # Load Waypoints from CSV
        # --------------------------------------------------

        self.csv_path = get_waypoints_path()
        self.waypoints = self.load_waypoints(self.csv_path)

        self.get_logger().info(
            f"Pure Pursuit Initialized! Loaded {len(self.waypoints)} waypoints from: {self.csv_path}"
        )

    # ------------------------------------------------------
    # Load waypoints helper [x, y, speed]
    # ------------------------------------------------------

    def load_waypoints(self, csv_file):

        if not os.path.exists(csv_file):

            self.get_logger().error(f"Waypoint file not found: {csv_file}!")
            return np.empty((0, 3))

        points = []

        with open(csv_file, "r") as f:

            reader = csv.reader(f)
            header = next(reader, None)

            for row in reader:

                if len(row) >= 3:

                    points.append([float(row[0]), float(row[1]), float(row[2])])

                elif len(row) >= 2:

                    points.append([float(row[0]), float(row[1]), self.default_speed])

        return np.array(points)

    # ------------------------------------------------------
    # Publish drive command
    # ------------------------------------------------------

    def publish_drive(self, speed, steering_angle):

        msg = AckermannDriveStamped()

        clamped_steer = float(np.clip(steering_angle, -self.max_steer, self.max_steer))

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = clamped_steer

        self.drive_pub.publish(msg)

    # ------------------------------------------------------
    # Convert Quaternion to Yaw angle (radians)
    # ------------------------------------------------------

    def get_yaw_from_quaternion(self, orientation):

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return float(np.arctan2(siny_cosp, cosy_cosp))

    # ------------------------------------------------------
    # Step 1: Find Goal Waypoint with Anticipatory Braking
    # ------------------------------------------------------

    def find_goal_waypoint(self, car_x, car_y, current_speed):

        # Dynamic lookahead based on speed
        lookahead = float(np.clip(self.lookahead_ratio * max(current_speed, 1.5), self.min_lookahead, self.max_lookahead))

        dx = self.waypoints[:, 0] - car_x
        dy = self.waypoints[:, 1] - car_y
        distances = np.hypot(dx, dy)

        closest_idx = np.argmin(distances)
        num_points = len(self.waypoints)

        target_idx = closest_idx

        # Search forward along the track for the first point >= lookahead distance
        for i in range(num_points):

            idx = (closest_idx + i) % num_points

            if distances[idx] >= lookahead:

                target_idx = idx
                break

        gx = self.waypoints[target_idx, 0]
        gy = self.waypoints[target_idx, 1]

        # Anticipatory Braking: Check minimum speed ahead
        target_speed = self.waypoints[target_idx, 2]

        for offset in range(1, 18):

            ahead_idx = (target_idx + offset) % num_points
            target_speed = min(target_speed, self.waypoints[ahead_idx, 2])

        return gx, gy, target_speed, lookahead

    # ------------------------------------------------------
    # Step 2: Transform Goal Point to Car's Local Body Frame
    # ------------------------------------------------------

    def transform_to_local_frame(self, gx, gy, car_x, car_y, car_yaw):

        dx = gx - car_x
        dy = gy - car_y

        x_local = dx * np.cos(car_yaw) + dy * np.sin(car_yaw)
        y_local = -dx * np.sin(car_yaw) + dy * np.cos(car_yaw)

        return x_local, y_local

    # ------------------------------------------------------
    # Step 3: Compute Pure Pursuit Steering Angle
    # ------------------------------------------------------

    def compute_pure_pursuit_steering(self, x_local, y_local, lookahead):

        actual_lookahead = np.hypot(x_local, y_local)

        if actual_lookahead < 1e-4:

            return 0.0

        alpha = np.arctan2(y_local, x_local)

        # Pure Pursuit Formula: delta = arctan( 2 * L_wheelbase * sin(alpha) / L_lookahead )
        steering_angle = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), actual_lookahead)

        return float(steering_angle)

    # ------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------

    def odom_callback(self, msg: Odometry):

        if len(self.waypoints) == 0:

            return

        car_x = float(msg.pose.pose.position.x)
        car_y = float(msg.pose.pose.position.y)
        car_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        current_speed = float(msg.twist.twist.linear.x)

        # 1. Find target lookahead point & target speed
        gx, gy, target_speed, lookahead = self.find_goal_waypoint(car_x, car_y, current_speed)

        # 2. Transform goal to vehicle local coordinates
        x_local, y_local = self.transform_to_local_frame(gx, gy, car_x, car_y, car_yaw)

        # 3. Calculate steering angle
        steering = self.compute_pure_pursuit_steering(x_local, y_local, lookahead)

        # 4. Status log
        self.get_logger().info(
            f"Car: ({car_x:+.2f}, {car_y:+.2f}) | Steer: {np.degrees(steering):+5.1f}° | "
            f"Speed: {target_speed:.1f} m/s | L_look: {lookahead:.2f} m",
            throttle_duration_sec=0.4
        )

        # 5. Publish drive command
        self.publish_drive(
            target_speed,
            steering
        )


def main(args=None):

    rclpy.init(args=args)

    node = PurePursuitAgent()

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
