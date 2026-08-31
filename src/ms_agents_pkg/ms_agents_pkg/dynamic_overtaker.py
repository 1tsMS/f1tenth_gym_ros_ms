#!/usr/bin/env python3
"""Level 5: Dynamic Overtaking & Trajectory Tracking Agent - Stable & Precise.

Key Features:
1. True Centerline Waypoint Tracking with Anticipatory Braking (Braking Zones).
2. Cartesian Corridor Obstacle Filtering (|y| <= 0.28m) to ignore track walls.
3. Smooth Lateral Frenet Shifting (Left/Right) with Hysteresis Latching.
4. Low-Pass Steering Smoothing to eliminate wheel oscillation.
5. Dual-role support: Opponent car (default) or Ego car via ROS parameter.
"""

import os
import csv
import time
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
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


class DynamicOvertaker(Node):

    def __init__(self):
        super().__init__("dynamic_overtaker")

        # --------------------------------------------------
        # ROS Parameters (Switch between Opponent & Ego)
        # --------------------------------------------------

        self.declare_parameter("is_opponent", True)
        self.is_opponent = self.get_parameter("is_opponent").value

        if self.is_opponent:

            odom_topic = "/opp_racecar/odom"
            scan_topic = "/opp_scan"
            drive_topic = "/opp_drive"
            role = "OPPONENT CAR (/opp_*)"

        else:

            odom_topic = "/ego_racecar/odom"
            scan_topic = "/scan"
            drive_topic = "/drive"
            role = "EGO CAR (/ego_*)"

        # --------------------------------------------------
        # ROS setup
        # --------------------------------------------------

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        # --------------------------------------------------
        # Tracking & Overtaking Parameters
        # --------------------------------------------------

        # Wheelbase of F1TENTH car (meters)
        self.wheelbase = 0.33

        # Dynamic Lookahead limits (meters)
        self.min_lookahead = 0.65
        self.max_lookahead = 1.25
        self.lookahead_ratio = 0.25

        # Maximum steering limits (~24 degrees)
        self.max_steer = 0.4189

        # Steering smoothing filter (0.75 = very smooth)
        self.smoothing_alpha = 0.70
        self.prev_steering = 0.0

        # Obstacle detection parameters (Cartesian corridor)
        self.obstacle_detection_dist = 2.00  # meters ahead
        self.corridor_half_width = 0.28       # meters lateral

        # Lateral shift distance for overtaking (meters)
        self.overtake_lateral_offset = 0.60

        # Overtake state & hysteresis latching
        self.is_overtaking = False
        self.overtake_side = 0.0
        self.last_obstacle_time = 0.0
        self.latch_duration = 1.0

        # Latest scan cache
        self.latest_scan = None

        # --------------------------------------------------
        # Load Waypoints from CSV
        # --------------------------------------------------

        self.csv_path = get_waypoints_path()
        self.waypoints = self.load_waypoints(self.csv_path)

        self.get_logger().info(
            f"Dynamic Overtaker Initialized as {role}!\n"
            f"• Waypoints: {len(self.waypoints)} loaded from {self.csv_path}\n"
            f"• Lookahead: {self.min_lookahead}m - {self.max_lookahead}m"
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

                    points.append([float(row[0]), float(row[1]), 3.5])

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
    # LiDAR Callback
    # ------------------------------------------------------

    def scan_callback(self, msg: LaserScan):

        self.latest_scan = msg

    # ------------------------------------------------------
    # Step 1: Detect Obstacles Using Cartesian Corridor Filtering
    # ------------------------------------------------------

    def check_forward_obstacle(self):

        if self.latest_scan is None:

            return False, 0.0

        ranges = np.asarray(self.latest_scan.ranges, dtype=np.float32)
        n_points = len(ranges)
        angles = self.latest_scan.angle_min + np.arange(n_points) * self.latest_scan.angle_increment

        valid = np.isfinite(ranges) & (ranges > 0.15) & (ranges < self.obstacle_detection_dist)

        if not np.any(valid):

            return False, 0.0

        r = ranges[valid]
        theta = angles[valid]

        x_pts = r * np.cos(theta)
        y_pts = r * np.sin(theta)

        in_corridor = (x_pts > 0.30) & (x_pts < self.obstacle_detection_dist) & (np.abs(y_pts) <= self.corridor_half_width)

        now = time.monotonic()

        if np.any(in_corridor):

            if not self.is_overtaking or (now - self.last_obstacle_time > self.latch_duration):

                left_mask = (angles > np.radians(15.0)) & (angles <= np.radians(65.0)) & np.isfinite(ranges)
                right_mask = (angles < -np.radians(15.0)) & (angles >= -np.radians(65.0)) & np.isfinite(ranges)

                left_clearance = np.mean(ranges[left_mask]) if np.any(left_mask) else 0.0
                right_clearance = np.mean(ranges[right_mask]) if np.any(right_mask) else 0.0

                self.overtake_side = 1.0 if left_clearance >= right_clearance else -1.0
                self.is_overtaking = True

            self.last_obstacle_time = now

            return True, self.overtake_side

        if self.is_overtaking and (now - self.last_obstacle_time < self.latch_duration):

            return True, self.overtake_side

        self.is_overtaking = False

        return False, 0.0

    # ------------------------------------------------------
    # Step 2: Select Lookahead Goal & Apply Anticipatory Braking
    # ------------------------------------------------------

    def get_overtaking_goal(self, car_x, car_y, current_speed):

        lookahead = float(np.clip(self.lookahead_ratio * max(current_speed, 1.5), self.min_lookahead, self.max_lookahead))

        dx = self.waypoints[:, 0] - car_x
        dy = self.waypoints[:, 1] - car_y
        distances = np.hypot(dx, dy)

        closest_idx = np.argmin(distances)
        num_points = len(self.waypoints)

        target_idx = closest_idx

        for i in range(num_points):

            idx = (closest_idx + i) % num_points

            if distances[idx] >= lookahead:

                target_idx = idx
                break

        gx = self.waypoints[target_idx, 0]
        gy = self.waypoints[target_idx, 1]

        # Anticipatory Braking: Check minimum speed in the next 1.5m ahead
        # This ensures the car decelerates BEFORE entering the corner!
        target_speed = self.waypoints[target_idx, 2]

        for offset in range(1, 18):

            ahead_idx = (target_idx + offset) % num_points
            target_speed = min(target_speed, self.waypoints[ahead_idx, 2])

        # Check for dynamic obstacle
        has_obstacle, overtake_side = self.check_forward_obstacle()

        if has_obstacle:

            next_idx = (target_idx + 4) % num_points
            tx = self.waypoints[next_idx, 0] - gx
            ty = self.waypoints[next_idx, 1] - gy
            tangent_norm = np.hypot(tx, ty)

            if tangent_norm > 1e-4:

                tx /= tangent_norm
                ty /= tangent_norm

                nx = -ty
                ny = tx

                gx += overtake_side * self.overtake_lateral_offset * nx
                gy += overtake_side * self.overtake_lateral_offset * ny

                target_speed = min(target_speed, 2.5)

        return gx, gy, target_speed, lookahead

    # ------------------------------------------------------
    # Step 3: Pure Pursuit Arc Steering
    # ------------------------------------------------------

    def compute_steering(self, gx, gy, car_x, car_y, car_yaw, lookahead):

        dx = gx - car_x
        dy = gy - car_y

        x_local = dx * np.cos(car_yaw) + dy * np.sin(car_yaw)
        y_local = -dx * np.sin(car_yaw) + dy * np.cos(car_yaw)

        actual_lookahead = np.hypot(x_local, y_local)

        if actual_lookahead < 1e-4:

            return 0.0

        alpha = np.arctan2(y_local, x_local)

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

        # 1. Goal selection with anticipatory braking
        gx, gy, target_speed, lookahead = self.get_overtaking_goal(car_x, car_y, current_speed)

        # 2. Raw steering angle calculation
        raw_steering = self.compute_steering(gx, gy, car_x, car_y, car_yaw, lookahead)

        # 3. Steering low-pass smoothing
        smoothed_steering = (self.smoothing_alpha * self.prev_steering) + ((1.0 - self.smoothing_alpha) * raw_steering)
        self.prev_steering = smoothed_steering

        # 4. Cornering speed safety
        if np.abs(smoothed_steering) > 0.14:

            target_speed = min(target_speed, 1.8)

        # 5. Status log
        self.get_logger().info(
            f"Car: ({car_x:+.2f}, {car_y:+.2f}) | Steer: {np.degrees(smoothed_steering):+5.1f}° | "
            f"Speed: {target_speed:.1f} m/s | Overtaking: {'YES' if self.is_overtaking else 'NO'}",
            throttle_duration_sec=0.4
        )

        # 6. Publish drive command
        self.publish_drive(
            target_speed,
            smoothed_steering
        )


def main(args=None):

    rclpy.init(args=args)

    node = DynamicOvertaker()

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
