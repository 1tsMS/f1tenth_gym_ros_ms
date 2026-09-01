#!/usr/bin/env python3
"""Level 5: Dynamic Overtaking & Trajectory Tracking Agent - Robust State Machine.

Features:
1. RACELINE Tracking by default (identical to Pure Pursuit).
2. OVERTAKING State Machine:
   - When approaching a slower car (< 2.2m ahead), commits to passing lane (Left/Right).
   - Ramps up speed to 4.0 m/s (Overtake Speed Boost).
   - Stays locked in the passing lane without swaying until completely past the lead car.
   - Smoothly re-merges onto the optimal racing line once clear!
3. Dual-role support: Opponent car (/opp_*) by default, or Ego car (/ego_*).
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
        # Pure Pursuit Parameters
        # --------------------------------------------------

        self.wheelbase = 0.33
        self.min_lookahead = 0.65
        self.max_lookahead = 1.25
        self.lookahead_ratio = 0.25
        self.default_speed = 3.5
        self.max_steer = 0.4189

        # Steering smoothing filter
        self.smoothing_alpha = 0.70
        self.prev_steering = 0.0

        # --------------------------------------------------
        # Overtaking State Machine
        # --------------------------------------------------

        self.state = "TRACKING"             # "TRACKING" or "OVERTAKING"
        self.overtake_side = 1.0            # +1.0 = Left lane, -1.0 = Right lane
        self.overtake_start_time = 0.0
        self.min_overtake_duration = 1.6    # Minimum seconds to hold pass before re-merging
        self.detection_dist = 2.20          # Distance to start overtaking (meters)
        self.overtake_lateral_offset = 0.65 # Passing lane offset (meters)
        self.overtake_speed = 4.0           # Speed boost during pass (m/s)

        self.latest_scan = None

        # --------------------------------------------------
        # Load Waypoints from CSV
        # --------------------------------------------------

        self.csv_path = get_waypoints_path()
        self.waypoints = self.load_waypoints(self.csv_path)

        self.get_logger().info(
            f"Dynamic Overtaker Initialized as {role}!\n"
            f"• Waypoints: {len(self.waypoints)} loaded from: {self.csv_path}\n"
            f"• Overtake Boost: {self.overtake_speed} m/s | Detection: {self.detection_dist} m"
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
    # LiDAR Callback
    # ------------------------------------------------------

    def scan_callback(self, msg: LaserScan):

        self.latest_scan = msg

    # ------------------------------------------------------
    # Step 1: Find Goal Waypoint with Anticipatory Braking
    # ------------------------------------------------------

    def find_goal_waypoint(self, car_x, car_y, current_speed):

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

        target_speed = self.waypoints[target_idx, 2]

        for offset in range(1, 18):

            ahead_idx = (target_idx + offset) % num_points
            target_speed = min(target_speed, self.waypoints[ahead_idx, 2])

        return gx, gy, target_speed, lookahead, target_idx

    # ------------------------------------------------------
    # Step 2: Overtaking State Machine
    # ------------------------------------------------------

    def update_overtaking_state(self):

        if self.latest_scan is None:

            return

        ranges = np.asarray(self.latest_scan.ranges, dtype=np.float32)
        n_points = len(ranges)
        angles = self.latest_scan.angle_min + np.arange(n_points) * self.latest_scan.angle_increment

        # Front corridor mask: ±18 degrees directly ahead
        front_mask = (np.abs(angles) <= np.radians(18.0)) & np.isfinite(ranges) & (ranges > 0.20)
        front_dist = np.min(ranges[front_mask]) if np.any(front_mask) else 99.0

        now = time.monotonic()

        # STATE TRANSITIONS:
        if self.state == "TRACKING":

            # If a car is detected ahead on our line (< detection_dist)
            if front_dist < self.detection_dist:

                # Choose the side with more drivable clearance
                left_mask = (angles > np.radians(15.0)) & (angles <= np.radians(65.0)) & np.isfinite(ranges)
                right_mask = (angles < -np.radians(15.0)) & (angles >= -np.radians(65.0)) & np.isfinite(ranges)

                left_space = np.mean(ranges[left_mask]) if np.any(left_mask) else 0.0
                right_space = np.mean(ranges[right_mask]) if np.any(right_mask) else 0.0

                self.overtake_side = 1.0 if left_space >= right_space else -1.0
                self.state = "OVERTAKING"
                self.overtake_start_time = now

                self.get_logger().info(
                    f"🏎️ OVERTAKE INITIATED! Shifting {'LEFT' if self.overtake_side > 0 else 'RIGHT'} | "
                    f"Ramping speed to {self.overtake_speed} m/s!"
                )

        elif self.state == "OVERTAKING":

            elapsed = now - self.overtake_start_time

            # Only re-merge after minimum passing time AND front path is clear
            if elapsed > self.min_overtake_duration and front_dist > 2.0:

                self.state = "TRACKING"
                self.get_logger().info("🏁 OVERTAKE COMPLETE! Re-merging onto optimal raceline.")

    # ------------------------------------------------------
    # Step 3: Pure Pursuit Steering
    # ------------------------------------------------------

    def compute_pure_pursuit_steering(self, x_local, y_local, lookahead):

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

        # 1. Update Overtaking State Machine
        self.update_overtaking_state()

        # 2. Get baseline raceline goal
        gx, gy, target_speed, lookahead, target_idx = self.find_goal_waypoint(car_x, car_y, current_speed)

        # 3. If in OVERTAKING state, hold passing lane and boost speed!
        if self.state == "OVERTAKING":

            num_points = len(self.waypoints)
            next_idx = (target_idx + 4) % num_points
            tx = self.waypoints[next_idx, 0] - gx
            ty = self.waypoints[next_idx, 1] - gy
            t_norm = np.hypot(tx, ty)

            if t_norm > 1e-4:

                nx = -ty / t_norm
                ny = tx / t_norm

                # Shift waypoint into passing lane
                gx += self.overtake_side * self.overtake_lateral_offset * nx
                gy += self.overtake_side * self.overtake_lateral_offset * ny

                # Power past the moving car with high speed!
                target_speed = max(target_speed, self.overtake_speed)

        # 4. Transform goal to vehicle body coordinates
        dx = gx - car_x
        dy = gy - car_y
        x_local = dx * np.cos(car_yaw) + dy * np.sin(car_yaw)
        y_local = -dx * np.sin(car_yaw) + dy * np.cos(car_yaw)

        # 5. Pure Pursuit Steering & Smoothing
        raw_steering = self.compute_pure_pursuit_steering(x_local, y_local, lookahead)
        smoothed_steering = (self.smoothing_alpha * self.prev_steering) + ((1.0 - self.smoothing_alpha) * raw_steering)
        self.prev_steering = smoothed_steering

        # 6. Corner safety: scale speed if steering hard
        if np.abs(smoothed_steering) > 0.16:

            target_speed = min(target_speed, 2.0)

        # 7. Status log
        self.get_logger().info(
            f"Car: ({car_x:+.2f}, {car_y:+.2f}) | Steer: {np.degrees(smoothed_steering):+5.1f}° | "
            f"Speed: {target_speed:.1f} m/s | State: {self.state}",
            throttle_duration_sec=0.4
        )

        # 8. Publish drive command
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
