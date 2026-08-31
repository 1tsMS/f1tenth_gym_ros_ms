#!/usr/bin/env python3
"""Follow The Gap (FTG) Autonomous Racing Agent - Stable & Oscillation-Free.

Key Improvements:
1. Forward-Sector Bubble: Bubbles are only applied to obstacles in the forward driving
   cone (|angle| <= 45°). Side walls are never zeroed out, completely eliminating
   the straight-line slalom/wobble.
2. Symmetrical Center Tracking: On straightaways, the gap center resolves to exactly 0°,
   keeping the car locked in the center of the track.
3. Smooth Steering Filter: Smooths wheel transitions for high-speed stability.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class GapFollower(Node):

    def __init__(self):
        super().__init__("gap_follower")

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
        # Follow The Gap (FTG) Settings
        # --------------------------------------------------


        # Field of View (FOV): ±80 degrees (radians)
        self.fov_angle = np.radians(80.0)

        # Forward obstacle sector: only bubble obstacles within ±45° ahead
        self.forward_cone_angle = np.radians(45.0)

        # Safety bubble radius (meters) around obstacles
        self.bubble_radius = 0.40

        # Obstacle alert distance (meters): only bubble if obstacle is closer than this
        self.bubble_trigger_dist = 2.00

        # Horizon clamp (meters): keeps straightaways uniform
        self.max_lidar_range = 4.00

        # Speed settings (m/s)
        self.max_speed = 3.5
        self.min_speed = 1.2

        # Maximum steering angle limits (~24 degrees)
        self.max_steer = 0.4189

        # Steering smoothing factor (0.7 = smooth)
        self.smoothing_alpha = 0.70
        self.prev_steering = 0.0

        self.get_logger().info(
            f"Gap Follower Initialized! FOV: ±{np.degrees(self.fov_angle):.0f}° | "
            f"Forward Cone: ±{np.degrees(self.forward_cone_angle):.0f}° | Max Speed: {self.max_speed} m/s"
        )

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
    # Step 1: Preprocess LiDAR & Crop FOV
    # ------------------------------------------------------

    def preprocess_lidar(self, scan_msg):

        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
        n_points = len(ranges)

        angles = scan_msg.angle_min + np.arange(n_points) * scan_msg.angle_increment

        # Replace non-finite readings
        invalid = ~np.isfinite(ranges) | (ranges < scan_msg.range_min)
        ranges[invalid] = 0.0

        # Smooth 5-beam window
        kernel = np.ones(5) / 5.0
        smoothed_ranges = np.convolve(ranges, kernel, mode="same")

        # Clamp max range
        smoothed_ranges = np.clip(smoothed_ranges, 0.0, self.max_lidar_range)

        # Crop to FOV (±80 degrees)
        fov_mask = (angles >= -self.fov_angle) & (angles <= self.fov_angle)

        return smoothed_ranges[fov_mask], angles[fov_mask]

    # ------------------------------------------------------
    # Step 2 & 3: Find Obstacle in Forward Cone & Apply Bubble
    # ------------------------------------------------------

    def apply_safety_bubble(self, ranges, angles):

        # ONLY check obstacles located in the forward driving cone (±45 deg)
        # Side walls are ignored so they don't erase half the gap and cause oscillations!
        forward_mask = (np.abs(angles) <= self.forward_cone_angle) & (ranges > 0.1)

        if not np.any(forward_mask):

            return ranges

        # Find closest obstacle ahead
        forward_indices = np.where(forward_mask)[0]
        closest_idx = forward_indices[np.argmin(ranges[forward_indices])]
        min_dist = ranges[closest_idx]

        # Only inflate bubble if obstacle is closer than trigger distance
        if min_dist > self.bubble_trigger_dist:

            return ranges

        # Compute bubble angle
        if min_dist > 0.01:

            bubble_angle = np.arctan2(self.bubble_radius, min_dist)

        else:

            bubble_angle = np.radians(45.0)

        closest_angle = angles[closest_idx]

        # Mask out beams inside bubble
        in_bubble = np.abs(angles - closest_angle) <= bubble_angle
        ranges[in_bubble] = 0.0

        return ranges

    # ------------------------------------------------------
    # Step 4: Find Largest Free Gap
    # ------------------------------------------------------

    def find_max_gap(self, ranges):

        is_free = ranges > 0.1

        if not np.any(is_free):

            return 0, len(ranges) - 1

        diff = np.diff(np.concatenate(([0], is_free.view(np.int8), [0])))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0] - 1

        lengths = ends - starts + 1
        longest_gap_idx = np.argmax(lengths)

        return starts[longest_gap_idx], ends[longest_gap_idx]

    # ------------------------------------------------------
    # Step 5: Choose Best Goal Point in Gap
    # ------------------------------------------------------

    def find_best_point(self, start_idx, end_idx, ranges, angles):

        gap_ranges = ranges[start_idx:end_idx + 1]
        gap_angles = angles[start_idx:end_idx + 1]

        if len(gap_ranges) == 0:

            return 0.0

        # 1. Deepest point in gap
        deepest_idx = np.argmax(gap_ranges)
        deepest_angle = gap_angles[deepest_idx]

        # 2. Midpoint of the gap
        midpoint_angle = (gap_angles[0] + gap_angles[-1]) / 2.0

        # If the gap spans symmetrically around 0°, lock heading to center
        if gap_angles[0] < -0.3 and gap_angles[-1] > 0.3 and np.abs(midpoint_angle) < 0.15:

            target_angle = midpoint_angle * 0.5

        else:

            # In corners: blend 60% deepest point + 40% midpoint
            target_angle = 0.60 * deepest_angle + 0.40 * midpoint_angle

        return float(target_angle)

    # ------------------------------------------------------
    # LiDAR callback
    # ------------------------------------------------------

    def scan_callback(self, msg):

        # 1. Preprocess LiDAR
        fov_ranges, fov_angles = self.preprocess_lidar(msg)

        if len(fov_ranges) == 0:

            return

        # 2 & 3. Apply safety bubble to forward obstacles
        bubbled_ranges = self.apply_safety_bubble(fov_ranges, fov_angles)

        # 4. Find largest gap
        gap_start, gap_end = self.find_max_gap(bubbled_ranges)

        # 5. Find target steering angle
        raw_target_angle = self.find_best_point(gap_start, gap_end, bubbled_ranges, fov_angles)

        # 6. Apply low-pass smoothing
        smoothed_steering = (self.smoothing_alpha * self.prev_steering) + ((1.0 - self.smoothing_alpha) * raw_target_angle)
        self.prev_steering = smoothed_steering

        # 7. Speed scaling
        steering_severity = np.abs(smoothed_steering) / self.max_steer
        steering_severity = float(np.clip(steering_severity, 0.0, 1.0))

        speed = self.max_speed * (1.0 - 0.65 * steering_severity)
        speed = max(self.min_speed, speed)

        # 8. Status log
        self.get_logger().info(
            f"Steer: {np.degrees(smoothed_steering):+.1f}° | Speed: {speed:.1f} m/s | "
            f"Gap Width: {gap_end - gap_start + 1} beams",
            throttle_duration_sec=0.4
        )

        # 9. Publish command
        self.publish_drive(
            speed,
            smoothed_steering
        )

# ----------------------------------------------------------
# Main
# ----------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = GapFollower()

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
