#!/usr/bin/env python3
"""Spline Smoother & Curvature Velocity Profiler.

1. Loads raw waypoints from waypoints.csv.
2. Fits a smooth cubic B-spline through the points to remove human steering jitter.
3. Calculates curvature kappa at each point along the curve.
4. Computes the optimal cornering speed profile: v = sqrt(a_lat_max / kappa).
5. Overwrites waypoints.csv with the optimized raceline (and backs up raw data).
"""

import os
import csv
import shutil
import numpy as np
from scipy import interpolate


def smooth_and_profile_waypoints():

    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    csv_path = os.path.join(pkg_dir, "waypoints.csv")
    backup_path = os.path.join(pkg_dir, "waypoints_raw.csv")

    if not os.path.exists(csv_path):

        print(f"❌ Error: {csv_path} does not exist. Please run waypoint_logger first!")
        return

    # Backup raw points
    shutil.copyfile(csv_path, backup_path)
    print(f"📦 Backed up raw points to: {backup_path}")

    # Load raw waypoints
    raw_points = []
    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) >= 2:
                raw_points.append([float(row[0]), float(row[1])])

    raw_points = np.array(raw_points)

    if len(raw_points) < 10:

        print("❌ Error: Not enough waypoints recorded (< 10 points). Drive a full lap first!")
        return

    x = raw_points[:, 0]
    y = raw_points[:, 1]

    # Close the loop if start and end are close
    if np.hypot(x[0] - x[-1], y[0] - y[-1]) > 0.5:

        x = np.append(x, x[0])
        y = np.append(y, y[0])

    # 1. Cubic B-Spline interpolation
    # s controls smoothing factor (higher = smoother curve)
    tck, u = interpolate.splprep([x, y], s=len(x) * 0.05, per=True)

    # Sample 400 evenly distributed points along the track
    u_fine = np.linspace(0, 1, 400)
    x_smooth, y_smooth = interpolate.splev(u_fine, tck)

    # 2. Compute 1st and 2nd derivatives for curvature calculation
    dx, dy = interpolate.splev(u_fine, tck, der=1)
    ddx, ddy = interpolate.splev(u_fine, tck, der=2)

    # 3. Path Curvature: kappa = |x' y'' - y' x''| / (x'^2 + y'^2)^(3/2)
    curvature = np.abs(dx * ddy - dy * ddx) / np.power(dx**2 + dy**2, 1.5)

    # 4. Curvature-Based Velocity Profiling
    # Max lateral acceleration tire grip (m/s^2)
    a_lat_max = 4.5
    max_speed = 5.5
    min_speed = 2.0

    # Optimal speed: v = sqrt(a_lat_max / kappa)
    speeds = np.sqrt(a_lat_max / (curvature + 1e-4))
    speeds = np.clip(speeds, min_speed, max_speed)

    # 5. Save optimized raceline to waypoints.csv
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "speed"])
        for px, py, pv in zip(x_smooth, y_smooth, speeds):
            writer.writerow([f"{px:.4f}", f"{py:.4f}", f"{pv:.2f}"])

    print("==================================================")
    print("🏁 RACELINE OPTIMIZATION COMPLETE!")
    print(f"  • Smoothed Points: {len(x_smooth)}")
    print(f"  • Max Speed on Straights: {np.max(speeds):.2f} m/s")
    print(f"  • Min Speed in Corners:   {np.min(speeds):.2f} m/s")
    print(f"  • Saved optimized raceline to: {csv_path}")
    print("==================================================")


def main(args=None):
    smooth_and_profile_waypoints()


if __name__ == "__main__":
    main()

