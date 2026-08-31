#!/usr/bin/env python3
"""One-Shot Reset Node for F1TENTH Gym.

Resets both Ego and Opponent cars back to their starting grid positions
without needing to restart gym_bridge or RViz.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped


class CarResetter(Node):

    def __init__(self):
        super().__init__("car_resetter")

        self.ego_reset_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )

        self.opp_reset_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10
        )

    def reset(self, ego_x=0.0, ego_y=0.0, opp_x=3.5, opp_y=0.0):

        time.sleep(0.1)

        # 1. Reset Ego Car (at start line)
        ego_msg = PoseWithCovarianceStamped()
        ego_msg.header.stamp = self.get_clock().now().to_msg()
        ego_msg.header.frame_id = "map"
        ego_msg.pose.pose.position.x = float(ego_x)
        ego_msg.pose.pose.position.y = float(ego_y)
        ego_msg.pose.pose.orientation.w = 1.0  # yaw = 0.0 (facing +X)
        self.ego_reset_pub.publish(ego_msg)

        # 2. Reset Opponent Car (3.5m ahead on track)
        opp_msg = PoseStamped()
        opp_msg.header.stamp = self.get_clock().now().to_msg()
        opp_msg.header.frame_id = "map"
        opp_msg.pose.position.x = float(opp_x)
        opp_msg.pose.position.y = float(opp_y)
        opp_msg.pose.orientation.w = 1.0  # yaw = 0.0 (facing +X)
        self.opp_reset_pub.publish(opp_msg)

        self.get_logger().info(
            f"✅ Cars Reset! Ego: ({ego_x:.1f}, {ego_y:.1f}) | Opponent: ({opp_x:.1f}, {opp_y:.1f})"
        )


def main(args=None):

    rclpy.init(args=args)

    node = CarResetter()

    # Publish reset messages
    for _ in range(5):

        node.reset(ego_x=0.0, ego_y=0.0, opp_x=3.5, opp_y=0.0)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()

