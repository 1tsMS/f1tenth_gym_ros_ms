#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class SimpleDriveAgent(Node):
    def __init__(self):
        super().__init__('simple_drive_agent')

        # publisher: send drive commands to the car
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10,
        )

        # initialize the drive command
        self.speed = 1.0
        self.steering_angle = 0.0

        # publish once every 0.1 sec
        self.timer = self.create_timer(0.1, self.publish_command)

    def publish_command(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDriveAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
