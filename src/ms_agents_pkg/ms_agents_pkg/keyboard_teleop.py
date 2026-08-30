#!/usr/bin/env python3
"""Cruise-Control Interactive Keyboard Teleop for F1TENTH.

Drive effortlessly:
- [W] Sets / increases forward cruise speed (Car stays driving forward!)
- [S] Brakes / Reverses
- [A] Turn Left (Instantly turns wheels left)
- [D] Turn Right (Instantly turns wheels right)
- Releasing A/D: Wheels automatically snap back to straight (0°)
- [Space] or [X]: Immediate Stop (0.0 m/s)
"""

import sys
import select
import termios
import tty
import time
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

HELP_MSG = """
--------------------------------------------------
🏎️  F1TENTH CRUISE TELEOP
--------------------------------------------------
  W : Forward (+0.3 m/s) -> Keeps Cruising!
  S : Brake / Slow Down (-0.5 m/s)
  A : Turn Left (Auto-centers when not pressed)
  D : Turn Right (Auto-centers when not pressed)
  Space / X : Emergency Stop (0.0 m/s)

Press Ctrl+C to exit.
--------------------------------------------------
"""


class CruiseTeleop(Node):

    def __init__(self):
        super().__init__("keyboard_teleop")

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",
            10
        )

        self.speed = 0.0
        self.steering_angle = 0.0

        # Turn angle for responsive steering (~20 degrees)
        self.turn_steer = 0.35

        self.max_speed = 3.0
        self.last_steer_time = 0.0

        # Run control loop at 30Hz
        self.timer = self.create_timer(0.033, self.control_loop)

    def control_loop(self):

        now = time.monotonic()

        # Auto-center steering if no A/D key received within 250ms
        if now - self.last_steer_time > 0.25:

            self.steering_angle = 0.0

        # Publish drive message continuously
        msg = AckermannDriveStamped()
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steering_angle)
        self.drive_pub.publish(msg)


def get_keys(settings):

    tty.setraw(sys.stdin.fileno())
    keys = ""

    while True:

        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)

        if rlist:

            ch = sys.stdin.read(1)
            keys += ch

        else:

            break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return keys


def main(args=None):

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)

    node = CruiseTeleop()
    print(HELP_MSG)

    try:

        while rclpy.ok():

            rclpy.spin_once(node, timeout_sec=0.01)
            keys = get_keys(settings)
            now = time.monotonic()

            if "\x03" in keys:  # Ctrl+C

                break

            if " " in keys or "x" in keys:

                node.speed = 0.0
                node.steering_angle = 0.0

            else:

                # Speed control (Cruise control - maintains speed while steering!)
                if "w" in keys:

                    node.speed = min(node.max_speed, max(1.2, node.speed + 0.3))

                if "s" in keys:

                    node.speed = max(-1.0, node.speed - 0.4)

                # Steering control (Simultaneous with speed)
                if "a" in keys:

                    node.steering_angle = node.turn_steer   # Left
                    node.last_steer_time = now

                elif "d" in keys:

                    node.steering_angle = -node.turn_steer  # Right
                    node.last_steer_time = now

            # Print live HUD
            print(
                f"\r🚗 Cruise Speed: {node.speed:+.2f} m/s | Steer: {np.degrees(node.steering_angle):+5.1f}°   ",
                end="",
                flush=True
            )

    except Exception as e:

        print(e)

    finally:

        node.speed = 0.0
        node.steering_angle = 0.0
        node.control_loop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nExiting Keyboard Teleop.")


if __name__ == "__main__":
    main()
