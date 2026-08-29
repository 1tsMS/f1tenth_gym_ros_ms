import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class DemoAgent(Node):
    def __init__(self):
        super().__init__('demo_agent')
        # The two-agent bridge steps only after both drive topics receive commands.
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.opponent_publisher = self.create_publisher(
            AckermannDriveStamped, '/opp_drive', 10)
        
        # 2. Set a timer to execute 'timer_callback' 20 times a second (0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # Publish both commands so this demo works with one or two agents.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 1.5           # Drive forward at 1.5 meters per second
        drive_msg.drive.steering_angle = 0.3  # Steer left at 0.3 radians
        self.publisher.publish(drive_msg)
        self.opponent_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DemoAgent()
    
    # Keep the node running endlessly
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()