import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Number of LiDAR array indices to set to 0 around an obstacle
        self.bubble_radius = 160 

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # 1. Preprocess: clip max distance and ignore data behind the car
        ranges = np.clip(ranges, 0.0, 3.0)
        start_idx = len(ranges) // 4
        end_idx = 3 * len(ranges) // 4
        ranges[:start_idx] = 0.0
        ranges[end_idx:] = 0.0
        
        # 2. Find closest obstacle
        non_zero = np.where(ranges > 0.0)[0]
        if len(non_zero) == 0:
            return
        closest_idx = non_zero[np.argmin(ranges[non_zero])]
        
        # 3. Draw safety bubble
        min_idx = max(0, closest_idx - self.bubble_radius)
        max_idx = min(len(ranges), closest_idx + self.bubble_radius)
        ranges[min_idx:max_idx] = 0.0
        
        # 4. Find the max gap
        non_zeros = ranges > 0.0
        edges = np.diff(non_zeros.astype(int))
        starts = np.where(edges == 1)[0] + 1
        ends = np.where(edges == -1)[0]
        
        if non_zeros[0]:
            starts = np.insert(starts, 0, 0)
        if non_zeros[-1]:
            ends = np.append(ends, len(ranges) - 1)
            
        if len(starts) == 0:
            return
            
        gap_lengths = ends - starts
        max_gap_idx = np.argmax(gap_lengths)
        start_i = starts[max_gap_idx]
        end_i = ends[max_gap_idx]
        
        # 5. Find the furthest point inside the gap and steer towards it
        gap = ranges[start_i:end_i]
        if len(gap) == 0:
            return
            
        target_idx = start_i + np.argmax(gap)
        steering_angle = msg.angle_min + target_idx * msg.angle_increment
        
        # Publish commands
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 2.0
        drive_msg.drive.steering_angle = steering_angle
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()