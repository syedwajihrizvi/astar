import rclpy
import math
import numpy as np
from scipy.optimize import minimize
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class SafetyFilterNode(Node):
    def __init__(self):
        super().__init__('safety_filter_node')
        self.latest_scan = None
        self.current_twist = None
        self.sector_size = 20
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        # Need to publish the obstacles
        self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, scan_qos)
        self.obstacle_pub = self.create_publisher(PoseArray, '/obstacles', 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Safety Filter Node has been started.')


    def laser_scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def control_loop(self):
        # Populate the obstacles from the scan data
        if self.latest_scan is not None:
            scan = self.latest_scan
            msg = PoseArray()
            msg.header.frame_id = 'base_scan'
            msg.header.stamp
            # Increments of 20 degrees for each sector
            for start in range(0, len(scan.ranges), self.sector_size):
                end = min(start + self.sector_size, len(scan.ranges))
                closest_range = float('inf')
                closest_idx = None
                for i in range(start, end):
                    r = scan.ranges[i]
                    if not math.isfinite(r):
                        continue
                    if r < scan.range_min or r > scan.range_max:
                        continue
                    if r < closest_range:
                        closest_range = r
                        closest_idx = i
                if closest_idx is None:
                    continue
                angle = scan.angle_min + start * scan.angle_increment
                x = closest_range * math.cos(angle)
                y = closest_range * math.sin(angle)
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                msg.poses.append(pose)
            self.obstacle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


        