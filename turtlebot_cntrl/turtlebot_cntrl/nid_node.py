import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Path

class NIDNode(Node):
    def __init__(self):
        super().__init__('nid_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path, '/path', self.path_callback, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)
        self.target_pose = None
        self.current_idx = -1
        self.path = None
        self.l = 0.04
        self.Kp = 1.0
        self.L_inv = np.array([[1, 0], [0, 1/self.l]])
        self.tolerance = 0.025
        self.get_logger().info('NID Node has been started.')

    def control_loop(self):
        if self.path is None or self.current_idx >= len(self.path.poses):
            self.get_logger().info('No path or robot pose set, or reached the end of the path.')
            return  # No path or robot pose set, or reached the end of the path
        if self.current_idx == -1:
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            return
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        theta = self.get_yaw_from_quaternion(transform.transform.rotation)
        twist_msg = self.kinematic_model(x, y, theta)
        self.vel_pub.publish(twist_msg)

    def path_callback(self, msg: Path):
        self.path = msg
        self.current_idx = 0
        if self.path.poses:
            self.target_pose = self.path.poses[self.current_idx].pose

    def get_yaw_from_quaternion(self, q):
        """
        Convert quaternion to yaw angle.
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def get_rotation_matrix(self, theta):
        """
        Get the 2D rotation matrix for a given angle theta.
        """
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta),  np.cos(theta)]])

    def get_look_ahead_points(self, x, y, theta):
        p_xg, p_yg = self.target_pose.position.x, self.target_pose.position.y
        p_xl = np.cos(theta)*self.l + x
        p_yl = np.sin(theta)*self.l + y
        return p_xl, p_yl, p_xg, p_yg

    def kinematic_model(self, x, y, theta):
        p_xl, p_yl, p_xg, p_yg = self.get_look_ahead_points(x, y, theta)

        distance_to_target = np.sqrt((p_xg - p_xl)**2 + (p_yg - p_yl)**2)
        if distance_to_target < self.tolerance:
            # Increment target index and update target pose
            self.current_idx += 1
            if self.current_idx < len(self.path.poses):
                self.target_pose = self.path.poses[self.current_idx].pose
            else:
                self.get_logger().info('Reached the end of the path.')
                self.target_pose = None
                return Twist() # No more targets
        p_xl, p_yl, p_xg, p_yg = self.get_look_ahead_points(x, y, theta)
        e_x = p_xg - p_xl
        e_y = p_yg - p_yl
        p_dot_x = self.Kp * e_x
        p_dot_y = self.Kp * e_y
        control_inputs = self.L_inv @ self.get_rotation_matrix(theta).transpose() @ np.array([[p_dot_x], [p_dot_y]])
        v, w = control_inputs[0, 0], control_inputs[1, 0]
        v = np.clip(v, -2.0, 2.0)
        w = np.clip(w, -1.8, 1.8)
        twist_msg = Twist()
        twist_msg.linear.x = float(v)
        twist_msg.angular.z = float(w)
        self.get_logger().info(
            f"ex={e_x:.3f}, ey={e_y:.3f}, theta={theta:.3f}, v={v:.3f}, w={w:.3f}"
        )
        return twist_msg


def main(args=None):
    rclpy.init(args=args)
    node = NIDNode()
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
