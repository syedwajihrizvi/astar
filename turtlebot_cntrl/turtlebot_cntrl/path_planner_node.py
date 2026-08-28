import rclpy
import heapq
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.goal_pose = None
        self.map = None
        self.grid = None
        self.raw_grid = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.inflated_grid_pub = self.create_publisher(OccupancyGrid, '/inflated_map', qos)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('inflation_radius', 20)
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'inflation_radius':
                self.inflation_radius = param.value
                self.get_logger().info(
                    f'Inflation radius changed to {self.inflation_radius}'
                )
                self.update_inflated_grid()
        return SetParametersResult(successful=True) 

    def inflate_obstacles(self, grid):
        rows, cols = len(grid), len(grid[0])
        inflated_grid = [[0 for _ in range(cols)] for _ in range(rows)]
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] >= 50:
                    for dy in range(-self.inflation_radius, self.inflation_radius + 1):
                        for dx in range(-self.inflation_radius, self.inflation_radius + 1):
                            if dx**2 + dy**2 <= self.inflation_radius**2:
                                inflated_x = c + dx
                                inflated_y = r + dy
                                if (0 <= inflated_x < cols) and (0 <= inflated_y < rows):
                                    inflated_grid[inflated_y][inflated_x] = 100
        return inflated_grid

    def publish_inflated_map(self, grid2d, reference_msg):
        inflated_msg = OccupancyGrid()

        inflated_msg.header.frame_id = 'map'
        inflated_msg.header.stamp = self.get_clock().now().to_msg()

        inflated_msg.info.resolution = reference_msg.info.resolution
        inflated_msg.info.width = reference_msg.info.width
        inflated_msg.info.height = reference_msg.info.height
        inflated_msg.info.origin = reference_msg.info.origin

        inflated_msg.data = [
            value
            for row in grid2d
            for value in row
        ]

        self.get_logger().info(
            f'Publishing /inflated_map: '
            f'{inflated_msg.info.width}x{inflated_msg.info.height}, '
            f'{len(inflated_msg.data)} cells'
        )

        self.inflated_grid_pub.publish(inflated_msg)    

    def map_callback(self, msg):
        self.get_logger().info('Received Map, planning path...')
        self.map = msg
        w, h = self.map.info.width, self.map.info.height
        self.raw_grid = [[0 for _ in range(w)] for _ in range(h)]
        for idx in range(len(self.map.data)):
            x = idx % w
            y = idx // w
            self.raw_grid[y][x] = msg.data[idx]
        # Inflate the obstacles in the grid based on the inflation radius
        self.update_inflated_grid()

    def update_inflated_grid(self):
        if self.raw_grid is None:
            return

        self.grid = self.inflate_obstacles(self.raw_grid)
        self.publish_inflated_map(self.grid, self.map)

        if self.goal_pose is not None:
            path = self.plan_path()
            self.path_pub.publish(path)

    def convert_pose_to_grid(self, gz_x, gz_y):
        """
        Convert Gazebo coordinates to occupancy grid coordinates.
        """
        og_x = int((gz_x - self.map.info.origin.position.x) / self.map.info.resolution)
        og_y = int((gz_y - self.map.info.origin.position.y) / self.map.info.resolution)
        return og_x, og_y

    def convert_grid_to_pose(self, og_x, og_y):
        """
        Convert occupancy grid coordinates to Gazebo coordinates.
        """
        gz_x = (og_x+0.5) * self.map.info.resolution + self.map.info.origin.position.x
        gz_y = (og_y+0.5) * self.map.info.resolution + self.map.info.origin.position.y
        return gz_x, gz_y


    def plan_path(self) -> Path:
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            return Path()  # No path can be planned without robot pose
        if self.goal_pose is None:
            self.get_logger().warn('Goal or Robot pose not set. Cannot plan path.')
            return Path()  # No goal pose set, return empty path
        # Make a grid and fill it with the occupancy data
        robot_x, robot_y = transform.transform.translation.x, transform.transform.translation.y
        w, h = self.map.info.width, self.map.info.height
        # Set start to the Robot position
        start_x, start_y = self.convert_pose_to_grid(robot_x, robot_y)
        # Set goal to the Goal position
        goal_x, goal_y = self.convert_pose_to_grid(self.goal_pose.position.x, self.goal_pose.position.y)
        if not (0 <= start_x < w and 0 <= start_y < h):
            self.get_logger().error(f'Start position ({start_x}, {start_y}) is out of bounds.')
            return Path()
        if not (0 <= goal_x < w and 0 <= goal_y < h):
            self.get_logger().error(f'Goal position ({goal_x}, {goal_y}) is out of bounds.')
            return Path()
        # Check if start or goal is in an occupied cell
        if self.grid[start_y][start_x] >= 50:
            self.get_logger().error(f'Start position ({start_x}, {start_y}) is in an occupied cell.')
            return Path()
        if self.grid[goal_y][goal_x] >= 50:
            self.get_logger().error(f'Goal position ({goal_x}, {goal_y}) is in an occupied cell.')
            return Path()
        start = (start_x, start_y)
        goal = (goal_x, goal_y)
        path = self.astar_algorithm(self.grid, start, goal, w, h)
        return path


    def astar_algorithm(self, grid, start, goal, w, h) -> Path:
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
        def heuristic(a):
            return abs(a[0] - goal[0]) + abs(a[1] - goal[1])  # Manhattan distance
        open_set = []
        visited = set()
        g_map = {start: 0}
        came_from = {}

        heapq.heappush(open_set, (heuristic(start), 0, start))
        while open_set:
            _, g, node = heapq.heappop(open_set)
            if node in visited:
                continue
            visited.add(node)
            if node == goal:
                break
            for dx, dy in directions:
                nx, ny = node[0] + dx, node[1] + dy
                nei = (nx, ny)
                if 0 <= nx < w and 0 <= ny < h and grid[ny][nx] < 50:
                    new_g = g + 1
                    if g_map.get(nei, float('inf')) > new_g:
                        g_map[nei] = new_g
                        f = new_g + heuristic(nei)
                        heapq.heappush(open_set, (f, new_g, nei))
                        came_from[nei] = node
        self.get_logger().info(f'Visited nodes: {len(visited)}')
        if goal not in visited:
            self.get_logger().error('No path found to the goal.')
            return Path()  # No path found
        final_path = []
        curr = goal
        final_path.append(curr)
        while curr in came_from:
            final_path.append(came_from[curr])
            curr = came_from[curr]
        final_path.reverse()
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for og_x, og_y in final_path:
            gz_x, gz_y = self.convert_grid_to_pose(og_x, og_y)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = gz_x
            pose.pose.position.y = gz_y
            pose.pose.orientation.w = 1.0  # No rotation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            path_msg.poses.append(pose)
        return path_msg



    def goal_callback(self, msg:PoseStamped):
        self.get_logger().info(f'Received Goal Pose: {msg.pose.position.x}, {msg.pose.position.y}')
        self.goal_pose = msg.pose
        if self.map is not None:
            path = self.plan_path()
            self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
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