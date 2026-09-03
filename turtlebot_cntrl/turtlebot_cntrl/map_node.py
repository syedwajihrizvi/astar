import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self.declare_parameter('resolution', 0.0125)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self._map_w = 5.0
        self._map_h = 5.0
        self._wall_thickness = 0.08
        self._resolution = self.get_parameter('resolution').value

        self._width = int(self._map_w / self._resolution)
        self._height = int(self._map_h / self._resolution)

        self._origin_x = -2.5
        self._origin_y = -2.5

        self.map = None
        self.published = False
        self.timer = self.create_timer(3.0, self.control_loop)

    def parameter_callback(self, params):

        for param in params:

            if param.name == 'resolution':
                self._resolution = param.value

                self.get_logger().info(
                    f'Map resolution changed to {self._resolution}'
                )

                self._width = int(self._map_w / self._resolution)
                self._height = int(self._map_h / self._resolution)

                self.map = self.create_map()
                self.publisher.publish(self.map)

        return SetParametersResult(successful=True)

    def control_loop(self):
        if not self.published:
            self.get_logger().info('Publishing map...')
            self.map = self.create_map()
            self.publisher.publish(self.map)
            self.published = True

    def create_map(self):
        msg = OccupancyGrid()

        msg.header.frame_id = 'map'

        msg.info.resolution = self._resolution
        msg.info.width = self._width
        msg.info.height = self._height

        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = [0] * (self._width * self._height)

        self.add_vertical_wall(msg, -0.75, -0.25, 1.75)
        self.add_vertical_wall(msg, 0.75, -0.25, 1.75)
        self.add_horizontal_wall(msg, -0.25, -0.75, 0.75)
        self.add_horizontal_wall(msg, 1.75, -0.75, 0.75)

        self.add_horizontal_wall(msg, 0.15, -0.72, 0.33)
        self.add_horizontal_wall(msg, 0.55, -0.33, 0.72)
        self.add_horizontal_wall(msg, 0.95, -0.72, 0.33)
        self.add_horizontal_wall(msg, 1.35, -0.33, 0.72)

        return msg

    def convert_gz_to_occupancy_grid(self, gz_x, gz_y):
        og_x = int((gz_x - self._origin_x) / self._resolution)
        og_y = int((gz_y - self._origin_y) / self._resolution)

        return og_x, og_y

    def set_occupied(self, msg, grid_x, grid_y):
        index = grid_y * self._width + grid_x
        msg.data[index] = 100

    def add_vertical_wall(self, msg, x, y_min, y_max):
        half_t = self._wall_thickness / 2.0

        x_min = x - half_t
        x_max = x + half_t

        gx_min, gy_min = self.convert_gz_to_occupancy_grid(x_min, y_min)
        gx_max, gy_max = self.convert_gz_to_occupancy_grid(x_max, y_max)

        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                if 0 <= gx < self._width and 0 <= gy < self._height:
                    self.set_occupied(msg, gx, gy)

    def add_horizontal_wall(self, msg, y, x_min, x_max):
        half_t = self._wall_thickness / 2.0

        y_min = y - half_t
        y_max = y + half_t

        gx_min, gy_min = self.convert_gz_to_occupancy_grid(x_min, y_min)
        gx_max, gy_max = self.convert_gz_to_occupancy_grid(x_max, y_max)

        for gy in range(gy_min, gy_max + 1):
            for gx in range(gx_min, gx_max + 1):
                if 0 <= gx < self._width and 0 <= gy < self._height:
                    self.set_occupied(msg, gx, gy)


def main(args=None):
    rclpy.init(args=args)

    node = MapPublisher()

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