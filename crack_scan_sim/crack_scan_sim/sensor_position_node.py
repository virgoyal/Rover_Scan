import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import yaml
import os

class SensorPositionNode(Node):
    def __init__(self):
        super().__init__('sensor_position_node')

        # Declare and load the config path
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found: {config_path}")
            raise FileNotFoundError(config_path)

        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)['plate']

        self.sensor_width = cfg['sensor_width']
        self.sensor_step = cfg['sensor_step']
        self.plate_width = cfg['width']
        self.plate_height = cfg['height']

        self.sensor_position_pub = self.create_publisher(Point, '/sensor_position', 10)
        self.rover_position_sub = self.create_subscription(Point, '/rover_position', self.rover_position_callback, 10)

        self.get_logger().info(f"Sensor sweep width: {self.sensor_width}, step: {self.sensor_step}")
        self.get_logger().info(f"Listening to /rover_position...")

    def rover_position_callback(self, msg):
        rover_x = msg.x
        rover_y = msg.y

        for offset in range(0, self.sensor_width, self.sensor_step):
            sensor_x = rover_x + offset
            if sensor_x >= self.plate_width:
                break  # Skip if out of bounds

            sensor_msg = Point()
            sensor_msg.x = float(sensor_x)
            sensor_msg.y = float(rover_y)

            self.sensor_position_pub.publish(sensor_msg)
            self.get_logger().info(f"Sensor: x={sensor_msg.x}, y={sensor_msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()