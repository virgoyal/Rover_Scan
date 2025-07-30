import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import yaml
import os

class RoverPositionNode(Node):
    def __init__(self):
        super().__init__('rover_position_node')

        # Declare and read parameter for config path
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found: {config_path}")
            raise FileNotFoundError(config_path)

        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)['plate']

        self.plate_width = cfg['width']
        self.plate_height = cfg['height']
        self.rover_step_x = cfg['rover_step_x']
        self.rover_step_y = cfg['rover_step_y']

        self.x = 0.0  # Starting at bottom-left
        self.y = 0.0
        self.direction_y = 1  # Start moving upward

        self.rover_position_pub = self.create_publisher(Point, '/rover_position', 10)
        self.timer = self.create_timer(0.3, self.update_position)

        self.get_logger().info("Rover position node started.")
        self.get_logger().info(f"Plate size: {self.plate_width} x {self.plate_height}")
        self.get_logger().info(f"Step X: {self.rover_step_x}, Step Y: {self.rover_step_y}")

    def update_position(self):
        # Publish current position
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        self.rover_position_pub.publish(msg)
        self.get_logger().info(f"Rover at x={self.x}, y={self.y}")

        # Move in y-direction (serpentine)
        self.y += self.direction_y * self.rover_step_y

        # Check if we reached top or bottom of the strip
        if self.y >= self.plate_height or self.y < 0:
            self.direction_y *= -1
            self.y += self.direction_y * self.rover_step_y  # Adjust after reversal
            self.x += self.rover_step_x  # Move to next strip

            if self.x >= self.plate_width:
                self.get_logger().info("✅ Finished scanning entire plate.")
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = RoverPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()