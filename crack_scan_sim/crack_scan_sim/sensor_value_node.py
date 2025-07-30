# crack_scan_sim/sensor_value_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class SensorValueNode(Node):
    def __init__(self):
        super().__init__('sensor_value_node')

        # Subscribe to sensor positions
        self.subscription = self.create_subscription(
            Point,
            '/sensor_position',
            self.sensor_position_callback,
            10
        )

        # Publish sensor values
        self.publisher = self.create_publisher(Point, '/sensor_value', 10)

        # Internal matrix for basic clustering logic
        self.grid_width = 200
        self.grid_height = 200
        self.grid = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]

        # Probability parameters (local to node, can be tweaked for realism)
        self.prob_6_initial = 0.05
        self.cluster_boost = 0.2

        self.get_logger().info("Sensor Value Node started")

    def sensor_position_callback(self, msg):
        x, y = int(msg.x), int(msg.y)

        if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
            # Check neighbors for clustering boost
            prob = self.prob_6_initial
            if self.has_neighboring_6(x, y):
                prob += self.cluster_boost

            value = 6 if random.random() < prob else 12
            self.grid[y][x] = value

            out_msg = Point()
            out_msg.x = float(x)
            out_msg.y = float(y)
            out_msg.z = float(value)

            self.publisher.publish(out_msg)
            self.get_logger().info(f"Published value: ({x}, {y}) -> {value}")

    def has_neighboring_6(self, x, y):
        neighbors = [
            (x-1, y), (x+1, y),
            (x, y-1), (x, y+1),
            (x-1, y-1), (x-1, y+1),
            (x+1, y-1), (x+1, y+1)
        ]
        for nx, ny in neighbors:
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                if self.grid[ny][nx] == 6:
                    return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = SensorValueNode()
    rclpy.spin(node)
    rclpy.shutdown()