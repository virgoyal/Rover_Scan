# crack_scan_sim/matrix_visualization_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

class MatrixVisualizationNode(Node):
    def __init__(self):
        super().__init__('matrix_visualization_node')

        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('refresh_interval', 1)

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.refresh_interval = self.get_parameter('refresh_interval').get_parameter_value().integer_value

        self.grid = np.full((self.height, self.width), np.nan)
        self.msg_count = 0
        self.max_x_seen = 0
        self.max_y_seen = 0

        self.subscription = self.create_subscription(
            Point,
            '/sensor_value',
            self.sensor_callback,
            10
        )

        # Initialize plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.im = self.ax.imshow(self.grid, cmap='Greys', vmin=6, vmax=12, origin='upper')
        self.colorbar = self.fig.colorbar(self.im)
        self.texts = {}

        self.ax.set_title("Sensor Matrix - Live Fill Simulation")
        self.ax.set_xlabel("Width (cm)")
        self.ax.set_ylabel("Height (cm)")

        # Timer-based redraw every 0.5 seconds
        self.create_timer(0.5, self.redraw)



    def sensor_callback(self, msg: Point):
        x = int(msg.x)
        y = int(msg.y)
        z = int(msg.z)

        # Sanity check
        if not (0 <= x < self.width and 0 <= y < self.height):
            self.get_logger().warn(f"Received out-of-bounds point: x={x}, y={y}")
            return

        self.grid[y, x] = z
        self.max_x_seen = max(self.max_x_seen, x)
        self.max_y_seen = max(self.max_y_seen, y)
        self.msg_count += 1

        # Update plot every N messages
        if self.msg_count % self.refresh_interval == 0:
            self.redraw()

    def redraw(self):
        self.im.set_data(self.grid)

        # Update axes to fit actual data
        self.ax.set_xlim(-0.5, self.max_x_seen + 0.5)
        self.ax.set_ylim(self.max_y_seen + 0.5, -0.5)

        # Clear previous labels
        for text in self.texts.values():
            text.remove()
        self.texts.clear()

        # Add number labels to cells
        for y in range(self.max_y_seen + 1):
            for x in range(self.max_x_seen + 1):
                val = self.grid[y, x]
                if not np.isnan(val):
                    self.texts[(x, y)] = self.ax.text(
                        x, y, str(int(val)), ha='center', va='center', fontsize=6, color='black' if val < 9 else 'white'
                    )

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = MatrixVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.redraw()  # final redraw
        plt.ioff()
        plt.show()
        rclpy.shutdown()