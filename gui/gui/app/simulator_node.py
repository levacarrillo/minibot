import rclpy
from rclpy.node import Node

class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator_node')
        self.get_logger().info('INITIALIZING SIMULATOR_NODE')