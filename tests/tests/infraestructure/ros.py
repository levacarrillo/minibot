from rclpy.node import Node


class Ros(Node):
    def __init__(self):
        super().__init__('tests_gui')
        self.get_logger().info('INITIALIZING GUI FOR TESTS NODE...')