import rclpy
from rclpy.node import Node

class Ros(Node):
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('INITIALIZING SIMULATOR NODE...')

    def get_current_step(self):
        return str(0)

    def get_environment_list(self):
        enviroments = ["EMPTY", "HOME", "ARENA 1", "ARENA 2"]
        return enviroments

    def get_behavior_list(self):
        behavior_list = ["LIGHT_FOLLOWER", "SM_DESTINATION", "SM_AVOID_OBSTACLES"]
        return behavior_list