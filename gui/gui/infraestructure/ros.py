import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.action import GoToPose

class Ros(Node):
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('INITIALIZING SIMULATOR NODE...')
        self.goal_angle    = 0.0
        self.goal_distance = 0.0
        self.movement_executing = False
        self.timer = threading.Timer(10, self.stop_movement)
        self._action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            self.execute_callback
        )

    def stop_movement(self):
        self.movement_executing = False
    
    def movement_executing(self):
        return self.movement_executing


    def execute_callback(self, goal_handle):
        self.get_logger().info('EXECUTING GOAL...')

        print(goal_handle.request.angle)
        print(goal_handle.request.distance)
        self.movement_executing = True
        self.goal_angle    = goal_handle.request.angle
        self.goal_distance = goal_handle.request.distance
        self.timer.start()
        while(self.movement_executing):
            pass
        self.get_logger().info('MOVEMENT FINISHED')
        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = True
        return result

    def get_current_step(self):
        return str(0)

    def get_environment_list(self):
        enviroments = ["EMPTY", "HOME", "ARENA 1", "ARENA 2"]
        return enviroments

    def get_behavior_list(self):
        behavior_list = ["LIGHT_FOLLOWER", "SM_DESTINATION", "SM_AVOID_OBSTACLES"]
        return behavior_list

    def get_goal_point(self):
        return { 'x': 10, 'y': 10, 'angle': 1.0 }