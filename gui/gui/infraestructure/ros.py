import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.srv import GetParam
from interfaces.action import GoToPose

class Ros(Node):
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('INITIALIZING SIMULATOR NODE...')
        self.goal_angle    = 0.0
        self.goal_distance = 0.0
        self.movement_executing = False
        self.timer = threading.Timer(10, self.stop_movement)
        self.client = self.create_client(GetParam, 'get_param')
        self._action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            self.execute_callback
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SERVICE /go_to_pose NOT AVAILABLE, WAITING AGAIN...')
        self.request = GetParam.Request()

    def get_param(self, param):
        self.request.param = param
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

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

    def get_max_turn_angle(self):
        return str(0.7857)

    def get_current_step(self):
        return str(0)

    def get_max_steps(self):
        return str(100)

    def get_goal_point(self):
        return { 'x': 10, 'y': 10, 'angle': 1.0 }