import rclpy
# import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.action import GoToPose
from interfaces.srv import GetParams, SetParams

class Ros(Node):
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('INITIALIZING SIMULATOR NODE...')
        self.goal_angle    = 0.0
        self.goal_distance = 0.0
        self.movement_executing = False
        # self.timer = threading.Timer(10, self.stop_movement)
        self.client = self.create_client(GetParams, 'get_params')
        self._action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            self.execute_callback
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SERVICE /get_params NOT AVAILABLE, WAITING AGAIN...')

        params = self.req_params()
        self.param_dict = {
            "behavior" : params.behavior,
            "run_behavior" : params.run_behavior,
            "behavior_list" : params.behavior_list,
            "step" : params.step,
            "max_steps" : params.max_steps,
            "max_advance" : params.max_advance,
            "max_turn_angle" : params.max_turn_angle,
            "light_threshold" : params.light_threshold,
            "laser_threshold" : params.laser_threshold
        }

        for key, value in self.param_dict.items():
            print(key, value)

    def req_params(self):
        req = GetParams.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_param(self, param):
        return self.param_dict[param]

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
        # self.timer.start()
        while(self.movement_executing):
            pass
        self.get_logger().info('MOVEMENT FINISHED')
        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = True
        return result

    def get_goal_point(self):
        return { 'x': 10, 'y': 10, 'angle': 1.0 }