import rclpy
import time
from interfaces.srv import *
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.action import GoToPose

class Ros(Node):
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('INITIALIZING SIMULATOR NODE...')
        self.goal_pose = None
        self.movement_executing = False
        self.get_params_cli = self.create_client(GetParams, 'get_params')
        self.set_params_cli = self.create_client(SetParams, 'set_params')
        self.get_lights_srv = self.create_service(GetLightReadings, 'get_light_readings', 
                                                    self.update_light_readings)
        self.get_scan_srv   = self.create_service(GetScan, 'get_scan', self.get_scan)
        
        self._action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            self.execute_movement_callback
        )

        while not self.get_params_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SERVICE /get_params NOT AVAILABLE, WAITING AGAIN...')

        while not self.set_params_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SERVICE /set_params NOT AVAILABLE, WAITING AGAIN...')

        time.sleep(0.1)
        self.update_params()

    def update_params(self):
        req = GetParams.Request()
        future = self.get_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def stop_movement(self):
        # self.get_logger().info('STOPPING MOVEMENTS...')
        self.movement_executing = False

    def execute_movement_callback(self, goal_handle):
        # self.get_logger().info('EXECUTING GOAL...')
        feedback_msg = GoToPose.Feedback()

        self.movement_executing = True
        self.goal_pose = {
                            "angle": goal_handle.request.angle,
                            "distance": goal_handle.request.distance
                        }

        while self.movement_executing:
            # self.get_logger().info('WAITING UNTIL MOVEMENT IS DONE...')
            feedback_msg.feedback = "WAITING UNTIL MOVEMENT IS DONE..."
            goal_handle.publish_feedback(feedback_msg)

        feedback_msg.feedback = "MOVEMENT IS DONE"
        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = True
        self.goal_pose = None
        self.get_logger().info('MOVEMENT FINISHED')
        return result

    def run_simulation(self, params):
        req                 = SetParams.Request()
        req.behavior        = params['behavior']
        req.run_behavior    = params['run_behavior']
        req.step            = params['step']
        req.max_steps       = params['max_steps']
        req.max_advance     = params['max_advance']
        req.max_turn_angle  = params['max_turn_angle']
        req.light_threshold = params['light_threshold']
        req.laser_threshold = params['laser_threshold']
        future = self.set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_goal_pose(self):
        return self.goal_pose

    def update_light_readings(self, request, response):
        response.readings  = [1, 0, 0, 0, 0, 0, 0, 0]
        response.max_index = 0
        response.max_value = 1.0
        return response

    def get_scan(self, request, response):
        response.scan = [1, 1, 1]
        return response