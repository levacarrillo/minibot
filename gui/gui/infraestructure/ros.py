import rclpy
import time
from threading import Event
from interfaces.srv import *
from rclpy.node import Node
from rclpy.action import ActionServer
from interfaces.action import GoToPose


class Ros(Node):
    def __init__(self):
        super().__init__('simulator')
        self.get_logger().info('INITIALIZING SIMULATOR NODE...')

        self.goal_pose       = None
        self.light_readings  = None
        self.movement_execution = Event()
        self.get_params_cli = self.create_client(GetParams, 'get_params')
        self.set_params_cli = self.create_client(SetParams, 'set_params')
        self.get_scan_srv   = self.create_service(GetScan, 'get_scan', self.get_scan)
        self.get_lights_srv = self.create_service(GetLightReadings, 'get_light_readings', 
                                                    self.update_light_readings)
        self._action_server = ActionServer(self, GoToPose, 'go_to_pose',
                                                        self.execute_movement_callback)

        while not self.get_params_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SERVICE /get_params NOT AVAILABLE, WAITING AGAIN...')

        while not self.set_params_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SERVICE /set_params NOT AVAILABLE, WAITING AGAIN...')

    def update_params(self):
        req = GetParams.Request()
        future = self.get_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_simulation(self, req):
        future = self.set_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def finish_movement(self):
        self.movement_execution.set()

    def execute_movement_callback(self, goal_handle):
        self.goal_pose = goal_handle.request
        self.movement_execution.clear()
        self.movement_execution.wait(timeout = 10)

        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = True
        self.goal_pose = None
        return result

    def get_goal_pose(self):
        return self.goal_pose

    def set_light_readings(self, light_readings):
        self.light_readings = light_readings

    def update_light_readings(self, request, response):
        response.readings  = self.light_readings['readings']
        response.max_index = self.light_readings['max_index']
        response.max_value = self.light_readings['max_value']
        return response

    def get_scan(self, request, response):
        response.scan = [1, 1, 1]
        return response
