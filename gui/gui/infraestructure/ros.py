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

        self._ros_params     = None
        self.goal_pose       = None
        self.light_data      = None
        self.lidar_data      = None
        self._movement_execution = Event()
        self._get_params_cli = self.create_client(GetParams, 'get_params')
        self.set_params_cli  = self.create_client(SetParams, 'set_params')
        self.get_scan_srv    = self.create_service(GetScan, 'get_scan', self._lidar_readings_res)
        self.get_lights_srv  = self.create_service(GetLightReadings, 'get_light_readings', 
                                                    self._light_readings_res)
        self._action_server = ActionServer(self, GoToPose, 'go_to_pose',
                                                        self._execute_movement_callback)

        while not self._get_params_cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().warn('SERVICE /get_params NOT AVAILABLE, WAITING AGAIN...')

        while not self.set_params_cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().warn('SERVICE /set_params NOT AVAILABLE, WAITING AGAIN...')

        self.create_timer(0.01, self._update_ros_params)

    def _update_ros_params(self):
        req = GetParams.Request()
        params_future = self._get_params_cli.call_async(req)
        params_future .add_done_callback(self._handle_response)
    
    def _handle_response(self, future):
        try:
            response = future.result()
            self._ros_params = response
        except Exception as e:
            self._ros_params = None

    def get_ros_params(self):
        return self._ros_params

    def request_ros_params(self):
        req = GetParams.Request()
        future = self._get_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_state_params(self, params):
        req = SetParams.Request()
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

    def finish_movement(self):
        self._movement_execution.set()

    def _execute_movement_callback(self, goal_handle):
        self.goal_pose = goal_handle.request
        self._movement_execution.clear()
        self._movement_execution.wait(timeout = 5)

        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = True
        self.goal_pose = None
        return result

    def get_goal_pose(self):
        return self.goal_pose

    def set_light_data(self, light_data):
        self.light_data = light_data

    def set_lidar_data(self, lidar_data):
        self.lidar_data = lidar_data

    def _light_readings_res(self, request, response):
        if self.light_data:
            response.readings  = self.light_data['readings']
            response.max_index = self.light_data['max_index']
            response.max_value = self.light_data['max_value']
        return response

    def _lidar_readings_res(self, request, response):
        if self.lidar_data:
            response.scan      = self.lidar_data['readings']
            response.angle_min = self.lidar_data['angle_min']
            response.angle_max = self.lidar_data['angle_max']
            response.max_value = self.lidar_data['max_value']
        return response
