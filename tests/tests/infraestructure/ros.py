import rclpy
from rclpy.node import Node
from interfaces.msg import *
from interfaces.srv import *
from interfaces.action import *
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient


class Ros(Node):
    def __init__(self):
        super().__init__('tests_gui')
        self.get_logger().info('INITIALIZING GUI FOR TESTS NODE...')
        self._goal_handle = None
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_client = ActionClient(self, GoToPose, 'go_to_pose')
        self.light_client = self.create_client(GetLightReadings, 'get_light_readings')
        self.lidar_client = self.create_client(GetScan, 'get_scan')
        self.mp_client    = self.create_client(GetParams, 'get_params')
        self.params_client = self.create_client(SetParams, 'set_params')
        self.get_vel_client = self.create_client(GetVelParams, 'get_vel_params')
        self.set_vel_client = self.create_client(SetVelParams, 'set_vel_params')

        self.subscription = self.create_subscription(RobotStatus, 'robot_status', self.listener_callback, 10)
        self.subscription

        delay = 0.2
        if not self.light_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_light_readings NOT AVAILABLE.')
        if not self.lidar_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_scan NOT AVAILABLE.')
        if not self.mp_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_params NOT AVAILABLE.')
        if not self.params_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /set_params NOT AVAILABLE.')
        if not self.get_vel_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_vel_params NOT AVAILABLE.')
        if not self.set_vel_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /set_vel_params NOT AVAILABLE.')

        self.light_req = GetLightReadings.Request()
        self.lidar_req = GetScan.Request()
        self.mp_req    = GetParams.Request()
        self.params_req = SetParams.Request()

        self.get_vel_req = GetVelParams.Request()
        self.set_vel_req = SetVelParams.Request()

        self.light_readings = None
        self.lidar_readings = None
        self.mp_params = None

        self.robot_name = None
        self.battery_charge_percentage = None
        self.create_timer(0.5, self.request_services)

        self.executing_movement = False

    def get_vel_params(self):
        future = self.get_vel_client.call_async(self.get_vel_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_vel_params(self, cmd_params):
        self.set_vel_req.linear_velocity  = cmd_params['linear_velocity']
        self.set_vel_req.angular_velocity = cmd_params['angular_velocity']
        future = self.set_vel_client.call_async(self.set_vel_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_request(self, params):
        self.params_req.behavior = params['behavior']
        self.params_req.run_behavior = params['run_behavior']
        self.params_req.step = 0
        self.params_req.behavior_list = params['behavior_list']
        self.params_req.max_steps = params['max_steps']
        self.params_req.max_advance = params['max_advance']
        self.params_req.max_turn_angle = params['max_turn_angle']
        self.get_logger().warn(f'req->{self.params_req}')
        future = self.params_client.call_async(self.params_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def listener_callback(self, msg):
        self.robot_name = msg.robot_name
        self.battery_charge_percentage = msg.battery_charge_percentage
    
    def get_robot_name(self):
        return self.robot_name

    def get_battery_charge(self):
        return self.battery_charge_percentage

    def request_services(self):
        light_future = self.light_client.call_async(self.light_req)
        lidar_future = self.lidar_client.call_async(self.lidar_req)
        mp_future    = self.mp_client.call_async(self.mp_req)

        light_future.add_done_callback(self.handle_light_response)
        lidar_future.add_done_callback(self.handle_lidar_response)
        mp_future.add_done_callback(self.handle_mp_response)

    def handle_light_response(self, future):
        try:
            response = future.result()
            self.light_readings = response
        except Exception as e:
            # self.get_logger().error(f'THERE WAS AN ERROR TO GET LIGHT READINGS: {e}')
            self.light_readings = None

    def handle_lidar_response(self, future):
        try:
            response = future.result()
            self.lidar_readings = response
        except Exception as e:
            # self.get_logger().error(f'THERE WAS AN ERROR TO GET LIDAR READINGS: {e}')
            self.lidar_readings = None

    def handle_mp_response(self, future):
        try:
            response = future.result()
            # self.get_logger().warn(f'response->{response}')
            self.mp_params = response
        except Exception as e:
            # self.get_logger().error(f'THERE WAS AN ERROR TO GET MP PARAMS: {e}')
            self.mp_params = None

    def get_light_readings(self):
        return self.light_readings

    def get_lidar_readings(self):
        return self.lidar_readings

    def get_behavior_running(self):
        if self.mp_params is not None:
            # self.get_logger().info(f'behavior running: {self.mp_params.run_behavior}')
            return self.mp_params.run_behavior
        return False

    def get_current_step(self):
        if self.mp_params is not None:
            # self.get_logger().info(f'current_step: {self.mp_params.step}')
            return self.mp_params.step
        return 0

    def get_mp_params(self):
        # self.get_logger().info(f'params->{self.mp_params}')
        return self.mp_params

    def pub_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info('MOVING AT: LINEAR->%f ANGULAR->%f' % (msg.linear.x, msg.angular.z))

    def send_goal(self, pose):
        movement_msg = GoToPose.Goal()
        movement_msg.angle = pose['angle']
        movement_msg.distance = pose['distance']
        self._action_client.wait_for_server()
        self.executing_movement = True
        self._send_goal_future = self._action_client.send_goal_async(movement_msg, feedback_callback = self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def movement_is_executing(self):
        return self.executing_movement

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('GOAL REJECTED')
            return

        # self.get_logger().info('GOAL ACCEPTED SUCCESSFULLY')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.executing_movement = False
        # self.get_logger().info('RESULT: {0}'.format(result.success))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('RECEIVED FEEDBACK: {0}'.format(feedback.feedback))

    def cancel_goal(self):
        if self._goal_handle:
            self.get_logger().info('CANCELING GOAL...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info('GOAL CANCELED SUCCESSFULLY')
        else:
            self.get_logger().info('FAILED TO CANCEL GOAL')
