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

        self._goal_pose_handle = None
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._lidar_client = self.create_client(GetScan, 'get_scan')
        self._go_to_pose_action = ActionClient(self, GoToPose, 'go_to_pose')
        self._get_velocity_client = self.create_client(GetVelParams,  'get_vel_params')
        self._set_velocity_client = self.create_client(SetVelParams,  'set_vel_params')
        self._light_client = self.create_client(GetLightReadings, 'get_light_readings')
        self._get_params_motion_planner_client = self.create_client(GetParams, 'get_params')
        self._set_params_motion_planner_client = self.create_client(SetParams, 'set_params')

        self._robot_status_sub = self.create_subscription(RobotStatus, 'robot_status', self._robot_status_callback, 10)
        self._robot_status_sub

        delay = 1.0
        if not self._light_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_light_readings NOT AVAILABLE.')
        if not self._lidar_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_scan NOT AVAILABLE.')
        if not self._get_params_motion_planner_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_params NOT AVAILABLE.')
        if not self._set_params_motion_planner_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /set_params NOT AVAILABLE.')
        if not self._get_velocity_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /get_vel_params NOT AVAILABLE.')
        if not self._set_velocity_client.wait_for_service(timeout_sec = delay):
            self.get_logger().warn('SERVICE /set_vel_params NOT AVAILABLE.')


        self._light_readings = None
        self._lidar_readings = None
        self._motion_planner_params = None

        self._robot_name = None
        self._battery_charge_percentage = None
        self._executing_movement = False

        self.create_timer(0.5, self._request_services)

    def get_vel_params(self):
        get_vel_req = GetVelParams.Request()
        future = self._get_velocity_client.call_async(get_vel_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_vel_params(self, cmd_params):
        set_vel_req = SetVelParams.Request()
        set_vel_req.linear_velocity  = cmd_params['linear_velocity']
        set_vel_req.angular_velocity = cmd_params['angular_velocity']
        future = self._set_velocity_client.call_async(set_vel_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_motion_planner_req(self, params):
        params_req = SetParams.Request()
        params_req.behavior = params['behavior']
        params_req.run_behavior = params['run_behavior']
        params_req.step = 0
        params_req.behavior_list = params['behavior_list']
        params_req.max_steps = params['max_steps']
        params_req.max_advance = params['max_advance']
        params_req.max_turn_angle = params['max_turn_angle']
        future = self._set_params_motion_planner_client.call_async(params_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _robot_status_callback(self, msg):
        self._robot_name = msg.robot_name
        self._battery_charge_percentage = msg.battery_charge_percentage
    
    def get_robot_name(self):
        return self._robot_name

    def get_battery_charge(self):
        return self._battery_charge_percentage

    def _request_services(self):
        lidar_req = GetScan.Request()
        motion_planner_req = GetParams.Request()
        light_req = GetLightReadings.Request()
        light_future = self._light_client.call_async(light_req)
        lidar_future = self._lidar_client.call_async(lidar_req)
        motion_planner_params_future = self._get_params_motion_planner_client.call_async(motion_planner_req)

        light_future.add_done_callback(self._handle_light_response)
        lidar_future.add_done_callback(self._handle_lidar_response)
        motion_planner_params_future.add_done_callback(self._handle_motion_planner_response)

    def _handle_light_response(self, future):
        try:
            response = future.result()
            self._light_readings = response
        except Exception as e:
            self._light_readings = None

    def _handle_lidar_response(self, future):
        try:
            response = future.result()
            self._lidar_readings = response
        except Exception as e:
            self._lidar_readings = None

    def _handle_motion_planner_response(self, future):
        try:
            response = future.result()
            self._motion_planner_params = response
        except Exception as e:
            self._motion_planner_params = None

    def get_light_readings(self):
        return self._light_readings

    def get_lidar_readings(self):
        return self._lidar_readings

    def get_behavior_running(self):
        if self._motion_planner_params is not None:
            return self._motion_planner_params.run_behavior
        return False

    def get_current_step(self):
        if self._motion_planner_params is not None:
            return self._motion_planner_params.step
        return 0

    def get_motion_planner_params(self):
        return self._motion_planner_params

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._pub_vel.publish(msg)
        self.get_logger().info('MOVING AT: LINEAR->%f ANGULAR->%f' % (msg.linear.x, msg.angular.z))

    def send_goal_pose(self, pose):
        movement_msg = GoToPose.Goal()
        movement_msg.angle = pose['angle']
        movement_msg.distance = pose['distance']
        self._go_to_pose_action.wait_for_server()
        self._executing_movement = True
        send_goal_pose_future = self._go_to_pose_action.send_goal_async(movement_msg, feedback_callback = self._feedback_callback)
        send_goal_pose_future.add_done_callback(self._goal_pose_response_callback)

    def movement_is_executing(self):
        return self._executing_movement

    def _goal_pose_response_callback(self, future):
        self._goal_pose_handle = future.result()
        if not self._goal_pose_handle.accepted:
            self.get_logger().info('GOAL REJECTED')
            return

        self._get_result_future = self._goal_pose_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self._executing_movement = False

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def cancel_goal(self):
        if self._goal_pose_handle:
            self.get_logger().info('CANCELING GOAL...')
            cancel_future = self._goal_pose_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == 0:
            self.get_logger().info('GOAL CANCELED SUCCESSFULLY')
        else:
            self.get_logger().info('FAILED TO CANCEL GOAL')
