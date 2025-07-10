import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from interfaces.action import GoToPose


class Ros(Node):
    def __init__(self):
        super().__init__('tests_gui')
        self.get_logger().info('INITIALIZING GUI FOR TESTS NODE...')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_client = ActionClient(self, GoToPose, 'go_to_pose')
        self._goal_handle = None

    def pub_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info('MOVING AT: LINEAR->%f ANGULAR->%f' % (msg.linear.x, msg.angular.z))

    def send_goal(self, angle, distance):
        movement_msg = GoToPose.Goal()
        movement_msg.angle = angle
        movement_msg.distance = distance
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(movement_msg, feedback_callback = self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
