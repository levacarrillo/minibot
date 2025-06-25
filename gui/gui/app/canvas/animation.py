class Animation:
    def __init__(self, context):
        # self.robot_curr_displacement = 0
        # self.angle_tolerance = 0.01
        # self.distance_tolerance = 1
        self.run()

    def run(self):
        print("TODO")

        # goal = self.controller.get_goal_pose()
        # curr_angle = self.robot.get_pose()['angle']

        # if self.buttons_section.simulation_running and goal is not None:
        #     delta = goal['angle'] - curr_angle
        #     if abs(delta) > self.angle_tolerance:
        #         direction = 1 if delta > 0 else -1 # 1: LEFT, -1: RIGHT
        #         self.robot.rotate(direction)
        #     else:
        #         delta = abs(goal['distance']) - self.robot_curr_displacement
        #         if abs(delta) >= self.distance_tolerance:
        #             direction = 1 if goal['distance'] > 0 else -1 # 1: FORWARD, 2: BACKWARD
        #             self.robot_curr_displacement += self.robot.displace(direction)
        #         else:
        #             self.robot_curr_displacement = 0
        #             self.buttons_section.simulation_running = False
        #             self.controller.finish_movement()

        # self.app.after(1, self.robot_animation)