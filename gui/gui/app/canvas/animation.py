class Animation:
    def __init__(self, context):
        self.context    = context
        self.app        = context.app
        self.robot      = context.robot
        self.controller = context.controller
        self.robot_curr_displacement = 0
        self.angle_tolerance = 0.01
        self.distance_tolerance = 1
        self.execute()

    def execute(self):
        goal = self.controller.get_goal_pose()
        robot_pose = self.robot.get_pose()
        if robot_pose is not None:
            curr_angle = self.robot.get_pose()['angle']

            if self.context.simulation_running and goal is not None:
                delta = goal['angle'] - curr_angle
                if abs(delta) > self.angle_tolerance:
                    direction = 1 if delta > 0 else -1 # 1: LEFT, -1: RIGHT
                    self.robot.rotate(direction)
                else:
                    delta = abs(goal['distance']) - self.robot_curr_displacement
                    if abs(delta) >= self.distance_tolerance:
                        direction = 1 if goal['distance'] > 0 else -1 # 1: FORWARD, 2: BACKWARD
                        self.robot_curr_displacement += self.robot.displace(direction)
                    else:
                        self.robot_curr_displacement = 0
                        self.context.simulation_running = False
                        self.controller.finish_movement()

        self.app.after(1, self.execute)