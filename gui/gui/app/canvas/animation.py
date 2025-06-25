import time


class Animation:
    def __init__(self, context):
        self.context    = context
        self.app        = context.app
        self.robot      = context.robot
        self.controller = context.controller

        self.execution_delay = 0

        self.robot_curr_displacement = 0

        self.angle_tolerance = 0.01
        self.distance_tolerance = 1


        self.rotation = 0.0
        self.execute()

    def execute(self):
        curr_pose = self.robot.get_pose()

        if curr_pose is not None: # VERIFY IF ROBOT EXIST IN CANVAS
            goal = self.controller.get_goal_pose()

            self.execution_delay = self.controller.get_execution_delay(self.context.velocity_slider)

            curr_angle = curr_pose['angle']
            # print(f"curr_angle->{curr_angle}")

            if self.context.simulation_running and goal is not None:
                if self.context.fast_mode == 1:
                    self.robot.rotate(goal['angle'])
                    self.robot.displace(goal['distance'])

                    self.context.simulation_running = False
                    self.controller.finish_movement()
                else:
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
                
                    time.sleep(self.execution_delay)            

        self.app.after(1, self.execute)
