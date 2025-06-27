import time


class Animation:
    def __init__(self, context):
        self.context    = context
        self.app        = context.app
        self.robot      = context.robot
        self.light      = context.light
        self.controller = context.controller


        self.angle_tolerance = 0.01
        self.distance_tolerance = 1
        
        self.delay = None
        self.angle_increment = 0.0
        self.displacement_increment = 0

        self.execute()

    def execute(self):
        curr_pose = self.robot.get_pose()
        if curr_pose is not None:   # VERIFY IF ROBOT EXISTS IN CANVAS
            goal = self.controller.get_goal_pose()
            self.delay = self.controller.get_execution_delay(self.context.velocity_slider)

            if self.light.get_position() is not None:
                self.controller.simulate_light_readings(self.robot.get_pose(), 
                                                self.robot.radius, self.light.get_position())

            if self.context.simulation_running and goal is not None:
                if self.context.fast_mode == 1:
                    self.robot.rotate(goal['angle'])
                    self.robot.displace(goal['distance'])
                    self.controller.finish_movement()
                else:
                    delta = goal['angle'] - self.angle_increment
                    if abs(delta) > self.angle_tolerance:
                        direction = 1 if goal['angle'] > 0 else -1 # 1: LEFT, -1: RIGHT
                        increment = self.controller.degrees_to_radians(direction)
                        self.robot.rotate(increment)
                        self.angle_increment += increment
                    else:
                        delta = goal['distance'] - self.displacement_increment
                        if abs(delta) >= self.distance_tolerance:
                            increment = 1 if goal['distance'] > 0 else -1
                            self.robot.displace(increment)
                            self.displacement_increment += increment
                        else:
                            self.angle_increment         = 0.0
                            self.displacement_increment  = 0
                            self.controller.finish_movement()
                
                    time.sleep(self.delay)            

        self.app.after(1, self.execute)
