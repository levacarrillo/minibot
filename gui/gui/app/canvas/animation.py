import time
from gui.app.canvas.robot.route import Route


class Animation:
    def __init__(self, context):
        self.context    = context
        self.app        = context.app
        self.robot      = context.robot
        self.light      = context.light
        self.controller = context.controller

        self.route = Route(context)

        self.angle_tolerance = 0.01
        self.distance_tolerance = 1
        
        self.delay = None
        self.angle_increment = 0.0
        self.displacement_increment = 0

        self.init_pose = None
        self.execute()
        self.flag = False
    
    def execute(self):
        curr_pose = self.robot.get_pose()
        if curr_pose is not None:   # VERIFY IF ROBOT EXISTS IN CANVAS
            # if self.robot.grasped is None and self.flag is False:
            #     self.flag = self.context.robot.grasp('Block_A')
            #     if self.flag:
            #         print(f'OBJECT GRASPED->{self.robot.grasped}')

            goal = self.controller.get_goal_pose()
            self.delay = self.context.get_execution_delay()

            if self.light.get_position() is not None:
                # METHOD TO SAVE VALUES TO ROS' SERVICE
                self.controller.simulate_light_readings(self.robot.get_pose(), 
                                                self.robot.radius, self.light.get_position())

            if self.context.simulation_running and goal is not None:
                if self.init_pose is None:
                    self.init_pose = curr_pose
                current_step = self.controller.get_current_step()
                self.context.panel_update_value('label_steps', current_step)

                if self.context.fast_mode == 1:
                    self.robot.rotate(goal['angle'])
                    self.robot.displace(goal['distance'])
                    self.controller.finish_movement()
                    final_pose = self.robot.get_pose()
                    self.route.trace(self.init_pose, final_pose)
                    self.init_pose = None
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
                            final_pose = self.robot.get_pose()
                            self.route.trace(self.init_pose, final_pose)
                            self.init_pose = None
                
                    time.sleep(self.delay)

                if self.context.get_param('max_steps') <= current_step or self.controller.get_param('run_behavior') is False:
                    # if self.robot.grasped is not None and self.context.fast_mode == 1:
                    #     self.context.robot.release()
                    self.context.enable_button_run()

        self.app.after(1, self.execute)
