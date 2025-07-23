class Sensors:
    def __init__(self, context):
        self.context = context
        self.robot   = context.robot
        context.set_sensors(self)

    def plot(self, laser_readings = None):
        robot_angle     = self.context.get_context_param('angle')
        robot_radius    = self.context.get_context_param('radius')
        num_sensors     = self.context.get_context_param('num_sensors')
        origin_angle    = self.context.get_context_param('origin_angle')
        range_sensor    = self.context.get_context_param('range_sensor')
        lidar_max_value = self.context.get_context_param('laser_threshold')

        position = self.robot.get_position()
        
        step_angle = robot_angle + origin_angle
        step = range_sensor / ( num_sensors - 1 )

        # laser_vector = self.controller.polar_to_cartesian_point(lidar_max_value, step_angle)
        # laser_max_point = self.controller.sum_vectors(robot_pose, laser_vector)
        # polygon_list = self.context.get_polygon_list()
        # laser_point = self.controller.get_laser_value(robot_pose, laser_max_point, polygon_points)
        # laser = self.controller.get_line_segment(robot_pose, laser_max_point)

        # self.controller.simulate_lidar_readings(1)
        for i in range(0, num_sensors):
            angle = step_angle + i * step
            radius_x, radius_y = self.context.polar_to_cartesian(robot_radius, angle)
            laser_x,  laser_y  = self.context.polar_to_cartesian(lidar_max_value, angle)
            laser = [position['x'] + radius_x, position['y'] + radius_y, 
                     position['x'] + laser_x , position['y'] + laser_y]
            self.context.canvas.create_line(
                laser,
                fill = self.context.color['laser'],
                tags = ('robot', 'laser'))

    def delete(self):
        self.context.canvas.delete('laser')
