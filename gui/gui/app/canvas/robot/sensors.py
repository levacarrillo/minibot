class Sensors:
    def __init__(self, context):
        self.context = context
        self.color   = context.color
        self.robot   = context.robot
        self.canvas  = context.canvas
        self.controller = context.controller

        self.lasers = []
        self.noise = context.sensor_noise

    def plot(self):
        robot_pose   = self.robot.get_pose()
        
        num_sensors  = self.context.get_param('num_sensors')
        origin_angle = self.context.get_param('origin_angle')
        range_sensor = self.context.get_param('range_sensor')
        entry_laser  = self.context.get_param('laser_threshold')

        lidar_max_value  = float(self.controller.m_to_pixels(entry_laser))

        for i in self.lasers:
            self.canvas.delete(i)

        self.lasers = []
        if num_sensors < 2:
            num_sensors = 2
            self.context.panel_update_value('num_sensors', num_sensors)

        step = range_sensor / ( num_sensors - 1 )
        step_angle = float(robot_pose['angle'] + origin_angle)

        polygon_list = self.context.get_polygon_list()

        polygon_points = polygon_list[0]
        laser_readings = []
        self.noise = context.sensor_noise
        for i in range(0, num_sensors):
            laser_vector = self.controller.polar_to_cartesian_point(lidar_max_value, step_angle)
            laser_max_point = self.controller.sum_vectors(robot_pose, laser_vector)
            print(f'\nstep_angle->{step_angle}')
            print(f'\tlidar_max_value->{lidar_max_value}')

            # laser_point = self.controller.get_laser_value(robot_pose, laser_max_point, polygon_points)

            laser = self.controller.get_line_segment(robot_pose, laser_max_point)
            laser_magnitude, laser_angle = self.controller.cartesian_to_polar(laser)
            laser_readings.append(laser_magnitude)
            self.lasers.append(
                self.canvas.create_line(robot_pose['x'],
                                        robot_pose['y'],
                                        laser_max_point[
                                            'x'],
                                        laser_max_point['y'],
                                        fill = self.color['laser'],
                                        tag = 'robot'
                                        )
                )
            step_angle += step
        self.controller.simulate_lidar_readings(1)

    def delete(self):
        for i in self.lasers:
            self.canvas.delete(i)
        self.lasers = []
