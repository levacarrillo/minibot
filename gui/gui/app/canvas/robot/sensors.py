class Sensors:
    def __init__(self, context):
        self.context = context
        self.color   = context.color
        self.robot   = context.robot
        self.canvas  = context.canvas
        self.controller = context.controller

        self.lasers = []

    def plot(self):
        robot_pose   = self.robot.get_pose()
        
        num_sensors  = self.context.get_param('num_sensors')
        origin_angle = self.context.get_param('origin_angle')
        range_sensor = self.context.get_param('range_sensor')
        entry_laser  = self.context.get_param('laser_threshold')

        lidar_value  = self.controller.m_to_pixels(entry_laser)

        for i in self.lasers:
            self.canvas.delete(i)

        self.lasers = []
        if num_sensors < 2:
            num_sensors = 2
            self.context.panel_update_value('num_sensors', num_sensors)

        step = range_sensor / ( num_sensors - 1 )
        step_angle = robot_pose['angle'] + origin_angle

        for i in range(0, num_sensors):
            # POINT IN ROBOT'S CIRCUNFERENCE
            c_x, c_y = self.controller.polar_to_cartesian(self.robot.radius, step_angle)

            sensor_i  = self.controller.set_position(robot_pose['x'] + c_x,
                                                    robot_pose['y'] + c_y)

            # POINTS FOR READINGS VALUES
            lx, ly = self.controller.polar_to_cartesian(lidar_value, step_angle)
            sensor_value_i = self.controller.set_position(robot_pose['x'] + lx,
                                                        robot_pose['y'] + ly)

        

            self.lasers.append(
                self.canvas.create_line(sensor_i['x'],
                                        sensor_i['y'],
                                        sensor_value_i['x'],
                                        sensor_value_i['y'],
                                        fill = self.color['laser'],
                                        tag = 'robot'
                                        )
                )
            step_angle += step

    def delete(self):
        for i in self.lasers:
            self.canvas.delete(i)
        self.lasers = []
