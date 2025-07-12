class AppContext:
    def __init__(self, app, content, controller):
        self.app        = app
        self.content    = content
        self.controller = controller
        self.canvas = None
        self.radians  = None
        self.distance = None

        self.linear_vel  = 0.2
        self.angular_vel = 0.3

        self.status   = None
        self.draw_panel   = None
        self.cmd_vel  = None
        self.cmd_pose = None
        self.behaviors = None


    def set_draw_panel(self, draw_panel):
        self.draw_panel = draw_panel
        self.canvas = draw_panel.canvas

    def set_cmd_vel(self, cmd_vel):
        self.cmd_vel = cmd_vel

    def set_cmd_pose(self, cmd_pose):
        self.cmd_pose = cmd_pose


    def on_click_start(self):
        self.controller.send_pose(self.radians, self.distance)

    def on_click_stop(self):
        self.controller.cancel_movement()

    def move_robot(self, movement):
        if movement == 'LEFT':
            self.controller.send_vel(0.0,  self.angular_vel)
        elif movement == 'RIGHT':
            self.controller.send_vel(0.0, -self.angular_vel)
        elif movement == 'STOP':
            self.controller.send_vel(0.0, 0.0)
        elif movement == 'FORWARD':
            self.controller.send_vel(self.linear_vel, 0.0)
        elif movement == 'BACKWARD':
            self.controller.send_vel(-self.linear_vel, 0.0)
        else: 
            print(f'MOVEMENT {movement} DOES NOT RECOGNIZED')

    def format_linear_vel(self, *args):
        linear_vel = self.app.linear_vel_var.get().replace("m/s", "")
        self.linear_vel = self.controller.format_vel(linear_vel)
        self.app.linear_vel_var.set(linear_vel + "m/s")

    def format_angular_vel(self, *args):
        angular_vel = self.app.angular_vel_var.get().replace("rad/s", "")
        self.angular_vel = self.controller.format_vel(angular_vel)
        self.app.angular_vel_var.set(angular_vel + "rad/s")


    def format_angle(self, *args):
        degrees = self.cmd_pose.angle_var.get().replace("°", "")
        self.cmd_pose.angle_var.set(degrees + "°")
        self.radians = self.controller.degrees_to_radians(degrees)
        self.cmd_pose.radian_var.set(f"{self.radians:.4f} rad")

    def format_distance(self, *args):
        distance_cm = self.cmd_pose.distance_var.get().replace("cm", "") + "cm"
        self.distance = self.controller.cm_to_m(distance_cm.replace("cm", ""))
        self.cmd_pose.distance_var.set(distance_cm)

    def get_light_max_intensity(self):
        response = self.controller.get_light_readings()
        return response.max_index if response else None

    def get_lidar_readings(self):
        response = self.controller.get_lidar_readings()
        if response:
            max_value = 0.0
            lidar_norm = []
            for i in range(len(response.scan)):
                # print(f'i->{i} value->{response.scan[i]}')
                lidar_norm.append(response.scan[i] / 0.3)
                if response.scan[i] > max_value:
                    max_value = response.scan[i]
            # print(f'max_value->{max_value}')
            return lidar_norm
        else:
            return None

    def get_body_coords(self, edge):
        coords = [
            edge * (9 / 22),
            edge * (9 / 22),
            edge * (13 / 22),
            edge * (13 / 22)]
        return coords
    
    def get_hokuyo_coords(self, edge):
        coords = [
            edge * (53 / 110),
            edge * (53 / 110),
            edge * (57 / 110),
            edge * (57 / 110)]
        return coords
    
    def get_head_coords(self, edge):
        coords = [
            edge / 2,
            edge / 2 - edge / 11,
            edge / 2 - edge / 11 + 8,
            edge / 2 - 10,
            edge / 2 + edge / 11 - 8,
            edge / 2 - 10]
        return coords

    def loop(self):
        id_max = self.get_light_max_intensity()
        # lidar_readings = self.get_lidar_readings()

        d_sensor = 60
        self.canvas.delete('spot_lights')
        self.canvas.delete('laser')
        

        for i in range(8):
            coords = self.controller.get_spot_light_coords(i, -1.5708, self.draw_panel.edge)
            light = 'yellow' if id_max == i else '#d9d9d9'    
            self.draw_panel.plot_spot_light(coords, color = light)

        # if lidar_readings:
        #     for i in range(len(lidar_readings)):
        #         step_angle = -i * math.pi / len(lidar_readings)
        #         # print(f'angle->{step_angle}')

        #         self.canvas.create_line(110 + self.robot_radius * math.cos(step_angle),
        #                                 110 + self.robot_radius * math.sin(step_angle),
        #                                 110 + d_sensor * (lidar_readings[i] * math.cos(step_angle)),
        #                                 110 + d_sensor * (lidar_readings[i] * math.sin(step_angle)),
        #                                 fill = 'red',
        #                                 tag = 'laser')
        self.app.after(50, self.loop)