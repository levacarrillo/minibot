class AppContext:
    def __init__(self, app, content, service, ros):
        self.app     = app
        self.content = content
        self.service = service
        self.ros     = ros

        self.canvas   = None
        self.radians  = None
        self.distance = None

        self.linear_vel  = 0.2
        self.angular_vel = 0.3

        self.status_panel    = None
        self.draw_panel      = None
        self.cmd_vel_panel   = None
        self.cmd_pose_panel  = None
        self.behaviors_panel = None

    def set_status_panel(self, status_panel):
        self.status_panel = status_panel

    def set_draw_panel(self, draw_panel):
        self.draw_panel = draw_panel
        self.canvas = draw_panel.canvas

    def set_cmd_vel_panel(self, cmd_vel_panel):
        self.cmd_vel_panel = cmd_vel_panel

    def set_cmd_pose_panel(self, cmd_pose_panel):
        self.cmd_pose_panel = cmd_pose_panel

    def set_behaviors_panel(self, behaviors_panel):
        self.behaviors_panel = behaviors_panel

    def on_click_start(self):
        self.ros.send_goal(self.radians, self.distance)

    def on_click_stop(self):
        self.ros.cancel_goal()

    def move_robot(self, movement):
        if movement == 'LEFT':
            self.ros.pub_vel(0.0,  self.angular_vel)
        elif movement == 'RIGHT':
            self.ros.pub_vel(0.0, -self.angular_vel)
        elif movement == 'STOP':
            self.ros.pub_vel(0.0, 0.0)
        elif movement == 'FORWARD':
            self.ros.pub_vel(self.linear_vel, 0.0)
        elif movement == 'BACKWARD':
            self.ros.pub_vel(-self.linear_vel, 0.0)
        else: 
            print(f'MOVEMENT {movement} DOES NOT RECOGNIZED')

    def format_linear_vel(self, *args):
        linear_vel = self.app.linear_vel_var.get().replace("m/s", "")
        self.linear_vel = self.service.format_vel(vel)
        self.app.linear_vel_var.set(linear_vel + "m/s")

    def format_angular_vel(self, *args):
        angular_vel = self.app.angular_vel_var.get().replace("rad/s", "")
        self.angular_vel = self.service.format_vel(angular_vel)
        self.app.angular_vel_var.set(angular_vel + "rad/s")


    def format_angle(self, *args):
        degrees = self.cmd_pose.angle_var.get().replace("°", "")
        self.cmd_pose.angle_var.set(degrees + "°")
        self.radians = self.service.degrees_to_radians(degrees)
        self.cmd_pose.radian_var.set(f"{self.radians:.4f} rad")

    def format_distance(self, *args):
        distance_cm = self.cmd_pose.distance_var.get().replace("cm", "") + "cm"
        self.distance = self.service.cm_to_m(distance_cm.replace("cm", ""))
        self.cmd_pose.distance_var.set(distance_cm)

    def get_light_max_intensity(self):
        response = self.ros.get_light_readings()
        return response.max_index if response else None

    def get_lidar_readings(self):
        response = self.ros.get_lidar_readings()
        if response:
            max_value = 0.0
            lidar_norm = []
            for i in range(len(response.scan)):
                lidar_norm.append(response.scan[i] / 0.3)
                if response.scan[i] > max_value:
                    max_value = response.scan[i]

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
        self.draw_panel.clear()
        id_max = self.get_light_max_intensity()
        lidar_readings = self.get_lidar_readings()
        
        for i in range(8):
            coords = self.service.get_spot_light_coords(i, -1.5708, self.draw_panel.edge)
            light = 'yellow' if id_max == i else None   
            self.draw_panel.plot_spot_light(coords, color = light)

        if lidar_readings:
            for i in range(len(lidar_readings)):
                coords = self.service.get_laser_coords(i, lidar_readings[i], len(lidar_readings), self.draw_panel.edge)
                self.draw_panel.plot_laser(coords)

        self.app.after(50, self.loop)