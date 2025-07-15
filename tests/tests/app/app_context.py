class AppContext:
    def __init__(self, app, content, service, ros):
        self.app     = app
        self.content = content
        self.service = service
        self.ros     = ros

        # FRAMES
        self.status_panel    = None
        self.draw_panel      = None
        self.cmd_vel_panel   = None
        self.cmd_pose_panel  = None
        self.behaviors_panel = None


        # VELOCITY COMMANDS
        self.linear_vel  = None
        self.angular_vel = None

        # POSITION COMMANDS
        self.radians  = None
        self.distance = None

        # BEHAVIOR PARAMS
        self.behavior_list = None

    def set_status_panel(self, status_panel):
        self.status_panel = status_panel

    def set_draw_panel(self, draw_panel):
        self.draw_panel = draw_panel

    def set_cmd_vel_panel(self, cmd_vel_panel):
        self.cmd_vel_panel = cmd_vel_panel
        self.format_linear_vel()
        self.format_angular_vel()

    def set_cmd_pose_panel(self, cmd_pose_panel):
        self.cmd_pose_panel = cmd_pose_panel
        self.format_angle()
        self.format_distance()

    def set_behaviors_panel(self, behaviors_panel):
        self.behaviors_panel = behaviors_panel

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
        linear_vel = self.cmd_vel_panel.linear_vel_var.get().replace("m/s", "")
        self.linear_vel = self.service.validate_vel(linear_vel)
        self.cmd_vel_panel.linear_vel_var.set(linear_vel + "m/s")

    def format_angular_vel(self, *args):
        angular_vel = self.cmd_vel_panel.angular_vel_var.get().replace("rad/s", "")
        self.angular_vel = self.service.validate_vel(angular_vel)
        self.cmd_vel_panel.angular_vel_var.set(angular_vel + "rad/s")

    def format_angle(self, *args):
        degrees = self.cmd_pose_panel.angle_var.get().replace("°", "")
        self.cmd_pose_panel.angle_var.set(degrees + "°")
        self.radians = self.service.degrees_to_radians(degrees)
        self.cmd_pose_panel.radian_var.set(f"{self.radians:.4f} rad")

    def format_distance(self, *args):
        distance_cm = self.cmd_pose_panel.distance_var.get().replace("cm", "") + "cm"
        self.distance = self.service.cm_to_m(distance_cm.replace("cm", ""))
        self.cmd_pose_panel.distance_var.set(distance_cm)

    def on_click_start(self):
        self.ros.send_goal(self.radians, self.distance)

    def on_click_stop(self):
        self.ros.cancel_goal()

    def get_head_coords(self, width):
        return self.service.get_head_coords(width)

    def get_body_coords(self, width):
        return self.service.get_body_coords(width)

    def get_hokuyo_coords(self, width):
        return self.service.get_hokuyo_coords(width)

    def get_lights_max_intensity(self):
        return self.service.validate_lights_response(self.ros.get_light_readings())

    def get_lidar_readings(self):
        return self.service.norm_lidar_response(self.ros.get_lidar_readings())

    def get_mp_params(self):
        params = self.ros.get_mp_params()
        if params is not None:
            self.behavior_list = params.behavior_list
            self.behavior_list.remove('') if '' in self.behavior_list else None
            self.behavior_list.remove('UNKNOWN') if 'UNKNOWN' in self.behavior_list else None
            
            self.behaviors_panel.max_steps.set(params.max_steps)

            self.behaviors_panel.list = self.behavior_list
            self.behaviors_panel.cb_behavior.config(values = self.behavior_list)
            self.behaviors_panel.cb_behavior.current(0)
            # self.behaviors_panel.cb_behavior.set('USER_SM')
            # self.ros.get_logger().warn(f'behavior_list->{self.behavior_list}')
            # self.ros.get_logger().warn(f'behavior->{params.behavior}')
            self.ros.get_logger().warn(f'max_steps->{params.max_steps}')


    def loop(self):
        battery_charge = self.ros.get_battery_charge()
        self.status_panel.progress_var.set(battery_charge)
        self.status_panel.robot_name_var.set(self.ros.get_robot_name())
        self.status_panel.battery_percentage_var.set(f'Battery: {battery_charge}%')

        id_max = self.get_lights_max_intensity()
        lidar_params = self.get_lidar_readings()
        width = self.draw_panel.width
        self.draw_panel.clear()

        for i in range(8):
            coords = self.service.get_spot_light_coords(i, width)
            light = 'yellow' if id_max == i else None   
            self.draw_panel.plot_spot_light(coords, color = light)

        for i in range(lidar_params['num_readings']):
            coords = self.service.get_laser_coords(i, lidar_params, width)
            self.draw_panel.plot_laser(coords)

        if self.behavior_list is None:
            self.get_mp_params()

        self.app.after(50, self.loop)