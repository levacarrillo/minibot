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
        self.params_pop_up   = None


        # VELOCITY COMMANDS
        self.cmd_vel = {
            'linear_vel': None,
            'angular_vel': None
        }

        # POSITION COMMANDS
        self.cmd_pose = {
            'angle': None,
            'distance': None
        }

        # BEHAVIOR PARAMS
        self.is_executing = False

        self.params = None


    def set_status_panel(self, status_panel):
        self.status_panel = status_panel

    def set_draw_panel(self, draw_panel):
        self.draw_panel = draw_panel

    def set_cmd_vel_panel(self, cmd_vel_panel):
        self.cmd_vel_panel = cmd_vel_panel

    def set_cmd_pose_panel(self, cmd_pose_panel):
        self.cmd_pose_panel = cmd_pose_panel

    def set_behaviors_panel(self, behaviors_panel):
        self.behaviors_panel = behaviors_panel

    def set_params_pop_up(self, params_pop_up):
        self.params_pop_up = params_pop_up
        self.params_pop_up.max_advance.set(self.params['max_advance'])
        self.params_pop_up.max_turn_angle.set(self.params['max_turn_angle'])
        self.params_pop_up.light_threshold.set(self.params['light_threshold'])
        self.params_pop_up.laser_threshold.set(self.params['laser_threshold'])
        self.params_pop_up.max_steps.set(self.params['max_steps'])

    def format_cmd_vel(self, *args):
        linear_vel = self.cmd_vel_panel.linear_vel.get().replace('m/s', '')
        self.cmd_vel['linear_vel'] = self.service.validate_vel(linear_vel)
        self.cmd_vel_panel.linear_vel.set(linear_vel + 'm/s')

        angular_vel = self.cmd_vel_panel.angular_vel.get().replace('rad/s', '')
        self.cmd_vel['angular_vel'] = self.service.validate_vel(angular_vel)
        self.cmd_vel_panel.angular_vel.set(angular_vel + 'rad/s')

    def move_robot(self, movement):
        if movement == 'LEFT':
            self.ros.pub_vel(0.0,  self.cmd_vel['angular_vel'])
        elif movement == 'RIGHT':
            self.ros.pub_vel(0.0, -self.cmd_vel['angular_vel'])
        elif movement == 'STOP':
            self.ros.pub_vel(0.0, 0.0)
        elif movement == 'FORWARD':
            self.ros.pub_vel(self.cmd_vel['linear_vel'], 0.0)
        elif movement == 'BACKWARD':
            self.ros.pub_vel(-self.cmd_vel['linear_vel'], 0.0)
        else: 
            print(f'MOVEMENT {movement} DOES NOT RECOGNIZED')

    def format_cmd_pose(self, *args):
        degrees = self.cmd_pose_panel.angle_var.get().replace('°', '')
        self.cmd_pose_panel.angle_var.set(degrees + '°')
        self.cmd_pose['angle'] = self.service.degrees_to_radians(degrees)
        self.cmd_pose_panel.radian_var.set(f'{self.cmd_pose['angle']:.4f} rad')

        distance_cm = self.cmd_pose_panel.distance_var.get().replace('cm', '') + 'cm'
        self.cmd_pose['distance'] = self.service.cm_to_m(distance_cm.replace('cm', ''))
        self.cmd_pose_panel.distance_var.set(distance_cm)

    def format_parameters(self, *args):
        linear_vel  = self.params_pop_up.linear_vel.get().replace('m/s', '')
        angular_vel = self.params_pop_up.angular_vel.get().replace('rad/s', '')
        max_advance = self.params_pop_up.max_advance.get().replace('m', '')
        max_turn_angle = self.params_pop_up.max_turn_angle.get().replace('rad', '')
        light_threshold = self.params_pop_up.light_threshold.get()
        laser_threshold = self.params_pop_up.laser_threshold.get()
        max_steps       = self.params_pop_up.max_steps.get()

        self.params_pop_up.linear_vel.set(linear_vel + 'm/s')
        self.params_pop_up.angular_vel.set(angular_vel + 'rads/s')
        self.params_pop_up.max_advance.set(max_advance + 'm')
        self.params_pop_up.max_turn_angle.set(max_turn_angle + 'rad')

    def on_click_run_stop(self):
        if self.is_executing:
            self.ros.cancel_goal()
        else:
            self.ros.send_goal(self.cmd_pose)

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

    def set_initial_params(self):
        self.params = self.service.format_params(self.ros.get_mp_params())

        if self.params is not None:
            self.behaviors_panel.behavior_list['values'] = self.params['behavior_list']
            self.behaviors_panel.behavior.set(self.params['behavior'])
            self.behaviors_panel.current_step.set(f'Step: 0')

    def on_click_behavior(self):
        if self.ros.get_behavior_running() is False:
            self.params['run_behavior'] = True
            self.params['behavior'] = self.behaviors_panel.behavior_list.get()
            self.ros.get_logger().warn(f'params->{self.params}')
        else:
            self.params['run_behavior'] = False
        self.ros.send_request(self.params)

    def on_click_set_params(self):
        self.params['max_steps'] = self.params_pop_up.max_steps.get()
        self.params['max_advance'] = float(self.params_pop_up.max_advance.get().replace('m', ''))
        self.params['max_turn_angle'] = float(self.params_pop_up.max_turn_angle.get().replace('rad', ''))
        self.params['light_threshold'] = self.params_pop_up.light_threshold.get()
        self.params['laser_threshold'] = self.params_pop_up.laser_threshold.get()
        self.params_pop_up.window.destroy()

    def loop(self):
        robot_name = self.ros.get_robot_name()
        self.status_panel.robot_name_var.set('No robot ' if robot_name is None else robot_name)

        battery_charge = self.ros.get_battery_charge()
        self.status_panel.progress_var.set(100 if battery_charge is None else battery_charge)
        self.status_panel.battery_percentage_var.set(f'Battery: {99 if battery_charge is None else battery_charge}%')
        
        self.is_executing = self.ros.movement_is_executing()

        if self.is_executing is False:
            self.cmd_pose_panel.run_stop.set('Run')
        else:
            self.cmd_pose_panel.run_stop.set('Stop')

        if self.ros.get_behavior_running() is False:
            self.behaviors_panel.run_stop.set('Run')
        else:
            self.behaviors_panel.run_stop.set('Stop')
            self.behaviors_panel.current_step.set(f'Steps: {self.ros.get_current_step()}')

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



        if self.params is None:
            self.set_initial_params()

        self.app.after(50, self.loop)