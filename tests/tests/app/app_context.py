class AppContext:
    def __init__(self, app, content, service, ros):
        self.app     = app
        self.content = content
        self._service = service
        self._ros     = ros

        # FRAMES
        self.status_panel    = None
        self._draw_panel      = None
        self._cmd_vel_panel   = None
        self._cmd_pose_panel  = None
        self._behaviors_panel = None
        self._params_pop_up   = None


        # VELOCITY COMMANDS
        self._cmd_vel = {
            'linear_vel': None,
            'angular_vel': None
        }

        # POSITION COMMANDS
        self._cmd_pose = {
            'angle': None,
            'distance': None
        }

        # PARAMS REQUIRED FOR MOTION PLANNER NODE
        self._params = None

        self._movement_is_executing = False


    # ADD PANELS TO CONTEXT
    def set_status_panel(self, status_panel):
        self.status_panel = status_panel

    def set_draw_panel(self, draw_panel):
        self._draw_panel = draw_panel

    def set_cmd_vel_panel(self, cmd_vel_panel):
        self._cmd_vel_panel = cmd_vel_panel

    def set_cmd_pose_panel(self, cmd_pose_panel):
        self._cmd_pose_panel = cmd_pose_panel

    def set_behaviors_panel(self, behaviors_panel):
        self._behaviors_panel = behaviors_panel

    def set_params_pop_up(self, params_pop_up):
        self._params_pop_up = params_pop_up

    # FORMATTING PARAMETERS
    def format_cmd_vel(self, *args):
        linear_vel = self._cmd_vel_panel.linear_vel.get().replace('m/s', '')
        self._cmd_vel['linear_vel'] = self._service.validate_vel(linear_vel)
        self._cmd_vel_panel.linear_vel.set(linear_vel + 'm/s')

        angular_vel = self._cmd_vel_panel.angular_vel.get().replace('rad/s', '')
        self._cmd_vel['angular_vel'] = self._service.validate_vel(angular_vel)
        self._cmd_vel_panel.angular_vel.set(angular_vel + 'rad/s')

    def move_robot(self, movement):
        if movement == 'LEFT':
            self._ros.publish_velocity(0.0,  self._cmd_vel['angular_vel'])
        elif movement == 'RIGHT':
            self._ros.publish_velocity(0.0, -self._cmd_vel['angular_vel'])
        elif movement == 'STOP':
            self._ros.publish_velocity(0.0, 0.0)
        elif movement == 'FORWARD':
            self._ros.publish_velocity(self._cmd_vel['linear_vel'], 0.0)
        elif movement == 'BACKWARD':
            self._ros.publish_velocity(-self._cmd_vel['linear_vel'], 0.0)
        else: 
            print(f'MOVEMENT {movement} DOES NOT RECOGNIZED')

    def format_cmd_pose(self, *args):
        degrees = self._cmd_pose_panel.angle_var.get().replace('°', '')
        self._cmd_pose_panel.angle_var.set(degrees + '°')
        self._cmd_pose['angle'] = self._service.degrees_to_radians(degrees)
        self._cmd_pose_panel.radian_var.set(f'{self._cmd_pose['angle']:.4f} rad')

        distance_cm = self._cmd_pose_panel.distance_var.get().replace('cm', '') + 'cm'
        self._cmd_pose['distance'] = self._service.cm_to_m(distance_cm.replace('cm', ''))
        self._cmd_pose_panel.distance_var.set(distance_cm)

    def set_initial_window_parameters(self):
        self._params_pop_up.linear_vel.set(self._params['linear_velocity'])
        self._params_pop_up.angular_vel.set(self._params['angular_velocity'])
        self._params_pop_up.max_advance.set(self._params['max_advance'])
        self._params_pop_up.max_turn_angle.set(self._params['max_turn_angle'])
        self._params_pop_up.light_threshold.set(self._params['light_threshold'])
        self._params_pop_up.laser_threshold.set(self._params['laser_threshold'])
        self._params_pop_up.max_steps.set(self._params['max_steps'])

    def format_parameters(self, *args):
        linear_vel  = self._params_pop_up.linear_vel.get().replace('m/s', '')
        angular_vel = self._params_pop_up.angular_vel.get().replace('rad/s', '')
        max_advance = self._params_pop_up.max_advance.get().replace('m', '')[:6]
        max_turn_angle = self._params_pop_up.max_turn_angle.get().replace('rad', '')
        light_threshold = self._params_pop_up.light_threshold.get()
        laser_threshold = self._params_pop_up.laser_threshold.get()
        max_steps       = self._params_pop_up.max_steps.get()

        self._params_pop_up.linear_vel.set(linear_vel + 'm/s')
        self._params_pop_up.angular_vel.set(angular_vel + 'rad/s')
        self._params_pop_up.max_advance.set(max_advance + 'm')
        self._params_pop_up.max_turn_angle.set(max_turn_angle + 'rad')

    def _set_initial_params(self):
        self._params = self._service.format_params(self._ros.get_motion_planner_params(), self._ros.get_vel_params())
        if self._params is not None:
            self._behaviors_panel.behavior_list['values'] = self._params['behavior_list']
            self._behaviors_panel.behavior.set(self._params['behavior'])
            self._behaviors_panel.current_step.set(f'Step: 0')

    # METHODS FOR CANVA'S PLOTTING
    def get_head_coords(self, width):
        return self._service.get_head_coords(width)

    def get_body_coords(self, width):
        return self._service.get_body_coords(width)

    def get_hokuyo_coords(self, width):
        return self._service.get_hokuyo_coords(width)

    # SENSORS DATA
    def get_lights_max_intensity(self):
        return self._service.validate_lights_response(self._ros.get_light_readings())

    def get_lidar_readings(self):
        return self._service.norm_lidar_response(self._ros.get_lidar_readings())

    # EVENTS METHODS
    def on_click_run_stop(self):
        if self._movement_is_executing:
            self._ros.cancel_goal()
        else:
            self._ros.send_goal_pose(self._cmd_pose)

    def on_click_behavior(self):
        if self._ros.get_behavior_running() is False:
            self._params['run_behavior'] = True
            self._params['behavior'] = self._behaviors_panel.behavior_list.get()
        else:
            self._params['run_behavior'] = False
        self._ros.set_vel_params(self._params)
        self._ros.send_motion_planner_req(self._params)

    def on_click_set_params(self):
        self._params['max_steps'] = self._params_pop_up.max_steps.get()
        self._params['linear_velocity']  = float(self._params_pop_up.linear_vel  .get().replace('m/s', ''))
        self._params['angular_velocity'] = float(self._params_pop_up.angular_vel .get().replace('rad/s', ''))
        self._params['max_advance'] = float(self._params_pop_up.max_advance.get().replace('m', ''))
        self._params['max_turn_angle'] = float(self._params_pop_up.max_turn_angle.get().replace('rad', ''))
        self._params['light_threshold'] = float(self._params_pop_up.light_threshold.get())
        self._params['laser_threshold'] = float(self._params_pop_up.laser_threshold.get())
        self._params_pop_up.window.destroy()

    # GENERAL METHOD
    def loop(self):
        robot_name = self._ros.get_robot_name()
        battery_charge  = self._ros.get_battery_charge()
        mc_temperature  = self._ros.get_mc_temperature()
        cpu_temperature = self._service.truncate(self._ros.get_cpu_temperature())

        self.status_panel.robot_name_var.set('No robot ' if robot_name is None else robot_name)
        self.status_panel.battery_percent_1_var.set(f'{99 if battery_charge is None else battery_charge[0]}%')
        self.status_panel.battery_percent_2_var.set(f'{99 if battery_charge is None else battery_charge[1]}%')
        self.status_panel.progress_bar1_var.set(100 if battery_charge is None else battery_charge[0])
        self.status_panel.progress_bar2_var.set(100 if battery_charge is None else battery_charge[1])
        
        self.status_panel.micro_c_temperature_var.set(f'MC:  {mc_temperature}°C')
        self.status_panel.cpu_temperature_var.set(f'CPU: {cpu_temperature}°C')
        self._movement_is_executing = self._ros.movement_is_running()

        self._cmd_pose_panel.run_stop.set('Stop' if self._movement_is_executing else 'Run')

        self._behaviors_panel.run_stop.set('Stop' if self._ros.get_behavior_running() else 'Run')
        self._behaviors_panel.current_step.set(f'Steps: {self._ros.get_current_step()}')

        id_max = self.get_lights_max_intensity()
        lidar_params = self.get_lidar_readings()
        width = self._draw_panel.width
        self._draw_panel.clear()

        for i in range(8):
            coords = self._service.get_spotlight_coords(i, width)
            light = 'yellow' if id_max == i else None   
            self._draw_panel.plot_spotlight(coords, color = light)

        for i in range(lidar_params['num_readings']):
            coords = self._service.get_laser_coords(i, lidar_params, width)
            self._draw_panel.plot_laser(coords)
        
        if self._params is None:
            self._set_initial_params()

        self.app.after(50, self.loop)