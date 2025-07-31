from PIL import Image, ImageDraw, ImageTk
from copy import copy

class AppContext:
    def __init__(self, app, color, content, service, ros, constants, file_manager):
        self.app      = app
        self.color    = color
        self.content  = content
        self.service  = service
        self.ros      = ros
        self.const    = constants
        self.file     = file_manager

        self.canvas   = None

        self.previus_canvas_size = None
        self.canvas_size  = { 'width':  500, 'height': 500 }
        self.canvas_scale = { 'width':  1,   'height': 1 }
        self.pixels_per_m = self.canvas_size['width']

        self.ros_params = None

        self.light      = None
        self.robot      = None

        self.side_frame      = None
        self.env_section     = None 
        self.sensors_section = None 
        self.robot_section   = None 
        self.buttons_section = None 

        self.objects = None
        self.route   = None

        self.angle_increment = 0.0
        self.displacement_increment = 0

        self.angle_tolerance = 0.01
        self.distance_tolerance = 1
        
        self.start_position = None
        self.current_step = 0

        self.map_polygons_points = None
        self.lasers_lines  = []
        self.lasers_values = []

        self._simulation_running = False
        self.run_last_simulation = False
        self.show_sensors = None
        self.velocity_slider = 1
        self.fast_mode = 0
        self.sensor_noise = False

        self.list_position = []
        self.list_angles   = []

    # SETTERS FOR SECTIONS
    def set_canvas(self, canvas):
        self.canvas = canvas

    def set_side_frame(self, side_frame):
        self.side_frame = side_frame

    def set_env_section(self, env_section):
        self.env_section = env_section

    def set_sensors_section(self, sensors_section):
        self.sensors_section = sensors_section

    def set_robot_section(self, robot_section):
        self.robot_section   = robot_section

    def set_buttons_section(self, buttons_section):
        self.buttons_section = buttons_section

    def set_light(self, light):
        self.light = light

    def set_robot(self, robot):
        self.robot = robot

    def set_objects(self, objects):
        self.objects = objects

    def set_route(self, route):
        self.route = route

    def get_canvas_size(self):
        return self.canvas_size

    def get_canvas_scale(self):
        return self.canvas_scale
    
    def set_canvas_scale(self, new_scale):
        self.canvas_scale = new_scale

    def polar_to_cartesian(self, magnitude, angle):
        return self.service.polar_to_cartesian(magnitude, angle)

    def request_ros_params(self):
        self.ros_params = self.service.format_ros_params(self.ros.request_ros_params())

    def get_ros_param(self, param_name):
        return self.ros_params[param_name]

    def get_context_param(self, name):
        if name == 'behavior':
            return self.env_section.behavior.get()
        elif name == 'max_steps':
            return int(self.env_section.max_steps.get())
        elif name == 'map':
            return self.env_section.environment.get()
        elif name == 'robot_angle':
            return float(self.robot_section.robot_angle.get())
        elif name == 'radius':
            return self.service.m_to_pixels(self.robot_section.robot_radius.get(), self.pixels_per_m)
        elif name == 'max_advance':
            return float(self.robot_section.max_advance.get())
        elif name == 'max_turn_angle':
            return float(self.robot_section.max_turn_angle.get())
        elif name == 'num_sensors':
            return int(self.sensors_section.num_sensors.get())
        elif name == 'origin_angle':
            return float(self.sensors_section.origin_angle.get())
        elif name == 'range_sensor':
            return float(self.sensors_section.range.get())
        elif name == 'light_threshold':
            return float(self.sensors_section.light_value.get())
        elif name == 'laser_threshold':
            return self.service.m_to_pixels(self.sensors_section.laser_value.get(), self.pixels_per_m)
        else:
            print(f'AppContext.get_context_param()->PARAMETER {name} NOT RECOGNIZED BY CONTEXT')
    
    def set_context_param(self, name, value):
        if name == 'robot_angle':
            angle = self.service.normalize_angle(value)
            self.robot_section.robot_angle.set(angle)
        elif name == 'button_plot_topological':
            self.buttons_section.plot_topological .config(state = value)
        elif name == 'button_run' :
            self.buttons_section.button_run       .config(state = value)
        elif name == 'button_stop':
            self.buttons_section.button_stop      .config(state = value)
        elif name == 'panel_status':
            self.buttons_section.button_run       .config(state = value)
            self.env_section.environment          .config(state = value)
            self.env_section.behavior             .config(state = value)
            self.env_section.steps                .config(state = value)
            self.robot_section.entry_radius       .config(state = value)
            self.robot_section.entry_advance      .config(state = value)
            self.robot_section.entry_turn_angle   .config(state = value)
        elif name == 'light_pose_x':
            self.env_section.light_pose_x.set(value)
        elif name == 'light_pose_y':
            self.env_section.light_pose_y.set(value)
        elif name == 'current_step':
            self.env_section.curr_step.set(value)
        elif name == 'robot_pose_x':
            self.robot_section.robot_pose_x.set(value)
        elif name == 'robot_pose_y':
            self.robot_section.robot_pose_y.set(value)
        elif name == 'num_sensors':
            self.sensors_section.num_sensors.set(value)
        elif name == 'origin_angle':
            self.sensors_section.origin_angle.set(value)
        elif name == 'range_sensor':
            self.sensors_section.range.set(value)
        elif name == 'light_threshold':
            self.sensors_section.light_value.set(value)
        elif name == 'laser_threshold':
            self.sensors_section.laser_value.set(value)
        else:
            print(f'AppContext.set_context_param()->PARAMETER {name} NOT RECOGNIZED BY CONTEXT')

    def format_to_position(self, x, y):
        return self.service.format_to_position(x, y)

    def format_to_robot_state(self, position, angle, radius):
        return self.service.format_to_robot_state(position, angle, radius)

    def set_light_position(self, e_point):
        self.light.plot(self.service.format_to_position(e_point.x, e_point.y))
        xm, ym = self.service.px_point_to_m(e_point.x, e_point.y, self.canvas_size)
        self.set_context_param('light_pose_x', xm)
        self.set_context_param('light_pose_y', ym)
        if self.robot.exists():
            self.run_simulation()

    def set_robot_position(self, e_point):
        # self.route.delete()
        self.robot.plot(self.service.format_to_position(e_point.x, e_point.y))
        xm, ym = self.service.px_point_to_m(e_point.x, e_point.y, self.canvas_size)
        self.set_context_param('robot_pose_x', xm)
        self.set_context_param('robot_pose_y', ym)
        if self.light.exists():
            self.set_context_param('button_run', 'normal')

    def plot_lidar_sensors(self):
        self.canvas.delete('laser')
        for laser in self.lasers_lines:
            self.canvas.create_line(laser, fill = self.color['laser'], tag = 'laser')

    def get_environment_list(self):
        return self.file.get_environment_list()

    def get_object_list(self):
        return self.service.parse_objects_file(self.file.get_objects_file(), self.canvas_size)

    def resize_canvas(self, width, height):
        self.previus_canvas_size = copy(self.canvas_size)
        self.canvas_size['width']  = width
        self.canvas_size['height'] = height
        self.canvas.configure(width = width, height = height)
        self.pixels_per_m = width
        self.plot_map()
        self.light.remap_position() if self.light.exists() else None
        self.robot.remap_position() if self.robot.exists() else None

    def remap_position(self, position):
        if self.previus_canvas_size is not None:
            position = self.service.remap_position(position, self.canvas_size, self.previus_canvas_size)
        return position

    def get_file_path(self, file_name):
        return self.file.get_path(file_name)
    
    def plot_grid(self):
        self.canvas.delete('grid')
        for axis in self.canvas_size:
            line = self.service.get_grid_line(self.canvas_size[axis], self.canvas_scale[axis], self.const['line_per_meters'])
            for i in range(1, int(self.const['line_per_meters'] * self.canvas_scale[axis])):
                line_points = self.service.get_line_points(i, line, axis, self.canvas_size[axis])
                self.canvas.create_line(line_points, dash = (4, 4), fill = self.color['grid'], tag  = 'grid' )

    def plot_map(self, event = None):
        self.canvas.delete('map')
        self.canvas.delete('topological_map')
        map_file = self.file.get_map(self.get_context_param('map'))
        scale, polygon_list, polygon_to_plot_list = self.service.parse_map(map_file, self.canvas_size)
        self.set_canvas_scale(scale)
        self.plot_grid()
        self.map_polygons_points = self.service.transoform_to_points_list(polygon_list)

        for polygon_to_plot in polygon_to_plot_list:
            self.canvas.create_polygon(
                polygon_to_plot,
                outline = self.color['obstacle_outline'],
                fill = self.color['obstacle_inner'],
                width = 1, tag = 'map')

    def get_circles_coords(self, center, radius):
        body_coords   = self.service.get_circle_coords(center, radius)
        body = self.service.format_figure_to_plot(body_coords, self.color['robot'])
        hokuyo_coords = self.service.get_circle_coords(center, radius / 5)
        hokuyo = self.service.format_figure_to_plot(hokuyo_coords, self.color['hokuyo'])
        return [body, hokuyo]

    def get_polygons_coords(self, anchor, radius, angle):
        coords = self.service.get_polygon(
            self.const['head_rel_points'], anchor, angle, radius)

        head = self.service.format_figure_to_plot(coords, self.color['head'])
        coords = self.service.get_polygon(
            self.const['left_wheel_rel_points'], anchor, angle, radius)
        left_wheel = self.service.format_figure_to_plot(coords, self.color['wheel'])

        coords = self.service.get_polygon(
            self.const['right_wheel_rel_points'], anchor, angle, radius)
        right_wheel = self.service.format_figure_to_plot(coords, self.color['wheel'])

        return [head, left_wheel, right_wheel]
    
    def set_robot_angle(self, event = None):
        if event is None:
            self.set_context_param('robot_angle', 0.0)
            self.robot.set_angle(0) if self.robot.exists() else None
        else:
            if self.robot.exists():
                self.robot.set_angle(self.get_context_param('robot_angle')) 

        self.robot.plot() if self.robot.exists() else None

    def set_robot_radius(self, event = None):
        self.robot.plot() if self.robot.exists() else None

    def on_check_fast_mode(self, value):
        self.fast_mode = value

    def on_check_show_sensors(self, value):
        self.show_sensors = value
        if self.robot.exists():
            self.robot.plot()

    def on_check_load_objects(self, load_objects):
            self.objects.plot() if load_objects == 1 else self.objects.delete()

    def on_check_noise(self, noise):
        self.sensor_noise = True if noise == 1 else False

    def set_velocity_slider(self, value):
        self.velocity_slider = value

    def on_change_robot_sensors(self, event = None):
        if self.show_sensors and self.robot.exists():
            self.robot.plot()

    # SHARED METHODS
    def run_simulation(self):
        if self.robot.exists():
            self.ros_params['run_behavior'] = True
            self.ros_params['step'] = 0
            self.ros_params['max_steps'] = self.get_context_param('max_steps')
            self.ros_params['behavior']  = self.get_context_param('behavior')
            self.ros_params['max_advance'] = self.get_context_param('max_advance')
            self.ros_params['max_turn_angle'] = self.get_context_param('max_turn_angle')

            self.ros.send_state_params(self.ros_params)
            self.route.delete()

    def stop_simulation(self):
        self.ros_params['run_behavior'] = False
        self.ros.send_state_params(self.ros_params)
        self.ros.finish_movement()

    def last_simulation(self):
        self.run_last_simulation = True

    def plot_topological_map(self):
        # self.buttons_section.plot_topological.config(state = "disabled")
        map = self.get_context_param('map')
        topological_file = self.file.get_map(map, topological = True)
        node_coords, node_coords_to_plot, connections = self.service.parse_topological_map(topological_file, self.canvas_size, self.canvas_scale)
        # # print(node_coords)
        # print(node_coords_to_plot)
        # if node_coords is None:
        #     return
        # # print(connections)
        image = Image.new('RGBA', (500, 500))
        draw = ImageDraw.Draw(image)
        for i in range(len(node_coords_to_plot)):
            # print(f'node_coords_to_plot[{i}]->{node_coords_to_plot[i]}')
            draw.ellipse((node_coords_to_plot[i]['x'] - 3, node_coords_to_plot[i]['y'] - 3, node_coords_to_plot[i]['x'] + 3, node_coords_to_plot[i]['y'] + 3), outline = '#9C4FDB', fill = '#9C4FDB')
            draw.text((node_coords_to_plot[i]['x'], node_coords_to_plot[i]['y'] + 2), fill = "darkblue" ,text = str(i))
        
        for i in range(len(connections)):
            a = connections[i]
            draw.line((node_coords_to_plot[a[0]]['x'], node_coords_to_plot[a[0]]['y'], node_coords_to_plot[a[1]]['x'], node_coords_to_plot[a[1]]['y']) , fill = '#9C4FDB')

        image.save('nodes.png')
        self.gif1 = ImageTk.PhotoImage( file ='nodes.png')
        self.canvas.create_image(250, 250, image = self.gif1, tag = 'topological_map')

    def generate_laser_readings(self):
        robot_state   = self.robot.get_state()
        sensor_params = self.service.format_to_sensors_params(
            self.get_context_param('robot_angle'),
            self.get_context_param('num_sensors'),
            self.get_context_param('origin_angle'),
            self.get_context_param('range_sensor'),
            self.get_context_param('laser_threshold'))
        
        noise_param = self.const['sigma'] if self.sensor_noise else None
        self.lasers_lines, self.lasers_values = self.service.generate_lasers_readings(
            robot_state, sensor_params, self.map_polygons_points, noise_param)

    def check_for_collision(self):
        ids_robot   = self.canvas.find_withtag('robot')
        ids_map = self.canvas.find_withtag('map')

        for id_robot in ids_robot:
            bbox_robot = self.canvas.bbox(id_robot)
            if bbox_robot is None:
                continue
            overlapping = self.canvas.find_overlapping(*bbox_robot)
            for id_map in ids_map:
                if id_robot != id_map and id_map in overlapping:
                    return True
        return False

    def animation_loop(self):
        if self.robot.exists():

            
            self.ros.set_lidar_data(self.lasers_values)
            
            goal_pose = self.service.format_goal_pose(self.ros.get_goal_pose(), self.canvas_size)
            if self.check_for_collision() and goal_pose:
                self.ros.get_logger().info(f'ROBOT COLLISION')
                self.stop_simulation()
                self.start_position = None

            if self.light.exists():
                light_data = self.service.simulate_light_data(self.robot.get_position(),
                                                              self.light.get_position(),
                                                              self.robot.get_angle(),
                                                              self.get_context_param('radius'))
                self.ros.set_light_data(light_data)

                if goal_pose and self.check_for_collision() is False:
                    self._simulation_running = True
                    current_position = self.robot.get_position()

                    if self.start_position is None:
                        self.start_position = current_position

                    if self.route.is_empty():
                        self.route.initialize_route(current_position, self.robot.get_angle())
                        self.current_step = 0


                    if self.fast_mode:
                        self.robot.rotate(goal_pose['angle'])
                        self.robot.displace(goal_pose['distance'])
                        self.ros.finish_movement()
                        self.route.trace(self.start_position,
                                            self.robot.get_position(),
                                            self.robot.get_angle())
                        self.start_position = None
                        if self.current_step < self.get_context_param('max_steps'):
                            self.current_step += 1 
                    else:
                        delta = goal_pose['angle'] - self.angle_increment
                        if abs(delta) >= self.angle_tolerance:
                            direction =  1 if goal_pose['angle'] > 0 else - 1 # 1: LEFT, -1: RIGHT
                            direction *= self.const['angle_increment']
                            increment = self.service.degrees_to_radians(direction)
                            self.angle_increment += increment
                            self.robot.rotate(increment)
                        else:
                            delta = goal_pose['distance'] - self.displacement_increment
                            if abs(delta) >= self.distance_tolerance:
                                increment = 1 if goal_pose['distance'] > 0 else - 1
                                increment *= self.const['distance_increment']
                                self.displacement_increment += increment
                                self.robot.displace(increment)
                            else:
                                self.angle_increment = 0.0
                                self.displacement_increment = 0.0
                                self.ros.finish_movement()
                                self.route.trace(self.start_position,
                                                 self.robot.get_position(),
                                                 self.robot.get_angle())
                                self.start_position = None
                                if self.current_step < self.get_context_param('max_steps'):
                                    self.current_step += 1 

                        self.service.sleep(self.velocity_slider)
                    self.set_context_param('current_step', self.current_step)
                elif self.ros.behavior_running() is False and self._simulation_running:
                    # SIMULATION FINISHED
                    self._simulation_running = False
                    self.list_position, self.list_angles= self.route.get_all_route()
                    self.set_context_param('panel_status', 'normal')
                    self.set_context_param('button_stop',  'disabled')

                elif self.run_last_simulation:
                    for i in range(len(self.list_position)):
                        self.robot.set_angle(self.list_angles[i])
                        self.robot.plot(self.list_position[i])
                        self.service.sleep(-10)
                        self.canvas.update()
                    self.run_last_simulation = False

        self.app.after(1, self.animation_loop)