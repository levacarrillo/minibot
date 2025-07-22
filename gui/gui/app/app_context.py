from PIL import Image, ImageDraw, ImageTk
from copy import copy


class AppContext:
    def __init__(self, app, color, content, service, ros, file_manager):
        self.app      = app
        self.color    = color
        self.content  = content
        self.service  = service
        self.ros      = ros
        self.file     = file_manager

        self.canvas   = None

        self.previus_canvas_size = None
        self.canvas_size  = { 'width':  500, 'height': 500 }
        self.canvas_scale = { 'width':  1,   'height': 1 }
        self.pixels_per_m = self.canvas_size['width']

        self.ros_params = None

        self.light        = None
        self.robot        = None

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

        self.simulation_running  = False

        self.run_last_simulation = False
        self.show_sensors = None
        self.velocity_slider = 1
        self.fast_mode = 0
        self.sensor_noise = False

        self.nodes_image = None

        self.polygon_list = None


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
        elif name == 'angle':
            return float(self.robot_section.robot_angle.get())
        elif name == 'radius':
            return self.service.m_to_pixels(self.robot_section.robot_radius.get(), self.pixels_per_m)
        elif name == 'max_advance':
            return float(self.robot_section.max_advance.get())
        elif name == 'max_turn_angle':
            return float(self.robot_section.max_turn_angle.get())
        elif name == 'num_sensors':
            return int(self.sensors_section.entry_num_sensors.get())
        elif name == 'origin_angle':
            return float(self.sensors_section.entry_origin_angle.get())
        elif name == 'range_sensor':
            return float(self.sensors_section.entry_range.get())
        elif name == 'light_threshold':
            return float(self.sensors_section.entry_light.get())
        elif name == 'laser_threshold':
            return float(self.sensors_section.entry_laser.get())
        else:
            print(f'AppContext.get_context_param()->PARAMETER {name} NOT RECOGNIZED BY CONTEXT')
    
    def set_context_param(self, name, value):
        if name == 'angle':
            angle = self.service.normalize_angle(value)
            self.robot_section.robot_angle.set(angle)
        elif name == 'button_stop':
            self.buttons_section.button_stop    .config(state = value)
        elif name == 'panel_status':
            self.buttons_section.button_run     .config(state = value)
            self.env_section.environment        .config(state = value)
            self.env_section.behavior           .config(state = value)
            self.env_section.steps              .config(state = value)
            self.robot_section.entry_radius     .config(state = value)
            self.robot_section.entry_advance    .config(state = value)
            self.robot_section.entry_turn_angle .config(state = value)
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
        else:
            print(f'AppContext.set_context_param()->PARAMETER {name} NOT RECOGNIZED BY CONTEXT')

    def set_light_position(self, x, y):
        xm, ym = self.service.px_point_to_m(x, y, self.canvas_size)
        self.set_context_param('light_pose_x', xm)
        self.set_context_param('light_pose_y', ym)
        return { 'x': x, 'y': y }

    def set_robot_position(self, x, y):
        xm, ym = self.service.px_point_to_m(x, y, self.canvas_size)
        self.set_context_param('robot_pose_x', xm)
        self.set_context_param('robot_pose_y', ym)
        return { 'x': x, 'y': y }

    def get_environment_list(self):
        return self.file.get_environment_list()

    def get_object_list(self):
        return self.service.parse_objects_file(self.file.get_objects_file(),
                                               self.canvas_size)

    def resize_canvas(self, width, height):
        self.previus_canvas_size = copy(self.canvas_size)
        self.canvas_size['width']  = width
        self.canvas_size['height'] = height
        self.canvas.configure(width = width, height = height)
        self.pixels_per_m = width
        self.plot_map()
        self.light.plot() if self.light.exists() else None
        self.robot.plot() if self.robot.exists() else None

    def remap_position(self, position):
        if self.previus_canvas_size is not None:
            position['x'] = self.canvas_size['width'] * position['x'] / self.previus_canvas_size['width']
            position['y'] = self.canvas_size['height'] * position['y'] / self.previus_canvas_size['height']
        return position

    def get_file_path(self, file_name):
        return self.file.get_path(file_name)

    def plot_map(self, event = None):
        self.canvas.delete('grid')
        map_file = self.file.get_map(self.get_context_param('map'))
        scale, polygon_to_plot_list = self.service.parse_map(map_file, self.canvas_size)
        self.set_canvas_scale(scale)
        line_per_meters = 10
        for axis in self.canvas_size:
            line = self.canvas_size[axis] / (line_per_meters * self.canvas_scale[axis])
            for i in range(1, int(line_per_meters * self.canvas_scale[axis])):
                points = [i * line if axis == 'width' else 0,
                          i * line if axis != 'width' else 0,
                          i * line if axis == 'width' else self.canvas_size[axis],
                          i * line if axis != 'width' else self.canvas_size[axis]]

                self.canvas.create_line(points, dash = (4, 4), fill = self.color['grid'], tag  = 'grid' )
        
        self.canvas.delete('map')
        # print(f'polygon_to_plot_list->{polygon_to_plot_list}')
        for polygon_to_plot in polygon_to_plot_list:
            self.canvas.create_polygon(polygon_to_plot, outline = self.color['grid'], fill = self.color['grid'],
                                                                                        width = 1, tag = 'map')

    def get_circles_coords(self, position, radius):
        body = {
            'color' : self.color['robot'],
            'coords': [
                position['x'] - radius,
                position['y'] - radius,
                position['x'] + radius,
                position['y'] + radius
            ]
        }
        hokuyo = {
            'color': self.color['hokuyo'],
            'coords': [
                position['x'] - (radius / 5),
                position['y'] - (radius / 5),
                position['x'] + (radius / 5),
                position['y'] + (radius / 5)
            ] 
        }
        return [body, hokuyo]

    def get_polygon_coords(self, position, radius, angle):
        # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
        head_rel_points = [{ 'x': 2/3, 'y': - 1/3 },
                  { 'x': 2/3, 'y':   1/3 },
                  { 'x': 5/6, 'y':  0.0  }]

        left_wheel_rel_points  = [{'x': -1/2, 'y': -5/6 },
                        {'x':  1/2, 'y': -5/6 },
                        {'x':  1/2, 'y': -3/6 },
                        {'x': -1/2, 'y': -3/6 }]

        right_wheel_rel_points = [{'x': -1/2, 'y':  3/6 },
                         {'x':  1/2, 'y':  3/6 },
                         {'x':  1/2, 'y':  5/6 },
                         {'x': -1/2, 'y':  5/6 }]

        head_polygon = []
        for relative_point in head_rel_points:
            head_polygon.append(self.service.transform_to_polygon_point(position, angle, radius, relative_point))
        left_wheel_polygon = []
        for relative_point in left_wheel_rel_points:
            left_wheel_polygon.append(self.service.transform_to_polygon_point(position, angle, radius, relative_point))
        right_wheel_polygon = []
        for relative_point in right_wheel_rel_points:
            right_wheel_polygon.append(self.service.transform_to_polygon_point(position, angle, radius, relative_point))

        head = {
            'color':  self.color['head'],
            'coords': head_polygon
        }

        left_wheel = {
            'color':  self.color['wheel'],
            'coords': left_wheel_polygon
        }

        right_wheel = {
            'color':  self.color['wheel'],
            'coords': right_wheel_polygon
        }

        return [head, left_wheel, right_wheel]
    
    def set_robot_angle(self, event = None):
        if event is None:
            self.set_context_param('angle', 0.0)
            self.robot.set_angle(0) if self.robot.exists() else None
        else:
            if self.robot.exists():
                self.robot.set_angle(self.get_context_param('angle')) 

        self.robot.plot() if self.robot.exists() else None

    def set_robot_radius(self, event = None):
        self.robot.plot() if self.robot.exists() else None

    def on_check_fast_mode(self, value):
        self.fast_mode = value

    def on_check_show_sensors(self, value):
        self.show_sensors = value

    def on_check_load_objects(self, load_objects):
            self.objects.plot() if load_objects == 1 else self.objects.delete()

    def on_check_noise(self, noise):
        self.sensor_noise = True if noise == 1 else False

    def set_velocity_slider(self, value):
        self.velocity_slider = value

    def on_change_robot_sensors(self, event = None):
        print('t0d0')

    # SHARED METHODS
    def run_simulation(self):
        self.ros_params['run_behavior'] = True
        self.ros_params['step'] = 0
        self.ros_params['max_steps'] = self.get_context_param('max_steps')
        self.ros_params['behavior']  = self.get_context_param('behavior')
        self.ros_params['max_advance'] = self.get_context_param('max_advance')
        self.ros_params['max_turn_angle'] = self.get_context_param('max_turn_angle')
        self.set_context_param('panel_status', 'disabled')
        self.set_context_param('button_stop',  'normal')
        self.ros.send_state_params(self.ros_params)
        self.route.delete()

    def stop_simulation(self):
        self.simulation_running = False
        self.set_context_param('panel_status', 'normal')
        self.set_context_param('button_stop',  'disabled')
        # controller.finish_movement()
        # context.simulation_running = False
        # context.enable_button_run()

    def last_simulation(self):
        self.run_last_simulation = True

        # if self.controller.check_for_topological_map(self.get_context_param('map')):
        #     self.buttons_section.plot_topological.config(state = NORMAL)
        # else:
        #     self.buttons_section.plot_topological.config(state = DISABLED)

    def plot_topological_map(self):
        self.buttons_section.plot_topological.config(state = DISABLED)
        node_coords, node_coords_to_plot, connections = self.controller.get_topological_map(self.get_context_param('map'), topological = True)
        # print(node_coords)
        # print(node_coords_to_plot)
        if node_coords is None:
            return
        # print(connections)
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
        self.nodes_image = self.canvas.create_image(250, 250, image = self.gif1)

    def clear_topological_map(self):
        if self.nodes_image is not None:
            self.canvas.delete(self.nodes_image)

    def get_polygon_list(self):
        print(f'POLYGONS NUM->{len(self.polygon_list)}')
        polygon_list = []
        for polygon_vertices in self.polygon_list:
            polygon_points = []
            for i in range(0, len(polygon_vertices), 2):
                point = self.controller.set_position(polygon_vertices[i], polygon_vertices[i+1])
                polygon_points.append(point)
            polygon_list.append(polygon_points)

        return polygon_list

    def animation_loop(self):
        if self.robot.exists():
            goal_pose = self.service.format_goal_pose(self.ros.get_goal_pose(), self.canvas_size)
            if self.light.exists():

                light_readings = self.service.get_light_readings(self.robot.get_position(),
                                                                 self.light.get_position(),
                                                                 self.robot.get_angle(),
                                                                 self.get_context_param('radius'))
                self.ros.set_light_readings(light_readings)

                if self.ros.get_ros_params().run_behavior and goal_pose:
                    self.simulation_running = True
                    current_position = self.robot.get_position()

                    if self.start_position is None:
                        self.start_position = current_position

                    if self.route.is_empty():
                        self.route.initialize_route(current_position, self.robot.get_angle())
                        self.current_step = 0


                    if self.fast_mode:
                        # print(f'angle->{goal_pose['angle']}, distance->{goal_pose['distance']}')
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
                            direction = 1 if goal_pose['angle'] > 0 else -1 # 1: LEFT, -1: RIGHT
                            increment = self.service.degrees_to_radians(direction)
                            self.angle_increment += increment
                            self.robot.rotate(increment)
                        else:
                            delta = goal_pose['distance'] - self.displacement_increment
                            if abs(delta) >= self.distance_tolerance:
                                increment = 1 if goal_pose['distance'] > 0 else -1
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
                elif self.simulation_running and self.ros.get_ros_params().run_behavior is False:
                    self.simulation_running = False
                    self.set_context_param('panel_status', 'normal')
                    self.set_context_param('button_stop',  'disabled')

        self.app.after(1, self.animation_loop)