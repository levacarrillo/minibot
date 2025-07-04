from tkinter import NORMAL, DISABLED, END


class AppContext:
    def __init__(self, app, color, content, controller):
        self.app          = app
        self.color        = color
        self.content      = content
        self.controller   = controller

        self.canvas       = None
        self.canvas_size  = None
        self.canvas_panel = None

        self.side_frame   = None

        self.grid         = None
        self.light        = None
        self.robot        = None

        self.env_section     = None 
        self.sensors_section = None 
        self.robot_section   = None 
        self.buttons_section = None 

        self.simulation_running  = False
        self.show_sensors = None
        self.velocity_slider = 1
        self.fast_mode = 0
        self.route = None

        self.polygon_list = None


    # SETTERS FOR SECTIONS
    def set_env_section(self, env_section):
        self.env_section = env_section

    def set_sensors_section(self, sensors_section):
        self.sensors_section = sensors_section

    def set_robot_section(self, robot_section):
        self.robot_section   = robot_section

    def set_buttons_section(self, buttons_section):
        self.buttons_section = buttons_section

    # SETTERS FOR CANVA'S COMPONENTS
    def set_canvas_panel(self, canvas_panel):
        self.canvas_panel = canvas_panel
        self.canvas = canvas_panel.canvas
        self.canvas_size  = canvas_panel.size

    def set_canvas_size(self, new_size_x, new_size_y):
        self.canvas_size  = self.controller.set_canvas_size(new_size_x, new_size_y)

    def set_grid(self, grid):
        self.grid = grid

    def set_light(self, light):
        self.light = light

    def set_robot(self, robot):
        self.robot = robot

    def set_velocity_slider(self, value):
        self.velocity_slider = value
    
    def set_fast_mode(self, value):
        self.fast_mode = value

    def set_show_sensors(self, value):
        self.show_sensors = value

    def set_route(self, value):
        self.route = value

    def get_param(self, name):
        if name == 'behavior':
            return self.env_section.behavior_list_cb.get()
        elif name == 'max_steps':
            return int(self.env_section.steps_entry.get())
        elif name == 'map':
            return self.env_section.environment_cb.get()
        elif name == 'entry_angle':
            return self.robot_section.entry_angle.get()
        elif name == 'entry_radius':
            return self.robot_section.entry_radius.get()
        elif name == 'max_advance':
            return float(self.robot_section.entry_advance.get())
        elif name == 'max_turn_angle':
            return float(self.robot_section.entry_turn_angle.get())
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
            print(f'GET_PARAM()->PARAMETER {name} NOT RECOGNIZED BY CONTEXT')

    def enable_button_run(self):
        self.buttons_section.button_stop   .config(state = DISABLED)
        self.buttons_section.button_run    .config(state = NORMAL)
        self.env_section.environment_cb    .config(state = NORMAL)
        self.env_section.behavior_list_cb  .config(state = NORMAL)
        self.env_section.steps_entry       .config(state = NORMAL)
        self.robot_section.entry_radius    .config(state = NORMAL)
        self.robot_section.entry_advance   .config(state = NORMAL)
        self.robot_section.entry_turn_angle.config(state = NORMAL)

    def disable_button_run(self):
        self.buttons_section.button_stop  .config(state = NORMAL)
        self.buttons_section.button_run   .config(state = DISABLED)
        self.env_section.environment_cb   .config(state = DISABLED)
        self.env_section.behavior_list_cb .config(state = DISABLED)
        self.env_section.steps_entry      .config(state = DISABLED)
        self.robot_section.entry_radius    .config(state = DISABLED)
        self.robot_section.entry_advance   .config(state = DISABLED)
        self.robot_section.entry_turn_angle.config(state = DISABLED)

    # SETTERS FOR MAIN FRAMES
    def set_side_frame(self, side_frame):
        self.side_frame = side_frame

    def panel_update_value(self, name, value):
        if name == 'label_light_pose_x':
            self.env_section.label_light_pose_x.config(text = value)
        elif name == 'label_light_pose_y':
            self.env_section.label_light_pose_y.config(text = value)
        elif name == 'label_steps':
            self.env_section.label_steps.config(text = value)
        elif name == 'entry_pose_x':
            self.robot_section.entry_pose_x.delete(0, END)
            self.robot_section.entry_pose_x.insert(0, value)
        elif name == 'entry_pose_y':
            self.robot_section.entry_pose_y.delete(0, END)
            self.robot_section.entry_pose_y.insert(0, value)
        elif name == 'entry_angle':
            self.robot_section.entry_angle.delete(0, END)
            self.robot_section.entry_angle.insert(0, str(value)[:6])
        elif name == 'num_sensors':
            self.sensors_section.entry_num_sensors.delete(0, END)
            self.sensors_section.entry_num_sensors.insert(0, str(value))
        else:
            print(f'PANEL_UPDATE_VALUE()->{name} NOT RECOGNIZED BY CONTEXT')        

    # SHARED METHODS
    def run_simulation(self):
        self.simulation_running = True
        self.route.delete()
        self.controller.set_ros_param('behavior', self.get_param('behavior'))
        self.controller.set_ros_param('run_behavior', self.simulation_running)
        self.controller.set_ros_param('step', 0)
        self.controller.set_ros_param('max_steps', int(self.get_param('max_steps')))
        self.controller.set_ros_param('max_advance', float(self.get_param('max_advance')))
        self.controller.set_ros_param('max_turn_angle', float(self.get_param('max_turn_angle')))
        self.controller.set_ros_param('light_threshold', float(self.get_param('light_threshold')))
        self.controller.set_ros_param('laser_threshold', float(self.get_param('laser_threshold')))
        self.controller.send_state_params()
    
    def plot_map(self):
        self.polygon_list, polygon_to_plot_list = self.controller.get_map(self.get_param('map'))
        self.grid.plot()

        self.canvas.delete('map')
        for polygon_to_plot in polygon_to_plot_list:
            self.canvas.create_polygon(
                polygon_to_plot, 
                outline = self.color['obstacle_outline'],
                fill = self.color['obstacle_inner'],
                width = 1,
                tag = 'map'
            )

    def get_polygon_list(self):
        print(f'POLYGONS NUM->{len(self.polygon_list)}')
        polygon_list = []
        for polygon_vertices in self.polygon_list:
            polygon_points = []
            for i in range(0, len(polygon_vertices), 2):
                point = self.controller.set_position(polygon_vertices[i], polygon_vertices[i+1])
                polygon_points.append(point)
            polygon_list.append(polygon_points)

            # print(f'polygon_points->{polygon_points}')
            # print(f'vertices_number->{len(polygon_points)}')
            # print('------------------------------')

        return polygon_list
