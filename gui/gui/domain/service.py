import math
from interfaces.srv import SetParams


class Service():
    def __init__(self):
        self.ros_params   = None
        # CANVA'S SIZE DEFAULT VALUES
        self.canvas_scale = { 'x': 1, 'y': 1 }
        self.previus_size = None
        self.current_size = { 'x': 500, 'y': 500 } # PIXELS

    # FILES SERVICES
    def parse_map(self, map_file):
        vertices_list = []
        coords = None
        lines = map_file.readlines()
        for line in lines:
            words = line.split()
            if words and words[0] == "(": # IGNORE EMPTY LINES AND COMMENTS
                if words[1] == "dimensions":
                    self.set_canvas_scale(float(words[3]), float(words[4])) 
                    print(f'NEW CANVAS_DIMENSION->{self.canvas_scale}')

                    # self.current_size = self.set_canvas_size(
                    #         self.m_to_pixels(words[3]), self.m_to_pixels(words[4]))
                elif words[1] == "polygon":
                    vertices_x = [self.m_to_pixels(x) for x in words[4:len(words)-1:2]]
                    vertices_y = [self.m_to_pixels(y) for y in words[5:len(words)-1:2]]
                    # vertex_y_calculus = [float(y) for y in words[5:len(words)-1:2]]
                    coords = [coord for xy in zip(vertices_x, vertices_y) for coord in xy]
                    vertices_list.append(coords)

        return self.current_size, vertices_list

    # GUI'S SERVICES
    def get_canvas_size(self):
        return self.current_size

    def set_canvas_size(self, x, y):
        self.previus_size = self.current_size
        self.current_size = { 'x': x, 'y': y }
        return self.current_size
    
    def set_canvas_scale(self, x, y):
        self.canvas_scale = { 'x': x, 'y': y }

    def set_position(self, x, y):
        return { 'x': x, 'y': y }

    def set_pose(self, x, y, angle):
        return { 'x': x, 'y': y, 'angle': angle }

    def remap_position(self, position):
        if self.previus_size is not None:
            position['x'] = self.current_size['x'] * position['x'] / self.previus_size['x']
            position['y'] = self.current_size['y'] * position['y'] / self.previus_size['y']
        return position

    def get_edge(self, axis, line_per_meters):
        return self.current_size[axis] / (self.canvas_scale[axis] * line_per_meters)

    def px_point_to_m(self, px, py):
        x = self.canvas_scale['x'] * px / self.current_size['x']
        y = self.canvas_scale['y'] * py / self.current_size['y']
        return str(x)[:6], str(y)[:6]

    def get_execution_delay(self, slider_value):
        delay = (3 - int(slider_value)) * 0.01
        return delay

    def set_polygon_point(self, pose, radius, portion):
        sinT = math.sin(-pose['angle'])
        cosT = math.cos(-pose['angle'])
        x = radius * (portion['x'] * cosT - portion['y'] * sinT) + pose['x']
        y = radius * (portion['x'] * sinT + portion['y'] * cosT) + pose['y']
        return x, y

    # MATH CONVERTIONS
    def m_to_pixels(self, length):
        return (float(length) * self.current_size['x']) / self.canvas_scale['x']

    def polar_to_cartesian(self, radius, angle):
        x = radius * math.cos(-angle)
        y = radius * math.sin(-angle)
        return x, y

    def degrees_to_radians(self, degrees):
        return math.radians(degrees)        

    def normalize_angle(self, angle):
        angle = float(angle)
        if angle > math.pi * 2:
            return angle % (math.pi * 2)
        elif angle < 0:
            return math.pi * 2 - ((angle * -1) % (math.pi * 2))
        return angle

    # ROS SERVICES
    def format_ros_params(self, params):
        behavior_list = params.behavior_list
        if '' in behavior_list:
            behavior_list.remove('')
        if 'UNKNOWN' in behavior_list:
            behavior_list.remove('UNKNOWN')
        
        max_advance = math.trunc(params.max_advance * 1000) / 1000
        laser_threshold = math.trunc(params.laser_threshold * 1000) / 1000

        self.ros_params = {
            "behavior" :       params.behavior,
            "run_behavior" :   params.run_behavior,
            "behavior_list" :  behavior_list,
            "step" :           params.step,
            "max_steps" :      params.max_steps,
            "max_advance" :    max_advance,
            "max_turn_angle":  params.max_turn_angle,
            "light_threshold": params.light_threshold,
            "laser_threshold": laser_threshold
        }

    def get_current_step(self, params):
        self.ros_params['step'] = params.step
        return self.ros_params['step']

    def get_ros_param(self, param_name):
        return self.ros_params[param_name]

    def set_ros_param(self, name, value):
        self.ros_params[name] = value

    def get_all_params(self):
        req = SetParams.Request()
        req.behavior        = self.ros_params['behavior']
        req.run_behavior    = self.ros_params['run_behavior']
        req.step            = self.ros_params['step']
        req.max_steps       = self.ros_params['max_steps']
        req.max_advance     = self.ros_params['max_advance']
        req.max_turn_angle  = self.ros_params['max_turn_angle']
        req.light_threshold = self.ros_params['light_threshold']
        req.laser_threshold = self.ros_params['laser_threshold']
        return req

    def get_light_readings(self, robot_pose, robot_radius, light_pose):
        max_index = 0
        max_value = 0.0
        readings = []

        for i in range(8):
            sensor_angle = robot_pose['angle'] + i * math.pi / 4
            sensor_x = robot_pose['x'] + robot_radius * math.cos(-sensor_angle)
            sensor_y = robot_pose['y'] + robot_radius * math.sin(-sensor_angle)
            x_distance = sensor_x - light_pose['x']
            y_distance = sensor_y - light_pose['y']

            simulated_reading = 1 / math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))
            
            if simulated_reading > max_value:
                max_value = simulated_reading
                max_index = i

            readings.append(simulated_reading)

        light_readings = {
            'max_index': max_index,
            'max_value': max_value,
            'readings' : readings
        }

        return light_readings
    
    def get_lidar_readings(self):
        lidar_readings = [0.25, 0.25, 0.25]
        return lidar_readings

    def format_goal_pose(self, goal):
        if goal is not None:
            distance = goal.distance * self.current_size['x']
            goal = {
                'angle'   : goal.angle,
                'distance': distance
            }
        return goal
