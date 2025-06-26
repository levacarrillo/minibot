import math

class Service():
    def __init__(self):
        self.canvas_scale = None
        # CANVA'S SIZE
        self.previus_size = None
        self.current_size = None

    def set_canvas_scale(self, x, y):
        self.canvas_scale = { 'x': x, 'y': y }
        return self.canvas_scale

    def set_canvas_size(self, x, y):
        self.previus_size = self.current_size
        self.current_size = { 'x': x, 'y': y }
        return self.current_size
    
    def set_position(self, x, y):
        return { 'x': x, 'y': y }

    def set_pose(self, x, y, angle):
        return { 'x': x, 'y': y, 'angle': angle }

    def remap_position(self, position):
        if self.previus_size is not None:
            position['x'] = self.current_size['x'] * position['x'] / self.previus_size['x']
            position['y'] = self.current_size['y'] * position['y'] / self.previus_size['y']
        return position
    
    def get_edge(self, size, scale, line_per_meters):
        return size / (scale * line_per_meters)

    def px_point_to_m(self, px, py):
        x = self.canvas_scale['x'] * px / self.current_size['x']
        y = self.canvas_scale['y'] * py / self.current_size['y']
        return str(x)[:6], str(y)[:6]

    def get_execution_delay(self, slider_value):
        delay = (3 - int(slider_value)) * 0.01
        return delay






    # MATH CONVERTIONS
    def pixels_to_m(self, scale, size, point):
        return str(scale * point / size)[:6]

    def m_to_pixels(self, length):
        return (float(length) * self.current_size['x']) / self.canvas_scale['x']

    def cartesian_to_polar(self, x, y):
        magnitude = math.sqrt(pow(x, 2) + pow(y, 2))
        angle = math.atan(y, x)
        return magnitude, angle

    def polar_to_cartesian(self, radius, angle):
        x = radius * math.cos(-angle)
        y = radius * math.sin(-angle)
        return x, y

    def radians_to_degrees(self, radians):
        return math.degrees(radians)

    def degrees_to_radians(self, degrees):
        return math.radians(degrees)        

    def normalize_angle(self, angle):
        angle = float(angle)
        if angle > math.pi * 2:
            return angle % (math.pi * 2)
        elif angle < 0:
            return math.pi * 2 - ((angle * -1) % (math.pi * 2))
        return angle

    def set_polygon_point(self, pose, radius, portion):
        sinT = math.sin(-pose['angle'])
        cosT = math.cos(-pose['angle'])
        x = radius * (portion['x'] * cosT - portion['y'] * sinT) + pose['x']
        y = radius * (portion['x'] * sinT + portion['y'] * cosT) + pose['y']
        return x, y

    def displace_point(self, initial_pose, distance, angle):
        return {
            'x': distance * math.cos(-(angle + initial_pose['angle'])) + initial_pose['x'],
            'y': distance * math.sin(-(angle + initial_pose['angle'])) + initial_pose['y'],
            'angle': angle + initial_pose['angle']
            }

    def format_params(self, params):
        behavior_list = params.behavior_list
        if '' in behavior_list:
            behavior_list.remove('')
        if 'UNKNOWN' in behavior_list:
            behavior_list.remove('UNKNOWN')

        param_dict = {
            "behavior" : params.behavior,
            "run_behavior" : params.run_behavior,
            "behavior_list" : behavior_list,
            "step" : params.step,
            "max_steps" : params.max_steps,
            "max_advance" : params.max_advance,
            "max_turn_angle" : params.max_turn_angle,
            "light_threshold" : params.light_threshold,
            "laser_threshold" : params.laser_threshold
        }

        return param_dict

    def get_lights_readings(self, robot_pose, robot_radius, light_pose):
        light_readings = []
        for i in range(8):
            sensor_angle = robot_pose['angle'] + i * math.pi / 4
            sensor_x = robot_pose['x'] + robot_radius * math.cos(-sensor_angle)
            sensor_y = robot_pose['y'] + robot_radius * math.sin(-sensor_angle)
            x_distance = sensor_x - light_pose['x']
            y_distance = sensor_y - light_pose['y']

            sensor_distance = math.sqrt(pow(x_distance, 2) + pow(y_distance, 2))
            light_readings.append(1 / sensor_distance)
            
        return light_readings

    def get_max_reading(self, list_values):
        max_index = 0
        max_value = 0.0
        for i in range(len(list_values)):
            if list_values[i] > max_value:
                max_index = i
                max_value = list_values[i]

        return max_index, max_value

    def format_goal_pose(self, goal):
        if goal is not None:
            distance = goal.distance * 500
            goal = {
                'angle'   : goal.angle,
                'distance': distance
            }
        return goal
