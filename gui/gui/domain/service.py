import time
import math
import numpy as np
import random

class Service():

    def remap_position(self, position, canvas_size, previus_canvas_size):
        position['x'] = canvas_size['width']  * position['x'] / previus_canvas_size['width']
        position['y'] = canvas_size['height'] * position['y'] / previus_canvas_size['height']
        return position

    def parse_map(self, map_file, canvas_size):
        scale = None
        polygon_list = []
        polygon_to_plot_list = []
        polygon = None
        polygon_to_plot = None
        lines = map_file.readlines()
        for line in lines:
            words = line.split()
            if words and words[0] == "(": # IGNORE EMPTY LINES AND COMMENTS
                if words[1] == "dimensions":
                    scale = { 'width': float(words[3]), 'height': float(words[4]) }
                elif words[1] == "polygon":
                    vertices_x  = [float(x) * canvas_size['width']  / scale['width']  for x in words[4:len(words)-1:2]]
                    vertices_y  = [float(y) * canvas_size['height'] / scale['height'] for y in words[5:len(words)-1:2]]
                    vertices_yp = [canvas_size['height'] - (float(y) * canvas_size['height'] / scale['height']) for y in words[5:len(words)-1:2]]
                    polygon = [vertices_xy for xy in zip(vertices_x, vertices_yp) for vertices_xy in xy]
                    polygon_to_plot = [vertices_xy for xy in zip(vertices_x, vertices_yp) for vertices_xy in xy]
                    polygon_list.append(polygon)
                    polygon_to_plot_list.append(polygon_to_plot)
        return scale, polygon_list, polygon_to_plot_list

    # def parse_topological_map(self, map_file):
    #     node_coords = []
    #     node_coords_to_plot = []
    #     connections = []
    #     if map_file is None:
    #         return None, None, None
    #     lines = map_file.readlines()

    #     for line in lines:
    #         words = line.split()
    #         if words and words[0] == "(": # IGNORE EMPTY LINES AND COMMENTS
    #             if words[1] == "num":
    #                 number_nodes = float(words[3])
    #             elif words[1] == "node":
    #                 # print(f'node->[{words[3]}, {words[4]}], px->[{self.m_to_pixels(words[3])}, {self.m_to_pixels(words[4])}]')
    #                 node_id = words[2]
    #                 node = { 'x': self.m_to_pixels(words[3]), 'y': self.m_to_pixels(words[4]) }
    #                 node_to_plot = { 'x': self.m_to_pixels(words[3]), 'y': self.current_size['y'] - self.m_to_pixels(words[4])}
    #                 node_coords.append(node)
    #                 node_coords_to_plot.append(node_to_plot)
    #             elif words[1] == "connection":
    #                 connections.append([int(words[2]), int(words[3])])
    #     return node_coords, node_coords_to_plot, connections

    def parse_objects_file(self, objects_file, canvas_size):
        if objects_file is None:
            return None
        lines = objects_file.readlines()
        object_list = []
        for line in lines:
            words = line.split()
            if words:
                x = float(words[1]) * canvas_size['width']
                y = canvas_size['height'] - float(words[2]) * canvas_size['height']
                obj = {
                    'name': words[0],
                    'x': x,
                    'y': y,
                    'rectangle': [x - 10, y - 10, x + 10, y + 10]
                }
                object_list.append(obj)

        return object_list
    
    def get_object_released(self, name, robot_pose, robot_radius):
        r = robot_radius + 10
        obj = {
            'name': name,
            'x': robot_pose['x'] + r * math.cos(-robot_pose['angle']),
            'y': robot_pose['y'] + r * math.sin(-robot_pose['angle'])
        }

        return obj

    # GUI'S SERVICES
    def format_to_position(self, x, y):
        return { 'x': x, 'y': y }      

    def get_line_segment(self, p1, p2):
        return { 'x': p2['x'] - p1['x'], 'y': p2['y'] - p1['y'] }

    def multiply_scalar_vector(self, vector, scalar):
        return { 'x': scalar * vector['x'], 'y': scalar * vector['y']}

    def px_point_to_m(self, px, py, canvas_size):
        x = px / canvas_size['width']
        y = (canvas_size['height'] - py)/ canvas_size['height']
        return str(x)[:6], str(y)[:6]

    def sleep(self, slider_value):
        delay = (3 - int(slider_value)) * 0.01
        time.sleep(delay)

    def generate_lasers_readings(self, robot_state, params, polygons, noise_param):
        lasers_list   = []
        lasers_values = [0.0]
        for i in range(params['num_sensors']):
            noise = self.get_noise(noise_param) if noise_param else 0
            angle = params['start_angle'] + i * params['step_angle']

            laser_max_vect  = self.polar_to_cartesian_point(params['max_value'] + noise, angle)

            laser_max_point = self.sum_vectors(robot_state['position'], laser_max_vect)
            laser_value_point = self.check_for_obstacle(
                robot_state['position'], laser_max_point, polygons)

            lasers_list.append(self.format_to_line(robot_state['position'], laser_value_point))
            lidar_values = self.format_lidar_data(
                params['origin_angle'],
                params['range_sensors'],
                params['max_value'],
                lasers_values)
        return lasers_list, lidar_values
        
    def transoform_to_points_list(self, list_of_list):
        points_lists = []
        for list in list_of_list:
            points = []
            for i in range(0, len(list), 2):
                point = self.format_to_position(list[i], list[i+1])
                points.append(point)
            points_lists.append(points)
        return points_lists

    def format_to_robot_state(self, position, angle, radius):
        return {
            'position': position,
            'angle'   : angle,
            'radius'  : radius
        }

    def format_to_sensors_params(
            self, robot_angle, num_sensors, origin_angle, range_sensors, max_value):
        start_angle = robot_angle + origin_angle
        step_angle  = range_sensors / ( num_sensors - 1 )
        return {
            'range_sensors': range_sensors,
            'origin_angle': origin_angle,
            'num_sensors': num_sensors,
            'start_angle': start_angle,
            'step_angle':  step_angle,
            'max_value' :  max_value
        }

    def check_for_obstacle(self, l0, l1, polygons_points):
        min_t = None
        for polygon_points in polygons_points:
            det_s = None
            for i in range(0, len(polygon_points)):
                if i + 1 < len(polygon_points):
                    det_s = - (l1['x'] - l0['x']) * (polygon_points[i+1]['y'] - polygon_points[i]['y']) + (l1['y'] - l0['y']) * (polygon_points[i+1]['x'] - polygon_points[i]['x'])
                    if det_s == 0:
                        continue
                    det_t = - (polygon_points[i]['x'] - l0['x']) * (polygon_points[i+1]['y'] - polygon_points[i]['y']) + (polygon_points[i]['y'] - l0['y']) * (polygon_points[i+1]['x'] - polygon_points[i]['x'])
                    det_u =   (l1['x'] - l0['x']) * (polygon_points[i]['y'] - l0['y']) - (l1['y'] - l0['y']) * (polygon_points[i]['x'] - l0['x'])
                else:
                    det_s = - (l1['x'] - l0['x']) * (polygon_points[0]['y'] - polygon_points[len(polygon_points) - 1]['y']) + (l1['y'] - l0['y']) * (polygon_points[0]['x'] - polygon_points[len(polygon_points) - 1]['x'])
                    if det_s == 0:
                        continue
                    det_t = - (polygon_points[len(polygon_points) - 1]['x'] - l0['x']) * (polygon_points[0]['y'] - polygon_points[len(polygon_points) - 1]['y']) + (polygon_points[len(polygon_points) - 1]['y'] - l0['y']) * (polygon_points[0]['x'] - polygon_points[len(polygon_points) - 1]['x'])
                    det_u =   (l1['x'] - l0['x']) * (polygon_points[len(polygon_points) - 1]['y'] - l0['y']) - (l1['y'] - l0['y']) * (polygon_points[len(polygon_points) - 1]['x'] - l0['x'])
                if det_s != 0:
                    t = det_t / det_s
                    u = det_u / det_s
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        if min_t is None:
                            min_t = t
                        elif t < min_t:
                            min_t = t
        if min_t and min_t >= 0:
            rel_laser = self.get_line_segment(l0, l1)
            new_laser = self.sum_vectors(l0, self.multiply_scalar_vector(rel_laser, min_t))
            return new_laser
        return l1

    def get_magnitude_between_two_points(self, p1, p2):
        return math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])

    def m_to_pixels(self, length, pixels_per_m):
        return  int(pixels_per_m * float(length))

    def polar_to_cartesian(self, radius, angle):
        x = radius * math.cos(-angle)
        y = radius * math.sin(-angle)
        return x, y
    
    def get_line_magnitude(self, point):
        return math.hypot(point['x'], point['y'])

    def polar_to_cartesian_point(self, radius, angle):
        x = radius * math.cos(-angle)
        y = radius * math.sin(-angle)
        return { 'x': int(x), 'y': int(y) }
    

    def degrees_to_radians(self, degrees):
        return math.radians(degrees)        

    def normalize_angle(self, angle):
        angle = float(angle)
        if angle > math.pi * 2:
            angle = angle % (math.pi * 2)
        elif angle < 0:
            angle = math.pi * 2 - ((angle * -1) % (math.pi * 2))
        
        angle = math.trunc(angle * 10000) / 10000
        return angle

    def format_ros_params(self, params):
        behavior_list = params.behavior_list
        if '' in behavior_list:
            behavior_list.remove('')
        if 'UNKNOWN' in behavior_list:
            behavior_list.remove('UNKNOWN')
        
        max_advance = math.trunc(params.max_advance * 1000) / 1000
        laser_threshold = math.trunc(params.laser_threshold * 1000) / 1000
        light_threshold = math.trunc(params.light_threshold * 1000) / 1000

        return {
            "behavior" :       params.behavior,
            "run_behavior" :   params.run_behavior,
            "behavior_list" :  behavior_list,
            "step" :           params.step,
            "max_steps" :      params.max_steps,
            "max_advance" :    max_advance,
            "max_turn_angle":  params.max_turn_angle,
            "light_threshold": light_threshold,
            "laser_threshold": laser_threshold
        }

    def simulate_light_data(self, robot_pose, light_pose, robot_angle, robot_radius):
        max_index =   0
        max_value = 0.0
        readings  =  []

        for i in range(8):
            sensor_angle = robot_angle + i * math.pi / 4
            sensor_x = robot_pose['x'] + robot_radius * math.cos(-sensor_angle)
            sensor_y = robot_pose['y'] + robot_radius * math.sin(-sensor_angle)
            x_distance = sensor_x - light_pose['x']
            y_distance = sensor_y - light_pose['y']
            simulated_reading = 1 / math.hypot(x_distance, y_distance)
            
            if simulated_reading > max_value:
                max_value = simulated_reading
                max_index = i

            readings.append(simulated_reading)

        light_data = {
            'max_index': max_index,
            'max_value': max_value,
            'readings' : readings
        }

        return light_data
    
    def format_lidar_data(self, angle_min, angle_max, max_value, lasers_values):
        lidar_data = {
            'angle_min': angle_min,
            'angle_max': angle_max,
            'max_value': float(max_value),
            'readings':  lasers_values
        } 
        return lidar_data

    def format_goal_pose(self, goal, canvas_size):
        if goal is not None:
            distance = goal.distance * canvas_size['width']
            goal = {
                'angle'   : goal.angle,
                'distance': distance
            }
        return goal

    def transform_to_polygon_point(self, anchor, angle, radius, point):
        sinT = math.sin(-float(angle))
        cosT = math.cos(-float(angle))

        x = radius * (point['x'] * cosT - point['y'] * sinT) + anchor['x']
        y = radius * (point['x'] * sinT + point['y'] * cosT) + anchor['y']
        return x, y

    def get_circle_coords(self, center, radius):
        return [center['x'] - radius,
                center['y'] - radius,
                center['x'] + radius,
                center['y'] + radius]

    def get_grid_line(self, canvas_axis_size, canvas_axis_scale, line_per_meters):
        return canvas_axis_size / (line_per_meters * canvas_axis_scale)
    
    def get_line_points(self, i, line, axis, canvas_axis_size):
        return [i * line if axis == 'width' else 0,
                i * line if axis != 'width' else 0,
                i * line if axis == 'width' else canvas_axis_size,
                i * line if axis != 'width' else canvas_axis_size]

    def get_polygon(self, relative_points, anchor, angle, radius):
        polygon = []
        for point in relative_points:
            polygon.append(
                self.transform_to_polygon_point(anchor, angle, radius, point))
        return polygon

    def format_figure_to_plot(self, coords, color):
        return {
            'color':  color,
            'coords': coords
        }

    def sum_vectors(self, p1, p2):
        return { 'x': p1['x'] + p2['x'], 'y': p1['y'] + p2['y'] }  

    def format_to_line(self, p1, p2):
        return [p1['x'], p1['y'], p2['x'], p2['y']]

    def get_noise(self, sigma):
        return int(random.gauss(0, sigma))
