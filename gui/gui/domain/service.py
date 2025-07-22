import time
import math
import numpy as np
from interfaces.srv import SetParams


class Service():
    # FILES SERVICES
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
                    polygon = [vertices_xy for xy in zip(vertices_x, vertices_y) for vertices_xy in xy]
                    polygon_to_plot = [vertices_xy for xy in zip(vertices_x, vertices_yp) for vertices_xy in xy]
                    polygon_list.append(polygon)
                    polygon_to_plot_list.append(polygon_to_plot)
        return scale, polygon_to_plot_list

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
                obj = {
                    'name': words[0],
                    'x': float(words[1]) * canvas_size['width'],
                    'y': canvas_size['height'] - float(words[2]) * canvas_size['height']
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
    def set_pose(self, x, y, angle):
        return { 'x': x, 'y': y, 'angle': angle }

    def set_position(self, x, y):
        return { 'x': int(x), 'y': int(y) }

    def sum_vectors(self, p1, p2):
        return { 'x': p1['x'] + p2['x'], 'y': p1['y'] + p2['y'] }        

    def get_line_segment(self, p1, p2):
        return { 'x': p2['x'] - p1['x'], 'y': p2['y'] - p1['y'] }


    def px_point_to_m(self, px, py, canvas_size):
        x = px / canvas_size['width']
        y = (canvas_size['height'] - py)/ canvas_size['height']
        return str(x)[:6], str(y)[:6]

    def sleep(self, slider_value):
        delay = (3 - int(slider_value)) * 0.01
        time.sleep(delay)


    def transform_to_polygon_point(self, position, angle, radius, point):
        sinT = math.sin(-float(angle))
        cosT = math.cos(-float(angle))

        x = radius * (point['x'] * cosT - point['y'] * sinT) + position['x']
        y = radius * (point['x'] * sinT + point['y'] * cosT) + position['y']
        return x, y

    def get_laser_value(self, robot_pose, laser_max_point, points):
        l = laser_max_point
        robot_pose = self.change_sys_reference(robot_pose)
        laser_max_point = self.change_sys_reference(laser_max_point)
        print(f'\trobot_pose->{robot_pose}')
        print(f'\tlaser_max_point->{laser_max_point}')
        
        laser_segment = self.get_line_segment(robot_pose, laser_max_point)
        print(f'laser_segment->{laser_segment}')
        polygon_edge = None
        aux_segment  = None
        for i in range(0, len(points)):
            if i + 1 < len(points):
                print(f'\tsegment->{points[i]} - {points[i + 1]}')
                polygon_edge = self.get_line_segment(points[i], points[i + 1])
                print(f'polygon_edge->{polygon_edge}')
                aux_segment = self.get_line_segment(points[i], robot_pose)
            else:
                print(f'\tsegment->{points[0]} - {points[len(points) - 1]}')
                polygon_edge = self.get_line_segment(points[0], points[len(points) - 1])
                aux_segment = self.get_line_segment(points[0], robot_pose)
            
            matrix_det = np.array([ [laser_segment['x'], - polygon_edge['x']],
                                    [laser_segment['y'], - polygon_edge['y']]])

            matrix_det_t = np.array([[aux_segment['x'], - polygon_edge['x']],
                                     [aux_segment['y'], - polygon_edge['y']]])

            matrix_det_u = np.array([[laser_segment['x'], aux_segment['x']],
                                     [laser_segment['y'], aux_segment['y']]])  

            det = np.linalg.det(matrix_det)
            det_t = np.linalg.det(matrix_det_t)
            det_u = np.linalg.det(matrix_det_u)
            print(f'\t\tdet->{det}')
            print(f'\t\tdet_t->{det_t}')
            print(f'\t\tdet_u->{det_u}')
            if det != 0:
                t = - det_t / det
                u = - det_u / det
                print(f'\t\tt->{t} - u->{u}')
                if 0 <= t <= 1 and 0 <= u <= 1:
                    print(f'\t\tINTERSECTION!')
                    x = robot_pose['x'] + t * (laser_max_point['x'] - robot_pose['x'])
                    y = robot_pose['y'] + t * (laser_max_point['y'] - robot_pose['y'])

                    print(f'----------------{self.redo_sys_reference({ 'x': int(x), 'y': int(y) })}')
                    l = self.redo_sys_reference({ 'x': int(x), 'y': int(y) })

        return l
        # return self.redo_sys_reference(laser_max_point)

    # MATH CONVERTIONS
    def get_magnitude_between_two_points(self, p1, p2):
        return math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])

    def m_to_pixels(self, length, pixels_per_m):
        return  int(pixels_per_m * float(length))

    def polar_to_cartesian(self, radius, angle):
        x = radius * math.cos(-angle)
        y = radius * math.sin(-angle)
        return x, y
    
    def cartesian_to_polar(self, point):
        return math.hypot(point['x'], point['y']), math.atan2(point['y'], point['x'])

    def polar_to_cartesian_point(self, radius, angle):
        x = radius * math.cos(-angle)
        y = radius * math.sin(-angle)
        return { 'x': int(x), 'y': int(y) }
    

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

    def get_light_readings(self, robot_pose, light_pose, robot_angle, robot_radius):
        max_index = 0
        max_value = 0.0
        readings = []

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

        light_readings = {
            'max_index': max_index,
            'max_value': max_value,
            'readings' : readings
        }

        return light_readings
    
    def get_lidar_readings(self, laser_readings):
        return laser_readings

    def format_goal_pose(self, goal, canvas_size):
        if goal is not None:
            distance = goal.distance * canvas_size['width']
            goal = {
                'angle'   : goal.angle,
                'distance': distance
            }
        return goal
