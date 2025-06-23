import math

class Service():
    def __init__(self):
        self.simulation_runnning = False

    def set_dymension(self, x, y):
        return { 'x': x, 'y': y }
    
    def set_pose(self, x, y, angle):
        return { 'x': x, 'y': y, 'angle': angle }

    def remap_pose(old_size, new_size, pose):
        pose['x'] = new_size['x'] * pose['x'] / old_size['x']
        pose['y'] = new_size['y'] * pose['y'] / old_size['y']
        return pose

    def get_edge(self, size, scale, line_per_meters):
        return size / (scale * line_per_meters)

    def pixels_to_m(self, scale, size, point):
        return str(scale * point / size)[:4]

    def m_to_pixels(self, scale, size, length):
        return (float(length) * size) / scale

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

    def rotate_pose(self, initial_pose, angle):
        return {
                'x': initial_pose['x'],
                'y': initial_pose['y'],
                'angle': initial_pose['angle'] + angle
                }

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
