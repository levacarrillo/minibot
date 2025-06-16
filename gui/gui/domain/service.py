import math

class Service():
    def __init__(self):
        self.simulation_runnning = False

    def set_dymension(x, y):
        return { 'x': x, 'y': y }
    
    def set_pose(x, y, angle):
        return { 'x': x, 'y': y, 'angle': angle }

    def remap_pose(old_size, new_size, pose):
        pose['x'] = new_size['x'] * pose['x'] / old_size['x']
        pose['y'] = new_size['y'] * pose['y'] / old_size['y']
        return pose

    def get_edge(size, scale, line_per_meters):
        return size / (scale * line_per_meters)

    def pixels_to_m(scale, size, point):
        return str(scale * point / size)[:4]

    def m_to_pixels(scale, size, length):
        return (float(length) * size) / scale

    def radians_to_degrees(radians):
        return math.degrees(radians)

    def degrees_to_radians(degrees):
        return math.radians(degrees)        

    def normalize_angle(angle):
        angle = float(angle)
        if angle > math.pi * 2:
            return angle % (math.pi * 2)
        elif angle < 0:
            return math.pi * 2 - ((angle * -1) % (math.pi * 2))
        return angle

    def set_polygon_point(pose, radius, portion):
        sinT = math.sin(-pose['angle'])
        cosT = math.cos(-pose['angle'])
        x = radius * (portion['x'] * cosT - portion['y'] * sinT) + pose['x']
        y = radius * (portion['x'] * sinT + portion['y'] * cosT) + pose['y']
        return x, y
