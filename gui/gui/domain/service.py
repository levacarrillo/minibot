import math

class Service():
    def __init__(self):
        self.simulation_runnning = False

    def get_edge(self, size, scale, line_per_meters):
        return size / (scale * line_per_meters)

    def pixels_to_m(self, scale, size, point):
        return str(scale * point / size)[:4]

    def m_to_pixels(self, scale, size, length):
        return (float(length) * size) / scale

    def normalize_angle(self, angle):
        angle = float(angle)
        if angle > math.pi * 2:
            return angle % (math.pi * 2)
        elif angle < 0:
            return math.pi * 2 - ((angle * -1) % (math.pi * 2))
        return angle

    def rotate_point(self, angle, ox, oy, x, y):
        nx = (x - ox ) * math.cos(-angle) - (y - oy) * math.sin(-angle) + ox
        ny = ( x - ox ) * math.sin(-angle) + (y - oy) * math.cos(-angle) + oy
        return nx, ny