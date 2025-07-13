import math


class Service():
    def cm_to_m(self, cm):
        return float(cm) * 0.01

    def degrees_to_radians(self, degrees):
        try:
            return math.radians(int(degrees))
        except:
            return 0

    def format_vel(self, vel):
        try:
            return float(vel)
        except:
            return 0.0

    def get_spot_light_coords(self, i, angle_min, edge):
        step_angle = i * math.pi / 4 - angle_min
        x = edge * (1 / 2 + 4 * math.cos(step_angle) / 11)
        y = edge * (1 / 2 + 4 * math.sin(step_angle) / 11)
        coords = [
                x - edge / 22,
                y - edge / 22,
                x + edge / 22,
                y + edge / 22]
        return coords

    def get_laser_coords(self, i, reading,  num_readings, edge):
        step_angle = - i * math.pi / num_readings
        
        coords = [
            edge * (1 / 2 + math.cos(step_angle) / 11),
            edge * (1 / 2 + math.sin(step_angle) / 11),
            edge * (1 / 2 + 3 * reading * math.cos(step_angle) / 11),
            edge * (1 / 2 + 3 * reading * math.sin(step_angle) / 11)]
        return coords