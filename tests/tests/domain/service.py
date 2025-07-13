import math


class Service():
    def validate_vel(self, vel):
        try:
            return float(vel)
        except:
            return 0.0

    def degrees_to_radians(self, degrees):
        try:
            return math.radians(int(degrees))
        except:
            return 0

    def cm_to_m(self, cm):
        return float(cm) * 0.01

    def get_head_coords(self, edge):
        coords = [
            edge / 2,
            edge / 2 - edge / 11,
            edge / 2 - edge / 11 + 8,
            edge / 2 - 10,
            edge / 2 + edge / 11 - 8,
            edge / 2 - 10]
        return coords

    def get_body_coords(self, edge):
        coords = [
            edge * (9 / 22),
            edge * (9 / 22),
            edge * (13 / 22),
            edge * (13 / 22)]
        return coords

    def get_hokuyo_coords(self, edge):
        coords = [
            edge * (53 / 110),
            edge * (53 / 110),
            edge * (57 / 110),
            edge * (57 / 110)]
        return coords

    def validate_lights_response(self, response):
        return response.max_index if response else None

    def norm_lidar_response(self, response):
        angle_range   = 0.0
        num_readings  = 0
        readings_norm = []
        if response:
            num_readings = len(response.scan)
            angle_range  = response.angle_min - response.angle_max
            for i in range(len(response.scan)):
                readings_norm.append(response.scan[i] / response.max_value)

        return angle_range, readings_norm, num_readings

    def get_spot_light_coords(self, i, edge):
        step_angle = i * math.pi / 4 - math.pi / 2
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
