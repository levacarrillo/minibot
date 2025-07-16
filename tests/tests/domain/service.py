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

    def get_head_coords(self, width):
        coords = [
            width / 2,
            width / 2 - width / 11,
            width / 2 - width / 11 + 8,
            width / 2 - 10,
            width / 2 + width / 11 - 8,
            width / 2 - 10]
        return coords

    def get_body_coords(self, width):
        coords = [
            width * (9 / 22),
            width * (9 / 22),
            width * (13 / 22),
            width * (13 / 22)]
        return coords

    def get_hokuyo_coords(self, width):
        coords = [
            width * (53 / 110),
            width * (53 / 110),
            width * (57 / 110),
            width * (57 / 110)]
        return coords

    def validate_lights_response(self, response):
        return response.max_index if response else None

    def norm_lidar_response(self, response):
        lidar_params = {
            'angle_min':    0.0,
            'num_readings' :  0,
            'readings_norm': []
        }

        if response:
            lidar_params['angle_min'] = response.angle_min
            lidar_params['num_readings'] = len(response.scan)
            for i in range(len(response.scan)):
                lidar_params['readings_norm'].append(response.scan[i] / response.max_value)

        return lidar_params

    def get_spot_light_coords(self, i, width):
        step_angle = i * math.pi / 4 - math.pi / 2
        x = width * (1 / 2 + 4 * math.cos(step_angle) / 11)
        y = width * (1 / 2 + 4 * math.sin(step_angle) / 11)
        coords = [
                x - width / 22,
                y - width / 22,
                x + width / 22,
                y + width / 22]

        return coords

    def get_laser_coords(self, i, lidar_params, width):
        angle_min    = lidar_params['angle_min']
        num_readings = lidar_params['num_readings']
        reading      = lidar_params['readings_norm'][i]

        step_angle = i * (angle_min - math.pi / 2) / num_readings
        
        coords = [
            width * (1 / 2 + math.cos(step_angle) / 11),
            width * (1 / 2 + math.sin(step_angle) / 11),
            width * (1 / 2 + 4 * reading * math.cos(step_angle) / 11),
            width * (1 / 2 + 4 * reading * math.sin(step_angle) / 11)]

        return coords

    def format_params(self, params):
        if params is not None:
            behavior_list = params.behavior_list
            behavior_list.remove('') if '' in behavior_list else None
            behavior_list.remove('UNKNOWN') if 'UNKNOWN' in behavior_list else None

            return {
                'behavior': params.behavior,
                'run_behavior': params.run_behavior,
                'behavior_list': params.behavior_list,
                'step': params.step,
                'max_steps': params.max_steps,
                'max_advance': params.max_advance,
                'max_turn_angle': params.max_turn_angle,
                'light_threshold': params.light_threshold,
                'laser_threshold': params.laser_threshold
            }
        return None
