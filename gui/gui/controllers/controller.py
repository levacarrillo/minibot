from gui.domain.service import Service
from gui.infraestructure.file_manager import FileManager

class Controller:
    def __init__(self, ros):
        self.ros = ros
        self.ros_params = None
        self.service = Service()
        self.file_manager = FileManager()

    # FILE CONTROLLERS
    def simulate_light_proximity(self, robot_pose, robot_radius, light_pose):
        light_readings = self.service.get_lights_readings(robot_pose, robot_radius, light_pose)
        max_index, max_value = self.service.get_max_reading(light_readings)
        print(max_index)
        self.ros.simulate_light_proximity(light_readings, max_index, max_value)

    def get_file_path(self, file_name):
        return self.file_manager.get_file_path(file_name)

    def get_environment_list(self):
        return ["EMPTY", "HOME", "ARENA 1", "ARENA 2"]

    # SERVICE CONTROLLERS
    def set_dymension(self, x, y):
        return self.service.set_dymension(x, y)

    def set_pose(self, x, y, angle = 0):
        return self.service.set_pose(x, y, angle)

    def remap_pose(self, old_size, new_size, pose):
        return self.service.remap_pose(old_size, new_size, pose)

    def radians_to_degrees(self, radians):
        return self.service.radians_to_degrees(radians)

    def degrees_to_radians(self, degrees):
        return self.service.degrees_to_radians(degrees)

    def get_edge(self, size, scale, line_per_meters):
        return self.service.get_edge(size, scale, line_per_meters)

    def pixels_to_m(self, scale, size, point):
        return self.service.pixels_to_m(scale, size, point)

    def m_to_pixels(self, scale, size, length):
        return self.service.m_to_pixels(scale, size, length)

    def normalize_angle(self, angle):
        return self.service.normalize_angle(angle)

    def set_polygon_point(self, pose, radius, portion_radius):
        return self.service.set_polygon_point(pose, radius, portion_radius)

    def rotate_pose(self, initial_pose, angle):
        return self.service.rotate_pose(initial_pose, angle)

    def displace_point(self, initial_pose, distance, angle):
        return self.service.displace_point(initial_pose, distance, angle)


    # ROS CONTROLLERS
    def update_params(self):
        params = self.ros.update_params()
        self.ros_params = self.service.format_params(params)

    def run_simulation(self, params):
        self.ros.run_simulation(params)

    def get_param(self, param_name):
        # return self.ros_params[param_name]
        return "0.0"

    def movement_executing(self):
        return self.ros.movement_executing()

    def finish_movement(self):
        self.ros.finish_movement()

    def get_goal_pose(self):
        goal = self.ros.get_goal_pose()
        return self.service.format_goal_pose(goal)

    def cartesian_to_polar(self, x, y):
        return self.service.cartesian_to_polar(x, y)

    def polar_to_cartesian(self, radius, angle):
        return self.service.polar_to_cartesian(radius, angle)
