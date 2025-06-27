from gui.domain.service import Service
from gui.infraestructure.file_manager import FileManager


class Controller:
    def __init__(self, ros):
        self.ros = ros
        self.service = Service()
        self.file_manager = FileManager()

    # FILE CONTROLLERS
    def get_file_path(self, file_name):
        return self.file_manager.get_file_path(file_name)

    def get_environment_list(self):
        return ["EMPTY", "HOME", "ARENA 1", "ARENA 2"]

    # SERVICE CONTROLLERS
    def set_canvas_scale(self, x, y):
        return self.service.set_canvas_scale(x, y)

    def set_canvas_size(self, x, y):
        return self.service.set_canvas_size(x, y)

    def set_pose(self, x, y, angle = 0):
        return self.service.set_pose(x, y, angle)

    def set_position(self, x, y):
        return self.service.set_position(x, y)

    def remap_position(self, position):
        return self.service.remap_position(position)

    def degrees_to_radians(self, degrees):
        return self.service.degrees_to_radians(degrees)

    def get_edge(self, size, scale, line_per_meters):
        return self.service.get_edge(size, scale, line_per_meters)

    def px_point_to_m(self, px, py):
        return self.service.px_point_to_m(px, py)

    def set_polygon_point(self, pose, radius, portion_radius):
        return self.service.set_polygon_point(pose, radius, portion_radius)

    def get_execution_delay(self, slider_value):
        return self.service.get_execution_delay(slider_value)

    def m_to_pixels(self, length):
        return self.service.m_to_pixels(length)

    def normalize_angle(self, angle):
        return self.service.normalize_angle(angle)

    def polar_to_cartesian(self, radius, angle):
        return self.service.polar_to_cartesian(radius, angle)

    # ROS CONTROLLERS
    def simulate_light_readings(self, robot_pose, robot_radius, light_pose):
        light_readings = self.service.get_light_readings(robot_pose, 
                                                        robot_radius, light_pose)
        self.ros.set_light_readings(light_readings)

    def update_params(self):
        self.service.format_ros_params(self.ros.update_params())

    def get_current_step(self):
        return self.service.get_current_step(self.ros.update_params())

    def get_param(self, param_name):
        return self.service.get_ros_param(param_name)

    def set_ros_param(self, name, value):
        return self.service.set_ros_param(name, value)

    def run_simulation(self):
        self.ros.run_simulation(self.service.get_all_params())

    def finish_movement(self):
        self.ros.finish_movement()

    def get_goal_pose(self):
        return self.service.format_goal_pose(self.ros.get_goal_pose())
