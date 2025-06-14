from gui.domain.service import Service
from gui.infraestructure.file_manager import FileManager

class Controller:
    def __init__(self, ros, service = Service):
        self.ros = ros
        self.service = service
        self.file_manager = FileManager()

    def radians_to_degrees(self, radians):
        return self.service.radians_to_degrees(radians)

    def degrees_to_radians(self, degrees):
        return self.service.degrees_to_radians(degrees)

    def get_file_path(self, file_name):
        return self.file_manager.get_file_path(file_name)

    def get_environment_list(self):
        return self.ros.get_environment_list()

    def get_behavior_list(self):
        return self.ros.get_behavior_list()

    def get_current_step(self):
        return self.ros.get_current_step()

    def get_edge(self, size, scale, line_per_meters):
        return self.service.get_edge(size, scale, line_per_meters)

    def pixels_to_m(self, scale, size, point):
        return self.service.pixels_to_m(scale, size, point)

    def m_to_pixels(self, scale, size, length):
        return self.service.m_to_pixels(scale, size, length)

    def normalize_angle(self, angle):
        return self.service.normalize_angle(angle)

    def set_point_in_robot(self, pose, radius, portion_radius):
        return self.service.set_point_in_robot(pose, radius, portion_radius)
