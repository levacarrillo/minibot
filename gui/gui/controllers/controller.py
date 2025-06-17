from gui.domain.service import Service
from gui.infraestructure.file_manager import FileManager

class Controller:
    def __init__(self, ros, service = Service):
        self.ros = ros
        self.service = service
        self.file_manager = FileManager()

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

    # FILE CONTROLLERS
    def get_file_path(self, file_name):
        return self.file_manager.get_file_path(file_name)

    # ROS CONTROLLERS
    def get_environment_list(self):
        return self.ros.get_environment_list()

    def get_behavior_list(self):
        return self.ros.get_behavior_list()

    def get_max_advance(self):
        return self.ros.get_max_advance()

    def get_lidar_threshold(self):
        return self.ros.get_lidar_threshold()

    def get_max_turn_angle(self):
        return self.ros.get_max_turn_angle()

    def get_current_step(self):
        return self.ros.get_current_step()
    
    def get_max_steps(self):
        return self.ros.get_max_steps()

    def get_goal_point(self):
        return self.ros.get_goal_point()

    def movement_executing(self):
        return self.ros.movement_executing()

    def stop_movement(self):
        self.ros.stop_movement()
