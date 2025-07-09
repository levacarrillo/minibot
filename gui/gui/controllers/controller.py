from gui.domain.service import Service
from gui.infraestructure.file_manager import FileManager


class Controller:
    def __init__(self, ros):
        self.ros = ros
        self.service = Service()
        self.file_manager = FileManager()

    # FILE CONTROLLERS
    def check_for_topological_map(self, file_name):
        return self.file_manager.check_for_topological_map(file_name)        

    def get_file_path(self, file_name):
        return self.file_manager.get_file_path(file_name)

    def get_environment_list(self):
        return self.file_manager.get_environment_list()

    def get_map(self, file_name):
        map_file = self.file_manager.get_map(file_name)
        return self.service.parse_map(map_file)

    def get_topological_map(self, file_name, topological):
        map_file = self.file_manager.get_map(file_name, topological)
        return self.service.parse_topological_map(map_file)

    def load_objects(self):
        objects_file = self.file_manager.load_objects()
        return self.service.parse_objects_file(objects_file)

    def get_object_released(self, name, robot_pose, robot_radius):
        return self.service.get_object_released(name, robot_pose, robot_radius)

    # SERVICE CONTROLLERS
    def get_canvas_size(self):
        return self.service.get_canvas_size()

    def set_canvas_size(self, x, y):
        return self.service.set_canvas_size(x, y)

    def set_pose(self, x, y, angle = 0):
        return self.service.set_pose(x, y, angle)

    def set_position(self, x, y):
        return self.service.set_position(x, y)

    def sum_vectors(self, p1, p2):
        return self.service.sum_vectors(p1, p2)

    def get_line_segment(self, p1, p2):
        return self.service.get_line_segment(p1, p2)

    def remap_position(self, position):
        return self.service.remap_position(position)

    def degrees_to_radians(self, degrees):
        return self.service.degrees_to_radians(degrees)

    def get_edge(self, axis, line_per_meters):
        return self.service.get_edge(axis, line_per_meters)

    def px_point_to_m(self, px, py):
        return self.service.px_point_to_m(px, py)

    def set_polygon_point(self, pose, radius, portion_radius):
        return self.service.set_polygon_point(pose, radius, portion_radius)

    def get_execution_delay(self, slider_value):
        return self.service.get_execution_delay(slider_value)

    def get_magnitude_between_two_points(self, p1, p2):
        return self.service.get_magnitude_between_two_points(p1, p2)

    def m_to_pixels(self, length):
        return self.service.m_to_pixels(length)

    def normalize_angle(self, angle):
        return self.service.normalize_angle(angle)

    def polar_to_cartesian(self, radius, angle):
        return self.service.polar_to_cartesian(radius, angle)

    def polar_to_cartesian_point(self, radius, angle):
        return self.service.polar_to_cartesian_point(radius, angle)

    def cartesian_to_polar(self, point):
        return self.service.cartesian_to_polar(point)
 
    def get_laser_value(self, robot_pose, laser_max_point, polygon_points):
        return self.service.get_laser_value(robot_pose, laser_max_point, polygon_points)
        
    # ROS CONTROLLERS
    def simulate_light_readings(self, robot_pose, robot_radius, light_pose):
        light_readings = self.service.get_light_readings(robot_pose, 
                                                        robot_radius, light_pose)
        self.ros.set_light_readings(light_readings)

    def simulate_lidar_readings(self, laser_readings):
        lidar_readings = self.service.get_lidar_readings(laser_readings)
        self.ros.set_lidar_readings(lidar_readings)

    def update_params(self):
        self.service.format_ros_params(self.ros.update_params())

    def get_current_step(self):
        return self.service.get_current_step(self.ros.update_params())

    def get_param(self, param_name):
        return self.service.get_ros_param(param_name)

    def set_ros_param(self, name, value):
        return self.service.set_ros_param(name, value)

    def send_state_params(self):
        self.ros.send_state_params(self.service.get_all_params())

    def finish_movement(self):
        self.ros.finish_movement()

    def get_goal_pose(self):
        return self.service.format_goal_pose(self.ros.get_goal_pose())
