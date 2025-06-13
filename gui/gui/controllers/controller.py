from gui.domain.service import Service
from gui.infraestructure.file_manager import FileManager

class Controller:
    def __init__(self, service = Service):
        self.service = service
        self.file_manager = FileManager()

    def get_file_path(self, file_name):
        return self.file_manager.get_file_path(file_name)

    def get_enviroments(self):
        enviroments = ["EMPTY", "HOME", "ARENA 1", "ARENA 2"]
        return enviroments


    def get_behavior_list(self):
        behavior_list = ["LIGHT_FOLLOWER", "SM_DESTINATION", "SM_AVOID_OBSTACLES"]
        return behavior_list

    def get_current_step(self):
        return '0'

    def get_edge(self, size, scale, line_per_meters):
        return self.service.get_edge(self, size, scale, line_per_meters)

    def pixels_to_m(self, scale, size, point):
        return self.service.pixels_to_m(self, scale, size, point)

    def m_to_pixels(self, scale, size, length):
        return self.service.m_to_pixels(self, scale, size, length)
