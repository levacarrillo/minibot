from gui.domain.service import Service

class Controller:
    def __init__(self, service = Service):
        self.service = service

    def get_file_path(self, file_name):
        return '/home/knight/ros2_ws/src/minibot/gui/gui/resources/light.png'

    def get_enviroments(self):
        enviroments = ["EMPTY", "HOME", "ARENA 1", "ARENA 2"]
        return enviroments

    def get_behavior_list(self):
        behavior_list = ["LIGHT_FOLLOWER", "SM_DESTINATION", "SM_AVOID_OBSTACLES"]
        return behavior_list

    def get_current_step(self):
        return '0'
