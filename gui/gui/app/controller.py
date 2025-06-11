from gui.domain.service import Service

class Controller:
    def __init__(self, service = Service):
        self.service = service

    def get_enviroments():
        enviroments = ["Empty", "Home", "Stage"]
        return enviroments

    def get_behavior_list():
        behavior_list = ["LIGHT_FOLLOWER", "SM_DESTINATION", "SM_AVOID_OBSTACLES"]
        return behavior_list

    def get_current_step():
        return '0'
    
    def resize_canvas(width, height):
        print('withxheight')