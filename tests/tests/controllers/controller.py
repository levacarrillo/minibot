from tests.domain.service import Service


class Controller:
    def __init__(self, ros):
        self.ros = ros
        self.service = Service()

    def move_robot(self, angle, distance):
        self.ros.send_goal(angle, distance)
    
    def cancel_movement(self):
        self.ros.cancel_goal()

    def cm_to_m(self, cm):
        return self.service.cm_to_m(cm)

    def degrees_to_radians(self, degrees):
        return self.service.degrees_to_radians(degrees)