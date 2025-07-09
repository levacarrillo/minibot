from tests.domain.service import Service


class Controller:
    def __init__(self, ros):
        self.ros = ros
        self.service = Service()

    def degrees_to_radians(self, degrees):
        return self.service.degrees_to_radians(degrees)