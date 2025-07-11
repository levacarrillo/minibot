from tests.domain.service import Service


class Controller:
    def __init__(self, ros):
        self.ros = ros
        self.service = Service()

    def send_vel(self, linear, angular):
        self.ros.pub_vel(linear, angular)

    def send_pose(self, angle, distance):
        self.ros.send_goal(angle, distance)
    
    def cancel_movement(self):
        self.ros.cancel_goal()

    def cm_to_m(self, cm):
        return self.service.cm_to_m(cm)

    def degrees_to_radians(self, degrees):
        return self.service.degrees_to_radians(degrees)

    def format_vel(self, vel):
        return self.service.format_vel(vel)

    def get_light_readings(self):
        return self.ros.get_light_readings()

    def get_lidar_readings(self):
        return self.ros.get_lidar_readings()
