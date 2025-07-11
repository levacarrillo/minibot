import math


class Service():
    def __init__(self):
        print('SERVICE...')

    def cm_to_m(self, cm):
        return float(cm) * 0.01

    def degrees_to_radians(self, degrees):
        try:
            return math.radians(int(degrees))
        except:
            return 0

    def format_vel(self, vel):
        try:
            return float(vel)
        except:
            return 0.0

