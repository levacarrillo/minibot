import math


class Service():
    def __init__(self):
        print('SERVICE...')

    def degrees_to_radians(self, degrees):
        try:
            radians = math.radians(int(degrees))
            return f"{radians:.4f} rad"
        except:
            return " - -"