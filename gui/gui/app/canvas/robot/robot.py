import time
import math
from gui.app.canvas.robot.parts import *
from gui.app.canvas.polygon import Polygon
from gui.app.canvas.robot.route import Route


class Robot:
    def __init__(self, canvas_panel):
        self.light      = canvas_panel.light
        self.color      = canvas_panel.color
        self.canvas     = canvas_panel.canvas
        self.controller = canvas_panel.controller

        self.body    = False
        self.pose    = self.controller.set_pose(0, 0, 0)
        self.radius  = 0
        self.polygon = Polygon(self)
        self.route   = Route(self)

    def plot(self, pose, radius):
        self.pose = pose
        self.radius = radius

        if self.body:
            self.canvas.delete(self.head)
            self.canvas.delete(self.body)
            self.canvas.delete(self.hokuyo)
            self.canvas.delete(self.left_wheel)
            self.canvas.delete(self.right_wheel)

        self.body        = get_body(self.canvas, pose, radius, self.color)
        self.hokuyo      = get_hokuyo(self.canvas, pose, radius, self.color)
        self.head        = self.polygon.get(pose, radius, head_points(), 'head', tag = "robot")
        self.left_wheel  = self.polygon.get(pose, radius, l_wheel_points(),'wheel', tag = "robot")
        self.right_wheel = self.polygon.get(pose, radius, r_wheel_points(),'wheel', tag = "robot")

    def rotate(self, direction):
        increment = self.controller.degrees_to_radians(1)
        new_pose  = self.controller.rotate_pose(self.pose, direction * increment)
        self.plot(new_pose, self.radius)

    def displace(self, direction):
        x, y = self.controller.polar_to_cartesian(direction, self.pose['angle'])
        self.canvas.move('robot', x, y)
        return abs(direction)

    #     self.controller.simulate_light_proximity(self.get_pose(), self.radius, 
                                                    # self.light.get_pose())
    #         self.route.trace(initial_pose, new_pose)

    def get_pose(self):
        return self.pose

    def delete(self):
        self.canvas.delete(self.body)
        self.canvas.delete(self.hokuyo)
        self.canvas.delete(self.head)
        self.canvas.delete(self.left_wheel)
        self.canvas.delete(self.right_wheel)