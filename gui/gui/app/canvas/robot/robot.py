import time
import math
from gui.app.canvas.polygon import Polygon
from gui.app.canvas.robot.parts import *


class Robot:
    def __init__(self, canvas_panel):
        self.color      = canvas_panel.color
        self.canvas     = canvas_panel.canvas
        self.controller = canvas_panel.controller

        self.body    = False
        self.pose    = self.controller.set_pose(0, 0, 0)
        self.radius  = 0
        self.polygon = Polygon(self)
        self.trace_route = []

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
        self.head   = self.polygon.get(pose, radius, head_points(), color_name = 'head')
        self.hokuyo      = get_hokuyo(self.canvas, pose, radius, self.color)
        self.left_wheel  = self.polygon.get(pose, radius, left_wheel_points(),  'wheel')
        self.right_wheel = self.polygon.get(pose, radius, right_wheel_points(), 'wheel')

    def move(self, distance, angle):
        increment = 0
        initial_pose = self.pose
        final_point = self.pose

        while(increment < math.fabs(angle)):
            increment += math.radians(1)
            if angle < 0:
                new_pose = self.controller.rotate_pose(initial_pose, -increment)
            else:
                new_pose = self.controller.rotate_pose(initial_pose,  increment)

            self.plot(new_pose, self.radius)
            time.sleep(0.0001)
            self.canvas.update()

        increment = 0
        while(increment < math.fabs(distance)):
            increment += 1
            if distance < 0:
                new_pose = self.controller.displace_point(initial_pose, - increment, angle)
            else:
                new_pose = self.controller.displace_point(initial_pose,   increment, angle)

            self.plot(new_pose, self.radius)
            time.sleep(0.0001)
            final_point = new_pose
            self.canvas.update()

        self.trace_route.append(self.canvas.create_line(
                                                        initial_pose['x'],
                                                        initial_pose['y'],
                                                        final_point['x'],
                                                        final_point['y'],
                                                        dash = (4, 4),
                                                        fill = self.color['trace']
                                                        )
                                                    )


    def get_pose(self):
        return self.pose

    def delete(self):
        self.canvas.delete(self.body)
        self.canvas.delete(self.hokuyo)
        self.canvas.delete(self.head)
        self.canvas.delete(self.left_wheel)
        self.canvas.delete(self.right_wheel)