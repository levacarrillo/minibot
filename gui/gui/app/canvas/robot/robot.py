from gui.app.canvas.robot.parts import *
from gui.app.canvas.polygon import Polygon
from gui.app.canvas.robot.route import Route


class Robot:
    def __init__(self, context):
        self.context    = context
        self.color      = context.color
        self.canvas     = context.canvas
        self.controller = context.controller

        self.pose    = None
        self.radius  = None

        self.body    = None
        context.set_robot(self)

        self.polygon = Polygon(context)
        self.route   = Route(context)

    def plot(self, position):
        entry_angle  = self.context.robot_section.entry_angle .get()
        entry_radius = self.context.robot_section.entry_radius.get()

        angle       = self.controller.normalize_angle(entry_angle)
    
        self.radius = self.controller.m_to_pixels(entry_radius)
        self.pose   = self.controller.set_pose(position['x'], position['y'], angle)


        if self.body:
            self.canvas.delete(self.body)
            self.canvas.delete(self.head)
            self.canvas.delete(self.hokuyo)
            self.canvas.delete(self.left_wheel)
            self.canvas.delete(self.right_wheel)

        self.body        = get_body(self.canvas, self.pose, self.radius, self.color)
        self.hokuyo      = get_hokuyo(self.canvas, self.pose, self.radius, self.color)
        self.head        = self.polygon.get(self.pose, self.radius, head_points(), 'head', tag = "robot")
        self.left_wheel  = self.polygon.get(self.pose, self.radius, l_wheel_points(),'wheel', tag = "robot")
        self.right_wheel = self.polygon.get(self.pose, self.radius, r_wheel_points(),'wheel', tag = "robot")

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

    # def get_pose(self):
    #     return self.pose

    def delete(self):
        self.canvas.delete(self.body)
        self.canvas.delete(self.hokuyo)
        self.canvas.delete(self.head)
        self.canvas.delete(self.left_wheel)
        self.canvas.delete(self.right_wheel)