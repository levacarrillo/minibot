from gui.app.canvas.polygon import Polygon
from gui.app.canvas.robot.body   import get_body
from gui.app.canvas.robot.hokuyo import get_hokuyo
from gui.app.canvas.robot.head import head_points
from gui.app.canvas.robot.wheels import left_wheel_points, right_wheel_points

class Robot:
    def __init__(self, canvas_panel):
        self.color      = canvas_panel.color
        self.canvas     = canvas_panel.canvas
        self.controller = canvas_panel.controller

        self.body    = False
        self.pose    = self.controller.set_pose(0, 0, 0)
        self.radius  = 0
        self.polygon = Polygon(self)

    def plot(self, pose, radius):
        self.pose = pose
        self.radius = radius

        if self.body:
            self.canvas.delete(self.head)
            self.canvas.delete(self.body)
            self.canvas.delete(self.hokuyo)
            self.canvas.delete(self.left_wheel)
            self.canvas.delete(self.right_wheel)

        self.head   = self.polygon.get(pose, radius, head_points(), color_name = 'head')
        self.body        = get_body(self.canvas, pose, radius, self.color)
        self.hokuyo      = get_hokuyo(self.canvas, pose, radius, self.color)
        self.left_wheel  = self.polygon.get(pose, radius, left_wheel_points(),  'wheel')
        self.right_wheel = self.polygon.get(pose, radius, right_wheel_points(), 'wheel')

    def get_pose(self):
        return self.pose

    def delete(self):
        self.canvas.delete(self.body)
        self.canvas.delete(self.hokuyo)
        self.canvas.delete(self.head)
        self.canvas.delete(self.left_wheel)
        self.canvas.delete(self.right_wheel)