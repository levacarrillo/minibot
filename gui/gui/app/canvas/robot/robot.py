from gui.app.canvas.polygon import Polygon
from gui.app.canvas.robot.body   import get_body
from gui.app.canvas.robot.hokuyo import get_hokuyo
from gui.app.canvas.robot.head import head_points
from gui.app.canvas.robot.wheels import left_wheel_points, right_wheel_points

class Robot:
    def __init__(self, canvas_panel):
        self.canvas_panel  = canvas_panel
        self.color         = canvas_panel.color
        self.canvas        = canvas_panel.canvas
        self.robot_section = canvas_panel.robot_section
        self.controller    = canvas_panel.controller
        self.canvas_scale  = canvas_panel.scale
        self.canvas_size   = canvas_panel.size

        self.pose   = self.controller.set_pose(-10, -10, 0) # OUT OF CANVAS
        self.radius = 0
        self.angle  = 0

        self.body   = False
        self.polygon = Polygon(self)


    def plot(self, pose_x = None , pose_y = None):
        if (pose_x and pose_y) is None:
            new_size = self.canvas_panel.size
            self.pose = self.controller.remap_pose(self.canvas_size, new_size, self.pose)
        else:
            self.angle  = self.controller.normalize_angle(self.robot_section.entry_angle.get())
            self.pose = self.controller.set_pose(pose_x, pose_y, self.angle)

        self.canvas_size = self.canvas_panel.size
        self.radius = self.controller.m_to_pixels(
                                                    self.canvas_scale['x'],
                                                    self.canvas_size['x'],
                                                    self.robot_section.entry_radius.get()
                                                )



        if self.body:
            self.canvas.delete(self.head)
            self.canvas.delete(self.body)
            self.canvas.delete(self.hokuyo)
            self.canvas.delete(self.left_wheel)
            self.canvas.delete(self.right_wheel)

        self.head  = self.polygon.get(self.pose, self.radius, head_points(), color_name = 'head')
        self.body = get_body(self.canvas, self.pose, self.radius, self.color)
        self.hokuyo = get_hokuyo(self.canvas, self.pose, self.radius, self.color)
        self.left_wheel  = self.polygon.get(self.pose, self.radius, left_wheel_points(),  'wheel')
        self.right_wheel = self.polygon.get(self.pose, self.radius, right_wheel_points(), 'wheel')

    def delete(self):
        self.canvas.delete(self.body)
        self.canvas.delete(self.hokuyo)
        self.canvas.delete(self.head)
        self.canvas.delete(self.left_wheel)
        self.canvas.delete(self.right_wheel)