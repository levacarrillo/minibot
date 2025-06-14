from gui.app.canvas.polygon import Polygon

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
            self.pose = self.controller.scale_pose(self.canvas_size, new_size, self.pose)
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


        self.body = self.canvas.create_oval(
            self.pose['x'] - self.radius,
            self.pose['y'] - self.radius,
            self.pose['x'] + self.radius,
            self.pose['y'] + self.radius,
            outline = self.color['robot'],
            fill    = self.color['robot'],
            width   = 1
        )

        self.hokuyo = self.canvas.create_oval(
            self.pose['x'] - (self.radius / 5),
            self.pose['y'] - (self.radius / 5),
            self.pose['x'] + (self.radius / 5),
            self.pose['y'] + (self.radius / 5),
            outline = self.color['hokuyo'],
            fill    = self.color['hokuyo'],
            width   = 1
        )

        # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
        head_points = [ 
            { 'x': 2/3, 'y': - 1/3 },
            { 'x': 2/3, 'y':   1/3 },
            { 'x': 5/6, 'y':    0  }
        ]

        left_wheel_points = [
            {'x': -1/2, 'y': -5/6 },
            {'x':  1/2, 'y': -5/6 },
            {'x':  1/2, 'y': -3/6 },
            {'x': -1/2, 'y': -3/6 }
        ]

        right_wheel_points = [ 
            {'x': -1/2, 'y':  3/6 },
            {'x':  1/2, 'y':  3/6 },
            {'x':  1/2, 'y':  5/6 },
            {'x': -1/2, 'y':  5/6 },
        ]

        self.head  = self.polygon.get(self.pose, self.radius, head_points, color_name = 'head')
        self.left_wheel  = self.polygon.get(self.pose, self.radius, left_wheel_points,  'wheel')
        self.right_wheel = self.polygon.get(self.pose, self.radius, right_wheel_points, 'wheel')

    def delete(self):
        self.canvas.delete(self.body)
        self.canvas.delete(self.hokuyo)
        self.canvas.delete(self.head)
        self.canvas.delete(self.left_wheel)
        self.canvas.delete(self.right_wheel)