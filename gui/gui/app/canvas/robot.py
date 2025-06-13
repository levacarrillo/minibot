class Robot:
    def __init__(self, canvas_panel):
        self.color         = canvas_panel.color
        self.canvas        = canvas_panel.canvas
        self.robot_section = canvas_panel.robot_section
        self.controller    = canvas_panel.controller

        radius       = self.robot_section.entry_radius.get()
        self.pose_x  = self.robot_section.entry_pose_x.get()
        self.pose_y  = self.robot_section.entry_pose_y.get()
        self.angle   = self.controller.normalize_angle(float(self.robot_section.entry_angle.get()))

        scale_x      = canvas_panel.scale_x
        size_x       = canvas_panel.size_x

        self.radius =  self.controller.m_to_pixels(scale_x, size_x, radius)
        self.robot  = False
        self.hokuyo = False
        self.head   = False


    def plot(self, pose_x, pose_y):
        if self.robot:
            self.canvas.delete(self.robot)
            self.canvas.delete(self.hokuyo)
            self.canvas.delete(self.head)

        self.robot = self.canvas.create_oval(
            pose_x - self.radius,
            pose_y - self.radius,
            pose_x + self.radius,
            pose_y + self.radius,
            outline = self.color['robot'],
            fill    = self.color['robot'],
            width   = 1
        )

        self.hokuyo = self.canvas.create_oval(
            pose_x - (self.radius / 5),
            pose_y - (self.radius / 5),
            pose_x + (self.radius / 5),
            pose_y + (self.radius / 5),
            outline = self.color['hokuyo'],
            fill    = self.color['hokuyo'],
            width   = 1
        )
        head = []
        head.append(self.controller.rotate_point(self.angle, pose_x, pose_y, pose_x + (2 * self.radius / 3), pose_y - ( self.radius / 3)))
        head.append(self.controller.rotate_point(self.angle, pose_x, pose_y, pose_x + (2 * self.radius / 3), pose_y + ( self.radius / 3)))
        head.append(self.controller.rotate_point(self.angle, pose_x, pose_y, pose_x + (5 * self.radius / 6), pose_y))

        self.head = self.canvas.create_polygon(
            head,
            outline = self.color['head'],
            fill    =  self.color['head'],
            width   = 1
        )
