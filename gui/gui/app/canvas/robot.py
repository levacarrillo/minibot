class Robot:
    def __init__(self, canvas_panel):
        self.color         = canvas_panel.color
        self.canvas        = canvas_panel.canvas
        self.robot_section = canvas_panel.robot_section
        self.controller    = canvas_panel.controller

        self.scale         = canvas_panel.scale
        self.size          = canvas_panel.size

        self.pose   = { 'x': 0, 'y': 0, 'angle': 0}
        self.radius = 0
        self.angle  = 0

        self.head   = False
        self.body   = False
        self.hokuyo = False


    def plot(self, pose_x, pose_y):
        self.angle  = self.controller.normalize_angle(self.robot_section.entry_angle.get())
        self.radius = self.controller.m_to_pixels(
                                                    self.scale['x'],
                                                    self.size['x'],
                                                    self.robot_section.entry_radius.get()
                                                )
        self.pose = { 'x': pose_x, 'y': pose_y, 'angle': self.angle }

        if self.body:
            self.canvas.delete(self.head)
            self.canvas.delete(self.body)
            self.canvas.delete(self.hokuyo)
            self.canvas.delete(self.wheelL)
            self.canvas.delete(self.wheelR)

        head = []
        points = [ # LENGHTS RELATIVE TO ROBOT'S RADIUS
            { 'x': 2/3, 'y': - 1/3 },
            { 'x': 2/3, 'y':   1/3 },
            { 'x': 5/6, 'y':    0  }
        ]

        for point in points:
            head.append(self.controller.set_point_in_robot(self.pose, self.radius, point))

        self.head = self.canvas.create_polygon(head, outline = self.color['head'],
                                                        fill =  self.color['head'], width = 1)

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


        wheel1x1 = self.pose['x'] - (self.radius / 2)
        wheel1y1 = self.pose['y'] - (5 * self.radius /6)
        wheel1x2 = self.pose['x'] + self.radius / 2
        wheel1y2 = self.pose['y'] - (3 * self.radius / 6)
        wheel2y1 = self.pose['y'] + (3 * self.radius / 6)
        wheel2y2 = self.pose['y'] + (5 * self.radius / 6)
        wh1= []
        wh2= []
        wh1.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x1, wheel1y1))
        wh1.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x2, wheel1y1))
        wh1.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x2, wheel1y2))
        wh1.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x1, wheel1y2))
        wh2.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x1, wheel2y1))
        wh2.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x2, wheel2y1))
        wh2.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x2, wheel2y2))
        wh2.append(self.controller.rotate_point(self.angle, self.pose['x'], self.pose['y'], wheel1x1, wheel2y2))
        self.wheelL = self.canvas.create_polygon(wh1, outline = self.color['wheel'], fill = self.color['wheel'], width=1)
        self.wheelR = self.canvas.create_polygon(wh2, outline = self.color['wheel'], fill = self.color['wheel'], width=1)


    def delete(self):
        self.canvas.delete(self.body)
        self.canvas.delete(self.hokuyo)
        self.canvas.delete(self.head)
        self.canvas.delete(self.wheelL)
        self.canvas.delete(self.wheelR)