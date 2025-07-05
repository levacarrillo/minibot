class Parts:
    def __init__(self, context):
        self.color      = context.color
        self.canvas     = context.canvas
        self.controller = context.controller

        self.anchor = None
        self.radius = None

    def get(self, anchor, radius, name):
        self.anchor = anchor
        self.radius = radius

        if name == 'body':
            return self.get_body()
        elif name == 'hokuyo':
            return self.get_hokuyo()
        elif name == 'head':
            return self.join_polygon(self.get_head_points(), 'head')
        elif name == 'left_wheel':
            return self.join_polygon(self.get_lw_points(), 'wheel')
        elif name == 'right_wheel':
            return self.join_polygon(self.get_rw_points(), 'wheel')

    def join_polygon(self, points, color_name):
        polygon = []
        for point in points:
            polygon.append(self.controller.set_polygon_point(self.anchor, self.radius, point))

        return self.canvas.create_polygon(polygon, outline = self.color[color_name],
                            fill = self.color[color_name], width = 1, tag = 'robot')

    def get_head_points(self):
        # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
        return [ 
                    { 'x': 2/3, 'y': - 1/3 },
                    { 'x': 2/3, 'y':   1/3 },
                    { 'x': 5/6, 'y':    0  }
                ]

    def get_body(self):
        return self.canvas.create_oval(
            self.anchor['x'] - self.radius,
            self.anchor['y'] - self.radius,
            self.anchor['x'] + self.radius,
            self.anchor['y'] + self.radius,
            outline = self.color['robot'],
            fill    = self.color['robot'],
            width   = 1,
            tag = "robot"
        )

    def get_hokuyo(self):
        return self.canvas.create_oval(
                self.anchor['x'] - (self.radius / 5),
                self.anchor['y'] - (self.radius / 5),
                self.anchor['x'] + (self.radius / 5),
                self.anchor['y'] + (self.radius / 5),
                outline = self.color['hokuyo'],
                fill    = self.color['hokuyo'],
                width   = 1,
                tag = "robot"
            )

    def get_lw_points(self):
        # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
        return [
                {'x': -1/2, 'y': -5/6 },
                {'x':  1/2, 'y': -5/6 },
                {'x':  1/2, 'y': -3/6 },
                {'x': -1/2, 'y': -3/6 }
            ]

    def get_rw_points(self):
        # POINTS MAGNITUDES RELATIVE TO ROBOT'S RADIUS
        return [
                {'x': -1/2, 'y':  3/6 },
                {'x':  1/2, 'y':  3/6 },
                {'x':  1/2, 'y':  5/6 },
                {'x': -1/2, 'y':  5/6 },
            ]
