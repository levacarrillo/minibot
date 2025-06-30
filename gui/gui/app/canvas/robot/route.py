class Route:
    def __init__(self, context):
        controller  = context.controller
        self.canvas = context.canvas
        self.color  = context.color
        self.robot  = context.robot

        self.line = None
        self.trace_route = []
        context.set_route(self)

    def trace(self, initial_point, final_point):
        xi = initial_point['x']
        yi = initial_point['y']
        xf = final_point['x']
        yf = final_point['y']
        self.line = self.canvas.create_line(xi, yi, xf, yf, dash = (4, 4),
                                                        fill = self.color['trace']
                                                        )
        self.robot.plot()
        self.trace_route.append(self.line)

    def delete(self):
        for trace in self.trace_route:
            self.canvas.delete(trace)
        self.trace_route = []
