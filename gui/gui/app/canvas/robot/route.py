class Route:
    def __init__(self, context):
        controller  = context.controller
        self.canvas = context.canvas
        self.color  = context.color

        self.line = None
        self.trace_route = []
        context.set_route(self)

    def trace(self, xi, yi, xf, yf):
        self.canvas.delete(self.trace_route)
        self.line = self.canvas.create_line(xi, yi, xf, yf, dash = (4, 4),
                                                        fill = self.color['trace']
                                                        )
        self.trace_route.append(self.line)

    def delete(self):
        self.canvas.delete(self.trace_route)
        self.trace_route = []
