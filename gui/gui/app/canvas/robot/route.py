class Route:
    def __init__(self, context):
        controller  = context.controller
        self.canvas = context.canvas
        self.color  = context.color
        self.trace_route = []

    def trace(self, initial_point, final_point):
        i = initial_point
        f = final_point
        self.canvas.delete(self.trace_route)
        self.trace_route = []
        new_line = self.canvas.create_line(i['x'], i['y'], f['x'], f['y'], 
                                        dash = (4, 4), fill = self.color['trace']
                                                        )
        self.trace_route.append(new_line)
