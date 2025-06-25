class Route:
    def __init__(self, context):
        controller  = context.controller
        self.canvas = context.canvas
        self.color  = context.color
        self.trace_route = []
        self.initial_point = controller.set_pose(0, 0)
        self.final_point   = controller.set_pose(0, 0)

    def trace(self, initial_point, final_point):
        self.initial_point = initial_point
        self.final_point   = final_point
        self.canvas.delete(self.trace_route)
        self.trace_route = []
        self.trace_route.append(self.canvas.create_line(
                                                        initial_point['x'],
                                                        initial_point['y'],
                                                        final_point['x'],
                                                        final_point['y'],
                                                        dash = (4, 4),
                                                        fill = self.color['trace']
                                                        )
                                                    )
    def get_init_point(self):
        return self.initial_point

    def get_final_point(self):
        return self.final_point