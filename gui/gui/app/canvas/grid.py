class Grid:
    def __init__(self, context):
        self.color        = context.color
        self.canvas       = context.canvas
        self.controller   = context.controller
        self.canvas_size  = context.canvas_size
        self.canvas_scale = context.canvas_scale
        self.grid = []
        context.set_grid(self)


    def plot(self, line_per_meters = 10):
        for i in self.grid:
            self.canvas.delete(i)

        for axis in self.canvas_scale:
            edge = self.controller.get_edge(
                                            self.canvas_size[axis],
                                            self.canvas_scale[axis], 
                                            line_per_meters
                                            )
            for i in range(0, int(self.canvas_scale[axis]) * line_per_meters):
                self.grid.append(
                    self.canvas.create_line(
                        i * edge if axis == 'x' else 0,
                        i * edge if axis != 'x' else 0,
                        i * edge if axis == 'x' else self.canvas_size[axis],
                        i * edge if axis != 'x' else self.canvas_size[axis],
                        dash=(4, 4),
                        fill=self.color['grid']
                    )
                )
