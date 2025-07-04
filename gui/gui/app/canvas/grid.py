class Grid:
    def __init__(self, context):
        self.context      = context
        self.color        = context.color
        self.canvas       = context.canvas
        self.controller   = context.controller
        self.line_per_meters = 10

        self.grid = []
        context.set_grid(self)

    def plot(self):
        canvas_size  = self.context.canvas_size
        for i in self.grid:
            self.canvas.delete(i)

        for axis in canvas_size:
            edge = self.controller.get_edge(axis, self.line_per_meters)
            for i in range(0, canvas_size[axis]):
                self.grid.append(
                    self.canvas.create_line(
                        i * edge if axis == 'x' else 0,
                        i * edge if axis != 'x' else 0,
                        i * edge if axis == 'x' else canvas_size[axis],
                        i * edge if axis != 'x' else canvas_size[axis],
                        dash=(4, 4),
                        fill=self.color['grid']
                    )
                )
