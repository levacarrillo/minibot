class Grid:
    def __init__(self, canvas_panel):
        self.canvas_size  = canvas_panel.size
        self.canvas_scale = canvas_panel.scale
        self.color        = canvas_panel.color
        self.canvas       = canvas_panel.canvas
        self.controller   = canvas_panel.controller
        self.grid = []

    def plot(self, line_per_meters = 10):
        for i in self.grid:
            self.canvas.delete(i)
        self.grid = []

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
