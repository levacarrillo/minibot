class Grid:
    def __init__(self, context):
        self.context = context
        self.line_per_meters = 10
        context.set_grid(self)

    def plot(self):
        self.context.canvas.delete('grid')
        canvas_size  = self.context.get_canvas_size()
        canvas_scale = self.context.get_canvas_scale()

        for axis in canvas_size:
            line = canvas_size[axis] / ( self.line_per_meters * canvas_scale[axis])
            for i in range(1, int(self.line_per_meters * canvas_scale[axis])):
                points = [i * line if axis == 'width' else 0,
                          i * line if axis != 'width' else 0,
                          i * line if axis == 'width' else canvas_size[axis],
                          i * line if axis != 'width' else canvas_size[axis]]
                self.context.canvas.create_line(points, dash = (4, 4),
                                    fill = self.context.color['grid'], tag  = 'grid' )
