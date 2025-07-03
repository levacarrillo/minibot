class Polygon:
    def __init__(self, context):
        self.color      = context.color
        self.canvas     = context.canvas
        self.controller = context.controller

    def get(self, anchor, radius, points, color_name, tag = None):
        polygon = []
        for point in points:
            polygon.append(self.controller.set_polygon_point(anchor, radius, point))

        return self.canvas.create_polygon(polygon, outline = self.color[color_name],
                                            fill = self.color[color_name], width = 1, tag = tag)
