class Polygon:
    def __init__(self, parent_panel):
        self.color      = parent_panel.color
        self.canvas     = parent_panel.canvas
        self.controller = parent_panel.controller

    def get(self, pose, radius, points, color_name):
        polygon = []
        for point in points:
            polygon.append(self.controller.set_point_in_robot(pose, radius, point))

        return self.canvas.create_polygon(polygon, outline = self.color[color_name],
                                                fill = self.color[color_name], width = 1)