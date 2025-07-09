class AppContext:
    def __init__(self, app, content, controller):
        self.app        = app
        self.content    = content
        self.controller = controller

    def format_angle(self, *args):
        degrees = self.app.angle_var.get().replace("°", "")
        self.app.angle_var.set(degrees + "°")
        radians = self.controller.degrees_to_radians(degrees)
        self.app.radian_var.set(radians)

    def format_distance(self, *args):
        distance = self.app.distance_var.get().replace("cm", "") + "cm"
        self.app.distance_var.set(distance)
