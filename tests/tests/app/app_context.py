class AppContext:
    def __init__(self, app, content, controller):
        self.app        = app
        self.content    = content
        self.controller = controller

        self.radians  = None
        self.distance = None

    def on_click_start(self):
        self.controller.move_robot(self.radians, self.distance)

    def on_click_stop(self):
        self.controller.cancel_movement()

    def format_angle(self, *args):
        degrees = self.app.angle_var.get().replace("°", "")
        self.app.angle_var.set(degrees + "°")
        self.radians = self.controller.degrees_to_radians(degrees)
        self.app.radian_var.set(f"{self.radians:.4f} rad")

    def format_distance(self, *args):
        distance_cm = self.app.distance_var.get().replace("cm", "") + "cm"
        self.distance = self.controller.cm_to_m(distance_cm.replace("cm", ""))
        self.app.distance_var.set(distance_cm)
