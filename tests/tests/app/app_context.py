class AppContext:
    def __init__(self, app, content, controller):
        self.app        = app
        self.content    = content
        self.controller = controller

        self.radians  = None
        self.distance = None

        self.linear_vel  = 0.2
        self.angular_vel = 0.1

    def on_click_start(self):
        self.controller.send_pose(self.radians, self.distance)

    def on_click_stop(self):
        self.controller.cancel_movement()

    def move_robot(self, movement):
        if movement == 'LEFT':
            self.controller.send_vel(0.0,  self.angular_vel)
        elif movement == 'RIGHT':
            self.controller.send_vel(0.0, -self.angular_vel)
        elif movement == 'STOP':
            self.controller.send_vel(0.0, 0.0)
        elif movement == 'FORWARD':
            self.controller.send_vel(self.linear_vel, 0.0)
        elif movement == 'BACKWARD':
            self.controller.send_vel(-self.linear_vel, 0.0)
        else: 
            print(f'MOVEMENT {movement} DOES NOT RECOGNIZED')

    def format_angle(self, *args):
        degrees = self.app.angle_var.get().replace("°", "")
        self.app.angle_var.set(degrees + "°")
        self.radians = self.controller.degrees_to_radians(degrees)
        self.app.radian_var.set(f"{self.radians:.4f} rad")

    def format_distance(self, *args):
        distance_cm = self.app.distance_var.get().replace("cm", "") + "cm"
        self.distance = self.controller.cm_to_m(distance_cm.replace("cm", ""))
        self.app.distance_var.set(distance_cm)
