import tkinter as tk
from tkinter import *
from tkinter import ttk
from tests.app.app_context import AppContext
import math

class App(tk.Tk):
    def __init__(self, controller):
        super().__init__()
        self.title("Mobile Robot GUI for Tests")

        content = Frame(self)

        context = AppContext(
            app        = self,
            content    = content,
            controller = controller
        )

        progress_var      = IntVar()
        self.radian_var   = StringVar()
        self.angle_var    = StringVar(content, value = "0°")
        self.distance_var = StringVar(content, value = "0.0cm")

        self.angle_var.trace_add("write", context.format_angle)
        self.distance_var.trace_add("write", context.format_distance)

        info_frame     = LabelFrame(content, text = "Status")
        velocity_frame = LabelFrame(content, text = "Move robot")  
        movement_frame = LabelFrame(content, text = "Move to pose")

        label_bot_id  = Label(info_frame, text = 'Minibot 2', font = ('arial', 11, "bold"), bg = "white")
        label_battery = Label(info_frame, text = 'Battery level:')
        label_percent = Label(info_frame, text = '80%')
        battery_bar   = ttk.Progressbar(info_frame, variable = progress_var, maximum = 100)
        battery_bar.step(80)

        canvas_center = 70
        canvas = Canvas(content, width = 2 * canvas_center, height = 2 * canvas_center)

        d_spot_light = 62
        robot_radius = 20
        spot_light_radius = 7
        head_cte = 10

        for i in range(8):
            x = canvas_center + d_spot_light * math.cos(i * math.pi / 4 - math.pi / 2)
            y = canvas_center + d_spot_light * math.sin(i * math.pi / 4 - math.pi / 2)
            canvas.create_oval( x - spot_light_radius,
                                y - spot_light_radius,
                                x + spot_light_radius,
                                y + spot_light_radius,
                                outline = "black")

        robot = canvas.create_oval( canvas_center - robot_radius,
                                    canvas_center - robot_radius,
                                    canvas_center + robot_radius,
                                    canvas_center + robot_radius,
                                    fill = "yellow")

        coords = [
            canvas_center,
            canvas_center - robot_radius,
            canvas_center - robot_radius + 8,
            canvas_center - head_cte,
            canvas_center + robot_radius - 8,
            canvas_center - head_cte]

        polygon = canvas.create_polygon(coords, fill="black", outline="black", width = 2)

        buton_arrow_left  = Button(velocity_frame, text = "<",  width = 2)
        buton_arrow_right = Button(velocity_frame, text = ">",  width = 2)
        buton_arrow_stop  = Button(velocity_frame, text = "||", width = 2)
        buton_arrow_up    = Button(velocity_frame, text = "^",  width = 2)
        buton_arrow_down  = Button(velocity_frame, text = "v",  width = 2)

        button_stop    = Button(movement_frame, text = "Stop",  command = context.on_click_stop)
        button_start   = Button(movement_frame, text = "Start", command = context.on_click_start)
        label_angle    = Label(movement_frame, text = "Angle:")
        label_distance = Label(movement_frame, text = "Distance:")
        radian_entry   = Entry(movement_frame, state = 'readonly', textvariable = self.radian_var, width = 9)
        angle_entry    = Spinbox(movement_frame, from_ = -180, to = 180, increment = 1, textvariable = self.angle_var, width = 4)
        distance_entry = Spinbox(movement_frame, from_ = -100, to=100, increment = 0.1, textvariable = self.distance_var, width = 7)

        content        .grid(column = 0, row = 0, padx = 10, pady = 10)

        info_frame     .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0),   pady = (5, 5), columnspan = 5)
        label_bot_id   .grid(column = 0, row = 0, sticky = (N, W), padx = (10, 10), pady = (15, 10))
        label_battery  .grid(column = 2, row = 0, sticky = (N, W), padx = (10, 0),  pady = (15, 10))
        label_percent  .grid(column = 3, row = 0, sticky = (N, W), padx = (5, 5),   pady = (15, 10))
        battery_bar    .grid(column = 4, row = 0, sticky = (N, W), padx = (0, 10),  pady = (18, 10))

        canvas         .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0),   pady = (15, 10))

        velocity_frame    .grid(column = 3, row = 1, sticky = (N, W), padx = (5, 5),  pady = (5, 5),  columnspan = 3)
        buton_arrow_left  .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0), pady = (0, 0),   columnspan = 2)
        buton_arrow_right .grid(column = 4, row = 1, sticky = (N, W), padx = (5, 10),  pady = (0, 0), columnspan = 2)
        buton_arrow_stop  .grid(column = 2, row = 1, sticky = (N, W), padx = (5, 0),  pady = (0, 0),  columnspan = 2)
        buton_arrow_up    .grid(column = 2, row = 0, sticky = (N, W), padx = (5, 0), pady = (10, 5),  columnspan = 2)
        buton_arrow_down  .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10),  columnspan = 2)

        movement_frame .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 5), pady = (5, 5), columnspan = 5)        
        
        button_start   .grid(column = 3, row = 3, sticky = (N, W), padx = (10, 0), pady = (10, 10), columnspan = 2)
        button_stop    .grid(column = 4, row = 3, sticky = (N, W), padx = (10, 0), pady = (10, 10), columnspan = 2)
        label_angle    .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        angle_entry    .grid(column = 1, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        radian_entry   .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        label_distance .grid(column = 3, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        distance_entry .grid(column = 4, row = 2, sticky = (N, W), padx = (5, 5), columnspan = 1)


    def run(self):
        self.mainloop()
