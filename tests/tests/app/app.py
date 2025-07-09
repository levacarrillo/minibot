import tkinter as tk
from tkinter import *
from tests.app.app_context import AppContext


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

        labelframe = LabelFrame(content)  

        button_stop   = Button(labelframe, text = "Stop", command = context.on_click_stop)
        button_start  = Button(labelframe, text = "Start",  command = context.on_click_start)
        button_params = Button(labelframe, text = "Params")
        label_bot_id  = Label (labelframe, text = 'Minibot 2',
                    font = ('arial', 11, "bold"), bg = "white")

        self.radian_var   = StringVar()
        self.angle_var    = StringVar(content, value = "170°")
        self.distance_var = StringVar(content, value = "99.0cm")

        label_angle = Label(content, text = "Angle:")
        label_distance = Label(content, text = "Distance:")

        self.angle_var.trace_add("write", context.format_angle)
        self.distance_var.trace_add("write", context.format_distance)

        angle_entry    = Spinbox(content, from_ = -180, to = 180, increment = 1, textvariable = self.angle_var, width = 4)
        radian_entry   = Entry(content, state = 'readonly', textvariable = self.radian_var, width = 9)
        distance_entry = Spinbox(content, from_ = -100, to=100, increment = 0.1, textvariable = self.distance_var, width = 7)

        content        .grid(column = 0, row = 0, padx = 10, pady = 10)
        labelframe     .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 5),  columnspan = 5)
        button_stop    .grid(column = 0, row = 0, sticky = (N, W), padx = (10, 0), pady = (10, 10))
        button_start   .grid(column = 1, row = 0, sticky = (N, W), padx = (10, 0), pady = (10, 10))
        button_params  .grid(column = 2, row = 0, sticky = (N, W), padx = (10, 0), pady = (10, 10))
        label_bot_id   .grid(column = 3, row = 0, sticky = (N, W), padx = (60, 10), pady = (15, 10))

        label_angle    .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        angle_entry    .grid(column = 1, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        radian_entry   .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        label_distance .grid(column = 3, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)
        distance_entry .grid(column = 4, row = 2, sticky = (N, W), padx = (5, 0), columnspan = 1)


    def run(self):
        self.mainloop()
