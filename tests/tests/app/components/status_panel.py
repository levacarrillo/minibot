from tkinter import *
from tkinter import ttk


class StatusPanel:
    def __init__(self, context):
        self.progress_var  = IntVar()
        self.robot_name_var = StringVar()
        self.battery_percentage_var = StringVar()

        frame         = LabelFrame(context.content, text = 'Status')
        label_select  = Label(frame, text = 'Robot:', font = ('arial', 11, 'bold'))
        label_minibot = Label(frame, textvariable = self.robot_name_var)
        label_battery = Label(frame, textvariable = self.battery_percentage_var)
        battery_bar   = ttk.Progressbar(frame, variable = self.progress_var, maximum = 100)

        frame         .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0),  pady = (5, 5),   columnspan = 5)
        label_select  .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0),  pady = (10, 10))
        label_minibot .grid(column = 1, row = 0, sticky = (N, W), padx = (0, 0),  pady = (9, 10))
        label_battery .grid(column = 2, row = 0, sticky = (N, W), padx = (80, 5), pady = (9, 10),  columnspan = 2)
        battery_bar   .grid(column = 4, row = 0, sticky = (N, W), padx = (0, 10), pady = (10, 10), columnspan = 2)

        context.set_status_panel(self)
