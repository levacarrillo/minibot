from tkinter import *
from tkinter import ttk


class StatusPanel:
    def __init__(self, context):
        self.progress_bar1_var  = IntVar()
        self.progress_bar2_var  = IntVar()
        self.robot_name_var = StringVar()
        self.battery_percent_1_var = StringVar()
        self.battery_percent_2_var = StringVar()
        self.micro_c_temperature_var = StringVar()
        self.cpu_temperature_var = StringVar()

        frame          = LabelFrame(context.content, text = 'Status')
        label_select   = Label(frame, text = 'Robot:', font = ('arial', 11, 'bold'))
        label_minibot  = Label(frame, textvariable = self.robot_name_var)
        label_temp     = Label(frame, text = 'Temp:', font = ('arial', 11, 'bold'))
        label_mc_temp  = Label(frame, textvariable = self.micro_c_temperature_var)
        label_cpu_temp = Label(frame, textvariable = self.cpu_temperature_var)
        label_battery  = Label(frame, text = 'Battery:', font = ('arial', 11, 'bold'))
        label_percent1 = Label(frame, textvariable = self.battery_percent_1_var)
        label_percent2 = Label(frame, textvariable = self.battery_percent_2_var)
        battery_bar1   = ttk.Progressbar(frame, variable = self.progress_bar1_var, maximum = 100)
        battery_bar2   = ttk.Progressbar(frame, variable = self.progress_bar2_var, maximum = 100)

        frame          .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0),  pady = (5, 5), columnspan = 5)
        label_select   .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0),  pady = (0, 0))
        label_minibot  .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0),  pady = (0, 5))
        label_temp     .grid(column = 3, row = 0, sticky = (N, W), padx = (2, 0),  pady = (0, 0))
        label_mc_temp  .grid(column = 4, row = 0, sticky = (N, W), padx = (2, 0),  pady = (0, 0))
        label_cpu_temp .grid(column = 4, row = 1, sticky = (N, W), padx = (1, 0),  pady = (0, 0))
        label_battery  .grid(column = 5, row = 0, sticky = (N, W), padx = (10, 0), pady = (0, 0))
        label_percent1 .grid(column = 6, row = 0, sticky = (N, W), padx = (5, 0),  pady = (0, 0))
        label_percent2 .grid(column = 6, row = 1, sticky = (N, W), padx = (5, 0),  pady = (0, 0))
        battery_bar1   .grid(column = 7, row = 0, sticky = (N, W), padx = (5, 6),  pady = (0, 0), columnspan = 2)
        battery_bar2   .grid(column = 8, row = 1, sticky = (N, W), padx = (5, 6),  pady = (0, 0), columnspan = 2)

        context.set_status_panel(self)
