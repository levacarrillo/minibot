from tkinter import *
from tkinter import ttk


class StatusPanel:
    def __init__(self, context):
        self.progress_var  = IntVar(value = 100)
        frame         = LabelFrame(context.content, text = 'Status')
        label_bot_id  = Label(frame, text = 'Minibot 2', font = ('arial', 11, 'bold'))
        label_battery = Label(frame, text = f'Battery: {self.progress_var.get()}%')
        battery_bar   = ttk.Progressbar(frame, variable = self.progress_var, maximum = 100)
        button_reset  = Button(frame, text = 'Reset config')

        frame         .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0),  pady = (5, 5), columnspan = 5)
        label_bot_id  .grid(column = 0, row = 0, sticky = (N, W), padx = (10, 5), pady = (15, 10))
        label_battery .grid(column = 1, row = 0, sticky = (N, W), padx = (10, 5), pady = (15, 10), columnspan = 2)
        battery_bar   .grid(column = 4, row = 0, sticky = (N, W), padx = (0, 8),  pady = (18, 10))
        button_reset  .grid(column = 5, row = 0, sticky = (N, W), padx = (0, 10), pady = (10, 10), columnspan = 2)

        context.set_status_panel(self)
