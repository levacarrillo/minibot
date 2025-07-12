from tkinter import *
from tkinter import ttk


class StatusPanel:
    def __init__(self, context):
        self.progress_var  = IntVar()
        self.frame         = LabelFrame(context.content, text = 'Status')
        self.label_bot_id  = Label(self.frame, text = 'Minibot 2', font = ('arial', 11, 'bold'))
        self.label_battery = Label(self.frame, text = 'Battery: 80%')
        self.battery_bar   = ttk.Progressbar(self.frame, variable = self.progress_var, maximum = 100)
        self.button_reset  = Button(self.frame, text = 'Reset config')

        self.battery_bar   .step(80)
        self.frame         .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0),   pady = (5, 5), columnspan = 5)
        self.label_bot_id  .grid(column = 0, row = 0, sticky = (N, W), padx = (10, 5), pady = (15, 10))
        self.label_battery .grid(column = 1, row = 0, sticky = (N, W), padx = (10, 5),  pady = (15, 10), columnspan = 2)
        self.battery_bar   .grid(column = 4, row = 0, sticky = (N, W), padx = (0, 8),  pady = (18, 10))
        self.button_reset  .grid(column = 5, row = 0, sticky = (N, W), padx = (0, 10),  pady = (10, 10), columnspan = 2)