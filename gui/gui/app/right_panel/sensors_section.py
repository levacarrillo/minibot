from tkinter import *

class SensorsSection:
    def __init__(self, right_menu, controller):
        self.label_sensors       = Label(right_menu, text = "Sensors")
        self.label_num_sensors   = Label(right_menu, text = "Num Sensors:")
        self.label_origing_angle = Label(right_menu, text = "Origin angle:" )
        self.label_range         = Label(right_menu, text = "Range:")
        self.label_value         = Label(right_menu, text = "Threshold value:")

        self.entry_num_sensors   = Entry(right_menu, validate = 'key', textvariable = StringVar(value="20"),      width = 10)
        self.entry_origin_angle  = Entry(right_menu, validate = 'key', textvariable = StringVar(value="-1.5707"), width = 10)
        self.entry_range         = Entry(right_menu, validate = 'key', textvariable = StringVar(value="3.1416"),  width = 10)
        self.entry_threshold     = Entry(right_menu, validate = 'key', textvariable = StringVar(value="0.5"),     width = 10)

        self.label_sensors       .grid(column = 0, row = 12, sticky = (N, W), padx = (5, 0))     
        self.label_num_sensors   .grid(column = 0, row = 13, sticky = (N, W), padx = (5, 0))
        self.label_origing_angle .grid(column = 0, row = 14, sticky = (N, W), padx = (5, 0))
        self.label_range         .grid(column = 0, row = 15, sticky = (N, W), padx = (5, 0))
        self.label_value         .grid(column = 0, row = 16, sticky = (N, W), padx = (5, 0))
        self.entry_num_sensors   .grid(column = 1, row = 13, sticky = (N, W), padx = (5, 0))
        self.entry_origin_angle  .grid(column = 1, row = 14, sticky = (N, W), padx = (5, 0))
        self.entry_range         .grid(column = 1, row = 15, sticky = (N, W), padx = (5, 0))
        self.entry_threshold     .grid(column = 1, row = 16, sticky = (N, W), padx = (5, 0))