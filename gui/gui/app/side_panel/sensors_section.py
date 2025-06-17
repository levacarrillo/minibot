from tkinter import *

class SensorsSection:
    def __init__(self, parent_panel):
        side = parent_panel.side_panel
        controller = parent_panel.controller

        self.label_sensors       = Label(side, text = "Sensors")
        self.label_num_sensors   = Label(side, text = "Num Sensors:")
        self.label_origing_angle = Label(side, text = "Origin angle:" )
        self.label_range         = Label(side, text = "Range:")
        self.label_value         = Label(side, text = "Threshold value:")

        self.entry_num_sensors   = Entry(side, validate = 'key', 
                                        textvariable = StringVar(value = "20"),      width = 10)
        self.entry_origin_angle  = Entry(side, validate = 'key',
                                        textvariable = StringVar(value = "-1.5707"), width = 10)
        self.entry_range         = Entry(side, validate = 'key',
                                        textvariable = StringVar(value = "3.1416"),  width = 10)
        self.entry_threshold     = Entry(side, validate = 'key',
                                    textvariable = StringVar(value = controller.get_lidar_threshold()),
                                    width = 10)

        self.label_sensors       .grid(column = 0, row = 12, sticky = (N, W), padx = (5, 0))     
        self.label_num_sensors   .grid(column = 0, row = 13, sticky = (N, W), padx = (5, 0))
        self.label_origing_angle .grid(column = 0, row = 14, sticky = (N, W), padx = (5, 0))
        self.label_range         .grid(column = 0, row = 15, sticky = (N, W), padx = (5, 0))
        self.label_value         .grid(column = 0, row = 16, sticky = (N, W), padx = (5, 0))
        self.entry_num_sensors   .grid(column = 1, row = 13, sticky = (N, W), padx = (5, 0))
        self.entry_origin_angle  .grid(column = 1, row = 14, sticky = (N, W), padx = (5, 0))
        self.entry_range         .grid(column = 1, row = 15, sticky = (N, W), padx = (5, 0))
        self.entry_threshold     .grid(column = 1, row = 16, sticky = (N, W), padx = (5, 0))