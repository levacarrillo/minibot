from tkinter import *


class SensorsSection:
    def __init__(self, context):

        light_threshold = StringVar(value = context.controller.get_param('light_threshold'))
        laser_threshold = StringVar(value = context.controller.get_param('laser_threshold'))

        self.label_sensors       = Label(context.side_frame, text = "Sensors")
        self.label_num_sensors   = Label(context.side_frame, text = "Num Sensors:")
        self.label_origing_angle = Label(context.side_frame, text = "Origin angle:" )
        self.label_range         = Label(context.side_frame, text = "Range:")
        self.lidar_value         = Label(context.side_frame, text = "Lidar value:")
        self.light_value         = Label(context.side_frame, text = "Light value:")

        self.entry_num_sensors   = Entry(context.side_frame, validate = 'key', 
                                        textvariable = StringVar(value = "30"),      width = 10)
        self.entry_origin_angle  = Entry(context.side_frame, validate = 'key',
                                        textvariable = StringVar(value = "-1.5707"), width = 10)
        self.entry_range         = Entry(context.side_frame, validate = 'key',
                                        textvariable = StringVar(value = "3.1416"),  width = 10)
        self.entry_light         = Entry(context.side_frame, validate = 'key', 
                                                textvariable = light_threshold, width = 10)
        self.entry_laser         = Entry(context.side_frame, validate = 'key',
                                                textvariable = laser_threshold, width = 10)

        self.label_sensors       .grid(column = 0, row = 12, sticky = (N, W), padx = (5, 0))     
        self.label_num_sensors   .grid(column = 0, row = 13, sticky = (N, W), padx = (5, 0))
        self.label_origing_angle .grid(column = 0, row = 14, sticky = (N, W), padx = (5, 0))
        self.label_range         .grid(column = 0, row = 15, sticky = (N, W), padx = (5, 0))
        self.lidar_value         .grid(column = 0, row = 16, sticky = (N, W), padx = (5, 0))
        self.light_value         .grid(column = 0, row = 17, sticky = (N, W), padx = (5, 0))
        self.entry_num_sensors   .grid(column = 1, row = 13, sticky = (N, W), padx = (5, 0))
        self.entry_origin_angle  .grid(column = 1, row = 14, sticky = (N, W), padx = (5, 0))
        self.entry_range         .grid(column = 1, row = 15, sticky = (N, W), padx = (5, 0))
        self.entry_laser         .grid(column = 1, row = 16, sticky = (N, W), padx = (5, 0))
        self.entry_light         .grid(column = 1, row = 17, sticky = (N, W), padx = (5, 0))

        context.set_sensors_section(self)
