from tkinter import *


class SensorsSection:
    def __init__(self, context):
        # light_threshold = StringVar(value = context.controller.get_param('light_threshold'))
        # laser_threshold = StringVar(value = context.controller.get_param('laser_threshold'))
        light_threshold = StringVar()
        laser_threshold = StringVar()

        label_sensors       = Label(context.side_frame, text = "Sensors", font = ('arial', 11, 'bold'))
        label_num_sensors   = Label(context.side_frame, text = "Num Sensors:")
        label_origing_angle = Label(context.side_frame, text = "Origin angle:" )
        label_range         = Label(context.side_frame, text = "Range:")
        lidar_value         = Label(context.side_frame, text = "Lidar value:")
        light_value         = Label(context.side_frame, text = "Light value:")

        entry_num_sensors   = Entry(context.side_frame, validate = 'key', 
                                        textvariable = StringVar(value = "50"),      width = 10)
        entry_origin_angle  = Entry(context.side_frame, validate = 'key',
                                        textvariable = StringVar(value = "-1.5707"), width = 10)
        entry_range         = Entry(context.side_frame, validate = 'key',
                                        textvariable = StringVar(value = "3.1416"),  width = 10)
        entry_light         = Entry(context.side_frame, validate = 'key', 
                                                textvariable = light_threshold, width = 10)
        entry_laser         = Entry(context.side_frame, validate = 'key',
                                                textvariable = laser_threshold, width = 10)

        label_sensors       .grid(column = 0, row = 12, sticky = (N, W), padx = (5, 0))     
        label_num_sensors   .grid(column = 0, row = 13, sticky = (N, W), padx = (5, 0))
        label_origing_angle .grid(column = 0, row = 14, sticky = (N, W), padx = (5, 0))
        label_range         .grid(column = 0, row = 15, sticky = (N, W), padx = (5, 0))
        lidar_value         .grid(column = 0, row = 16, sticky = (N, W), padx = (5, 0))
        light_value         .grid(column = 0, row = 17, sticky = (N, W), padx = (5, 0))
        entry_num_sensors   .grid(column = 1, row = 13, sticky = (N, W), padx = (5, 0))
        entry_origin_angle  .grid(column = 1, row = 14, sticky = (N, W), padx = (5, 0))
        entry_range         .grid(column = 1, row = 15, sticky = (N, W), padx = (5, 0))
        entry_laser         .grid(column = 1, row = 16, sticky = (N, W), padx = (5, 0))
        entry_light         .grid(column = 1, row = 17, sticky = (N, W), padx = (5, 0))

        entry_num_sensors  .bind("<Return>", context.robot_plot())
        entry_origin_angle .bind("<Return>", context.robot_plot())
        entry_range        .bind("<Return>", context.robot_plot())
        entry_laser        .bind("<Return>", context.robot_plot())

        context.set_sensors_section(self)
