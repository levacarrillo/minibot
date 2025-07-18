from tkinter import *


class RobotSection:
    def __init__(self, context):
        # max_advance    = StringVar(value = context.controller.get_param('max_advance'))
        # max_turn_angle = StringVar(value = context.controller.get_param('max_turn_angle'))
        max_advance    = StringVar()
        max_turn_angle = StringVar()

        label_robot      = Label(context.side_frame, text = "Robot", font = ('arial', 11, 'bold'))
        label_pose_x     = Label(context.side_frame, text = "Pose X:")
        label_pose_y     = Label(context.side_frame, text = "Pose Y:")
        label_angle      = Label(context.side_frame, text = "Angle:")
        label_radius     = Label(context.side_frame, text = "Radius:")
        label_advance    = Label(context.side_frame, text = "Advance:")
        label_turn_angle = Label(context.side_frame, text = "Turn Angle:")
        label_velocity   = Label(context.side_frame,  text = "Execution velocity:")
        slider_velocity  = Scale(context.side_frame,  from_= 1, to=3, orient = HORIZONTAL, 
                                    length = 162, command = context.set_velocity_slider)

        entry_pose_x     = Entry(context.side_frame, validate = 'key', 
                                    textvariable = StringVar(value = "1.5"),  width = 9)
        entry_pose_y     = Entry(context.side_frame, validate = 'key', 
                                    textvariable = StringVar(value = "2.2"),  width = 9)
        entry_angle      = Entry(context.side_frame, validate = 'key',
                                    textvariable = StringVar(value = "1.5707"),  width = 9)
        entry_radius     = Entry(context.side_frame, validate = 'key',
                                    textvariable = StringVar(value = "0.04"), width = 9)
        entry_advance    = Entry(context.side_frame, validate = 'key',
                                    textvariable = max_advance,    width = 9)
        entry_turn_angle = Entry(context.side_frame, validate = 'key',
                                    textvariable = max_turn_angle, width = 9)
        button_set_zero  = Button(context.side_frame, width = 8, text = "Angle Zero",
                                                            command = context.set_angle)

        label_robot      .grid(column = 4, row = 0, sticky = (N, W), padx = (5, 0))     
        label_pose_x     .grid(column = 4, row = 1, sticky = (N, W), padx = (5, 0))
        label_pose_y     .grid(column = 4, row = 2, sticky = (N, W), padx = (5, 0))
        label_angle      .grid(column = 4, row = 3, sticky = (N, W), padx = (5, 0))
        label_radius     .grid(column = 4, row = 5, sticky = (N, W), padx = (5, 0))
        label_advance    .grid(column = 4, row = 6, sticky = (N, W), padx = (5, 0))
        label_turn_angle .grid(column = 4, row = 7, sticky = (N, W), padx = (5, 0))

        entry_pose_x     .grid(column = 4, row = 1, sticky = (N, E), padx = (5, 0))
        entry_pose_y     .grid(column = 4, row = 2, sticky = (N, E), padx = (5, 0))
        entry_angle      .grid(column = 4, row = 3, sticky = (N, E), padx = (5, 0))
        entry_radius     .grid(column = 4, row = 5, sticky = (N, E), padx = (5, 0))
        entry_advance    .grid(column = 4, row = 6, sticky = (N, E), padx = (5, 0))
        entry_turn_angle .grid(column = 4, row = 7, sticky = (N, E), padx = (5, 0))
        button_set_zero  .grid(column = 4, row = 4, sticky = (N, W), padx = (5, 0),
                                    columnspan = 2)
        label_velocity	 .grid(column = 4, row = 9, sticky = (N, W), padx = (5, 0))
        slider_velocity  .grid(column = 4, row = 10, columnspan = 2, rowspan = 1,
                                                    sticky = (N, W), padx = (5, 0), pady = (0, 10))

        entry_angle  .bind("<Return>", context.set_angle)
        entry_radius .bind("<Return>", context.robot_plot())

        context.set_robot_section(self)
