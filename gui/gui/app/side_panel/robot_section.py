from tkinter import *


class RobotSection:
    def __init__(self, context):
        self.context = context

        max_advance    = StringVar(value = context.controller.get_param('max_advance'))
        max_turn_angle = StringVar(value = context.controller.get_param('max_turn_angle'))
        
        self.label_robot      = Label(context.side_frame, text = "Robot")
        self.label_pose_x     = Label(context.side_frame, text = "Pose X:")
        self.label_pose_y     = Label(context.side_frame, text = "Pose Y:")
        self.label_angle      = Label(context.side_frame, text = "Angle:")
        self.label_radius     = Label(context.side_frame, text = "Radius:")
        self.label_advance    = Label(context.side_frame, text = "Advance:")
        self.label_turn_angle = Label(context.side_frame, text = "Turn Angle:")

        self.entry_pose_x     = Entry(context.side_frame, validate = 'key', 
                                    textvariable = StringVar(value = "1.5"),  width = 9)
        self.entry_pose_y     = Entry(context.side_frame, validate = 'key', 
                                    textvariable = StringVar(value = "2.2"),  width = 9)
        self.entry_angle      = Entry(context.side_frame, validate = 'key',
                                    textvariable = StringVar(value = "0.0"),  width = 9)
        self.entry_radius     = Entry(context.side_frame, validate = 'key',
                                    textvariable = StringVar(value = "0.04"), width = 9)
        self.entry_advance    = Entry(context.side_frame, validate = 'key',
                                    textvariable = max_advance,    width = 9)
        self.entry_turn_angle = Entry(context.side_frame, validate = 'key',
                                    textvariable = max_turn_angle, width = 9)
        self.button_set_zero  = Button(context.side_frame, width = 8, text = "Angle Zero",
                                                            command = self.set_angle_zero)

        self.label_robot      .grid(column = 4, row = 0, sticky = (N, W), padx = (5, 0))     
        self.label_pose_x     .grid(column = 4, row = 1, sticky = (N, W), padx = (5, 0))
        self.label_pose_y     .grid(column = 4, row = 2, sticky = (N, W), padx = (5, 0))
        self.label_angle      .grid(column = 4, row = 3, sticky = (N, W), padx = (5, 0))
        self.label_radius     .grid(column = 4, row = 5, sticky = (N, W), padx = (5, 0))
        self.label_advance    .grid(column = 4, row = 6, sticky = (N, W), padx = (5, 0))
        self.label_turn_angle .grid(column = 4, row = 7, sticky = (N, W), padx = (5, 0))

        self.entry_pose_x     .grid(column = 4, row = 1, sticky = (N, E), padx = (5, 0))
        self.entry_pose_y     .grid(column = 4, row = 2, sticky = (N, E), padx = (5, 0))
        self.entry_angle      .grid(column = 4, row = 3, sticky = (N, E), padx = (5, 0))
        self.entry_radius     .grid(column = 4, row = 5, sticky = (N, E), padx = (5, 0))
        self.entry_advance    .grid(column = 4, row = 6, sticky = (N, E), padx = (5, 0))
        self.entry_turn_angle .grid(column = 4, row = 7, sticky = (N, E), padx = (5, 0))
        self.button_set_zero  .grid(column = 4, row = 4, sticky = (N, W), padx = (5, 0),
                                    columnspan = 2)

        context.set_robot_section(self)

    def set_angle_zero(self):
        self.context.panel_update_value('entry_angle', 0.0)
        self.context.robot.plot()