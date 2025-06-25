from tkinter import *

class RobotSection:
    def __init__(self, context):
        side = context.side_frame
        controller = context.controller
        
        self.label_robot      = Label(side, text = "Robot")
        self.label_pose_x     = Label(side, text = "Pose X:")
        self.label_pose_y     = Label(side, text = "Pose Y:")
        self.label_angle      = Label(side, text = "Angle:")
        self.label_radius     = Label(side, text = "Radius:")
        self.label_advance    = Label(side, text = "Advance:")
        self.label_turn_angle = Label(side, text = "Turn Angle:")

        self.entry_pose_x     = Entry(side, validate = 'key', 
                                        textvariable = StringVar(value = "1.5"),    width = 9)
        self.entry_pose_y     = Entry(side, validate = 'key', 
                                        textvariable = StringVar(value = "2.2"),    width = 9)
        self.entry_angle      = Entry(side, validate = 'key',
                                        textvariable = StringVar(value = "0.0"),    width = 9)
        self.entry_radius      = Entry(side, validate = 'key',
                                        textvariable = StringVar(value = "0.04"),   width = 9)
        self.entry_advance    = Entry(side, validate = 'key',
                                    textvariable = StringVar(value = controller.get_param('max_advance')),
                                    width = 9)
        self.entry_turn_angle = Entry(side, validate = 'key',
                                    textvariable = StringVar(value = controller.get_param('max_turn_angle')),
                                    width = 9)
        self.button_set_zero  = Button(side, width = 8, text = "Angle Zero")

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
        self.entry_radius      .grid(column = 4, row = 5, sticky = (N, E), padx = (5, 0))
        self.entry_advance    .grid(column = 4, row = 6, sticky = (N, E), padx = (5, 0))
        self.entry_turn_angle .grid(column = 4, row = 7, sticky = (N, E), padx = (5, 0))
        self.button_set_zero  .grid(column = 4, row = 4, sticky = (N, W), padx = (5, 0),
                                    columnspan = 2)
        context.set_robot_section(self)