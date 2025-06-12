from tkinter import *

class RobotSection:
    def __init__(self, parent_panel):
        side_panel = parent_panel.side_panel
        
        self.label_robot      = Label(side_panel, text = "Robot")
        self.label_pose_x     = Label(side_panel, text = "Pose X:")
        self.label_pose_y     = Label(side_panel, text = "Pose Y:")
        self.label_angle      = Label(side_panel, text = "Angle:")
        self.label_radio      = Label(side_panel, text = "Radio:")
        self.label_advance    = Label(side_panel, text = "Advance:")
        self.label_turn_angle = Label(side_panel, text = "Turn Angle:")

        self.entry_pose_x     = Entry(side_panel, validate = 'key', textvariable = StringVar(value = "1.5"),    width = 9)
        self.entry_pose_y     = Entry(side_panel, validate = 'key', textvariable = StringVar(value = "2.2"),    width = 9)
        self.entry_angle      = Entry(side_panel, validate = 'key', textvariable = StringVar(value = "0.0"),    width = 9)
        self.entry_radio      = Entry(side_panel, validate = 'key', textvariable = StringVar(value = "0.03"),   width = 9)
        self.entry_advance    = Entry(side_panel, validate = 'key', textvariable = StringVar(value = "0.04"),   width = 9)
        self.entry_turn_angle = Entry(side_panel, validate = 'key', textvariable = StringVar(value = "0.7857"), width = 9)
        self.button_set_zero  = Button(side_panel, width = 8, text = "Angle Zero")

        self.label_robot      .grid(column = 4, row = 0, sticky = (N, W), padx = (5, 0))     
        self.label_pose_x     .grid(column = 4, row = 1, sticky = (N, W), padx = (5, 0))
        self.label_pose_y     .grid(column = 4, row = 2, sticky = (N, W), padx = (5, 0))
        self.label_angle      .grid(column = 4, row = 3, sticky = (N, W), padx = (5, 0))
        self.label_radio      .grid(column = 4, row = 5, sticky = (N, W), padx = (5, 0))
        self.label_advance    .grid(column = 4, row = 6, sticky = (N, W), padx = (5, 0))
        self.label_turn_angle .grid(column = 4, row = 7, sticky = (N, W), padx = (5, 0))

        self.entry_pose_x     .grid(column = 4, row = 1, sticky = (N, E), padx = (5, 0))
        self.entry_pose_y     .grid(column = 4, row = 2, sticky = (N, E), padx = (5, 0))
        self.entry_angle      .grid(column = 4, row = 3, sticky = (N, E), padx = (5, 0))
        self.entry_radio      .grid(column = 4, row = 5, sticky = (N, E), padx = (5, 0))
        self.entry_advance    .grid(column = 4, row = 6, sticky = (N, E), padx = (5, 0))
        self.entry_turn_angle .grid(column = 4, row = 7, sticky = (N, E), padx = (5, 0))
        self.button_set_zero  .grid(column = 4, row = 4, columnspan = 2, sticky = (N, W), padx = 5)