from tkinter import *

class RobotSection:
    def __init__(self, right_menu, controller):
        self.label_robot      = Label(right_menu, text = "Robot")
        self.label_pose_x     = Label(right_menu, text = "Pose X:")
        self.label_pose_y     = Label(right_menu, text = "Pose Y:")
        self.label_angle      = Label(right_menu, text = "Angle:")
        self.label_radio      = Label(right_menu, text = "Radio:")
        self.label_advance    = Label(right_menu, text = "Advance:")
        self.label_turn_angle = Label(right_menu, text = "Turn Angle:")

        self.entry_pose_x     = Entry(right_menu, validate = 'key', textvariable = StringVar(value = "1.5"),    width = 9)
        self.entry_pose_y     = Entry(right_menu, validate = 'key', textvariable = StringVar(value = "2.2"),    width = 9)
        self.entry_angle      = Entry(right_menu, validate = 'key', textvariable = StringVar(value = "0.0"),    width = 9)
        self.entry_radio      = Entry(right_menu, validate = 'key', textvariable = StringVar(value = "0.03"),   width = 9)
        self.entry_advance    = Entry(right_menu, validate = 'key', textvariable = StringVar(value = "0.04"),   width = 9)
        self.entry_turn_angle = Entry(right_menu, validate = 'key', textvariable = StringVar(value = "0.7857"), width = 9)
        self.button_set_zero  = Button(right_menu, width = 8, text = "Angle Zero")

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