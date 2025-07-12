from tkinter import *


class CmdPosePanel:
    def __init__(self, context):

        self.radian_var   = StringVar()
        self.angle_var    = StringVar(value = '0°')
        self.distance_var = StringVar(value = '0.0cm')


        self.frame          = LabelFrame(context.content, text = 'Move to pose')
        self.label_angle    = Label(self.frame,   text  = 'Angle:')
        self.label_distance = Label(self.frame,   text  = 'Distance:')
        self.radian_entry   = Entry(self.frame,   state = 'readonly', textvariable  = self.radian_var, width = 9)
        self.angle_entry    = Spinbox(self.frame, from_ = -180, to = 180, increment = 5,   textvariable = self.angle_var,    width = 4)
        self.distance_entry = Spinbox(self.frame, from_ = -100, to = 100, increment = 0.5, textvariable = self.distance_var, width = 7)
        self.button_start   = Button(self.frame,  text  = 'Start')

        self.frame           .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 5), pady = (5, 5),  columnspan = 5)
        self.label_angle     .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        self.angle_entry     .grid(column = 1, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        self.radian_entry    .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        self.label_distance  .grid(column = 3, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        self.distance_entry  .grid(column = 4, row = 2, sticky = (N, W), padx = (5, 5), pady = (5, 10), columnspan = 1)
        self.button_start    .grid(column = 5, row = 2, sticky = (N, W), padx = (5, 5), pady = (0, 10))

        context.set_cmd_pose(self)

        self.angle_var     .trace_add('write', context.format_angle)
        self.distance_var  .trace_add('write', context.format_distance)
        self.button_start  .config(command   = context.on_click_start)
