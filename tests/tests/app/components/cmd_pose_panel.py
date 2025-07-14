from tkinter import *


class CmdPosePanel:
    def __init__(self, context):

        self.radian_var   = StringVar()
        self.angle_var    = StringVar(value = '0°')
        self.distance_var = StringVar(value = '0.0cm')


        frame          = LabelFrame(context.content, text = 'Move to pose')
        label_angle    = Label(frame,   text  = 'Angle:')
        label_distance = Label(frame,   text  = 'Distance:')
        radian_entry   = Entry(frame,   state = 'readonly', textvariable  = self.radian_var, width = 9)
        angle_entry    = Spinbox(frame, from_ = -180, to = 180, increment = 5,   textvariable = self.angle_var,    width = 4)
        distance_entry = Spinbox(frame, from_ = -100, to = 100, increment = 0.5, textvariable = self.distance_var, width = 7)
        button_start   = Button(frame,  text  = 'Start')

        frame           .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 5), pady = (5, 5),  columnspan = 5)
        label_angle     .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        angle_entry     .grid(column = 1, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        radian_entry    .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        label_distance  .grid(column = 3, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        distance_entry  .grid(column = 4, row = 2, sticky = (N, W), padx = (5, 5), pady = (5, 10), columnspan = 1)
        button_start    .grid(column = 5, row = 2, sticky = (N, W), padx = (5, 5), pady = (0, 10))

        context.set_cmd_pose_panel(self)

        button_start       .config(command   = context.on_click_start)
        self.angle_var     .trace_add('write', context.format_angle)
        self.distance_var  .trace_add('write', context.format_distance)
