from tkinter import *


class CmdVelPanel:
    def __init__(self, context):

        self.linear_vel_var  = StringVar(value = '0.20m/s')
        self.angular_vel_var = StringVar(value = '0.30rad/s')

        frame = LabelFrame(context.content, text = 'Move robot')

        label_linear_vel  = Label(  frame, text = 'Linear:')
        label_angular_vel = Label(  frame, text = 'Angular:')
        buton_arrow_left  = Button( frame, text = '<',  width = 2, command = lambda: context.move_robot('LEFT'))
        buton_arrow_right = Button( frame, text = '>',  width = 2, command = lambda: context.move_robot('RIGHT'))
        buton_arrow_stop  = Button( frame, text = '||', width = 2, command = lambda: context.move_robot('STOP'))
        buton_arrow_up    = Button( frame, text = '^',  width = 2, command = lambda: context.move_robot('FORWARD'))
        buton_arrow_down  = Button( frame, text = 'v',  width = 2, command = lambda: context.move_robot('BACKWARD'))
        linear_vel_entry  = Spinbox(frame, from_ = 0.0, to = 2.0, increment = 0.01, textvariable = self.linear_vel_var,  width = 8)
        angular_vel_entry = Spinbox(frame, from_ = 0.0, to = 2.0, increment = 0.01, textvariable = self.angular_vel_var, width = 8)

        frame             .grid(column = 3, row = 1, sticky = (N, W), padx = (5, 5),  pady = (5, 5),  columnspan = 3)
        buton_arrow_left  .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0),  pady = (0, 0),  columnspan = 2)
        buton_arrow_right .grid(column = 4, row = 1, sticky = (N, W), padx = (5, 10), pady = (0, 0),  columnspan = 2)
        buton_arrow_stop  .grid(column = 2, row = 1, sticky = (N, W), padx = (5, 0), pady = (0, 0),   columnspan = 2)
        buton_arrow_up    .grid(column = 2, row = 0, sticky = (N, W), padx = (5, 0), pady = (10, 5),  columnspan = 2)
        buton_arrow_down  .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10),  columnspan = 2)
        label_linear_vel  .grid(column = 0, row = 3, sticky = (N, W), padx = (5, 0), pady = (5, 10),  columnspan = 3)
        linear_vel_entry  .grid(column = 3, row = 3, sticky = (N, W), padx = (5, 10), pady = (5, 10), columnspan = 3)
        label_angular_vel .grid(column = 0, row = 4, sticky = (N, W), padx = (5, 0), pady = (5, 10),  columnspan = 3)
        angular_vel_entry .grid(column = 3, row = 4, sticky = (N, W), padx = (5, 10), pady = (5, 10), columnspan = 3)

        context.set_cmd_vel_panel(self)

        self.linear_vel_var  .trace_add('write', context.format_linear_vel)
        self.angular_vel_var .trace_add('write', context.format_angular_vel)
