from tkinter import *


class CmdVelPanel:
    def __init__(self, context):

        self.linear_vel_var  = StringVar(value = '0.20m/s')
        self.angular_vel_var = StringVar(value = '0.30rad/s')

        self.frame = LabelFrame(context.content, text = 'Move robot')

        self.buton_arrow_left  = Button(self.frame, text = '<',  width = 2, command = lambda: context.move_robot('LEFT'))
        self.buton_arrow_right = Button(self.frame, text = '>',  width = 2, command = lambda: context.move_robot('RIGHT'))
        self.buton_arrow_stop  = Button(self.frame, text = '||', width = 2, command = lambda: context.move_robot('STOP'))
        self.buton_arrow_up    = Button(self.frame, text = '^',  width = 2, command = lambda: context.move_robot('FORWARD'))
        self.buton_arrow_down  = Button(self.frame, text = 'v',  width = 2, command = lambda: context.move_robot('BACKWARD'))
        self.label_linear_vel  = Label(self.frame,  text = 'Linear:')
        self.label_angular_vel = Label(self.frame,  text = 'Angular:')
        self.linear_vel_entry  = Spinbox(self.frame, from_ = 0.0, to = 2.0, increment = 0.01, textvariable = self.linear_vel_var,  width = 8)
        self.angular_vel_entry = Spinbox(self.frame, from_ = 0.0, to = 2.0, increment = 0.01, textvariable = self.angular_vel_var, width = 8)

        self.frame             .grid(column = 3, row = 1, sticky = (N, W), padx = (5, 5),  pady = (5, 5),  columnspan = 3)
        self.buton_arrow_left  .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0),  pady = (0, 0),  columnspan = 2)
        self.buton_arrow_right .grid(column = 4, row = 1, sticky = (N, W), padx = (5, 10), pady = (0, 0),  columnspan = 2)
        self.buton_arrow_stop  .grid(column = 2, row = 1, sticky = (N, W), padx = (5, 0), pady = (0, 0),   columnspan = 2)
        self.buton_arrow_up    .grid(column = 2, row = 0, sticky = (N, W), padx = (5, 0), pady = (10, 5),  columnspan = 2)
        self.buton_arrow_down  .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), pady = (5, 10),  columnspan = 2)
        self.label_linear_vel  .grid(column = 0, row = 3, sticky = (N, W), padx = (5, 0), pady = (5, 10),  columnspan = 3)
        self.linear_vel_entry  .grid(column = 3, row = 3, sticky = (N, W), padx = (5, 10), pady = (5, 10), columnspan = 3)
        self.label_angular_vel .grid(column = 0, row = 4, sticky = (N, W), padx = (5, 0), pady = (5, 10),  columnspan = 3)
        self.angular_vel_entry .grid(column = 3, row = 4, sticky = (N, W), padx = (5, 10), pady = (5, 10), columnspan = 3)

        context.set_cmd_vel_panel(self)

        self.linear_vel_var  .trace_add('write', context.format_linear_vel)
        self.angular_vel_var .trace_add('write', context.format_angular_vel)
