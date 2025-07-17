from tkinter import *


class ParamsPopUp:
    def __init__(self, context):
        self.window = Toplevel(context.app)
        self.window.title('Parameters used for Motion Planner')

        self.max_steps       = IntVar(value = 100)
        self.linear_vel      = StringVar(value = '0.20m/s')
        self.angular_vel     = StringVar(value = '0.30rad/s')
        self.max_advance     = StringVar(value = '0.03m')
        self.max_turn_angle  = StringVar(value = '0.5rad')
        self.light_threshold = StringVar(value = '0.04503')
        self.laser_threshold = StringVar(value = '0.3')


        params_frame          = LabelFrame(self.window,  text = 'Parameters')
        label_linear_vel      = Label(params_frame, text = 'Linear velocity:')
        label_angular_vel     = Label(params_frame, text = 'Angular velocity:')
        label_max_advance     = Label(params_frame, text = 'Max advance:')
        label_max_turn_angle  = Label(params_frame, text = 'Max turn angle:')
        label_light_threshold = Label(params_frame, text = 'Light threshold:')
        label_laser_threhold  = Label(params_frame, text = 'Laser threshold:')
        label_max_steps       = Label(params_frame, text = 'Max steps:')

        sp_linear_vel      = Spinbox(params_frame, textvariable = self.linear_vel,  from_= 0.0,  to = 2.0, increment = 0.01, width = 8)
        sp_angular_vel     = Spinbox(params_frame, textvariable = self.angular_vel, from_= 0.0,  to = 2.0, increment = 0.01, width = 8)
        sp_max_advance     = Spinbox(params_frame, textvariable = self.max_advance, from_= -100, to = 100, increment = 0.5,  width = 7)
        sp_max_steps       = Spinbox(params_frame, textvariable = self.max_steps,   from_= 0,    to = 120, increment = 1,    width = 4)
        sp_light_threshold = Spinbox(params_frame, textvariable = self.light_threshold, from_= 0,    to = 10,  increment = 0.01, width = 8)
        sp_laser_threshold = Spinbox(params_frame, textvariable = self.laser_threshold, from_= 0,    to = 10,  increment = 0.01, width = 7)
        sp_turn_angle      = Spinbox(params_frame, textvariable = self.max_turn_angle, from_=-3.1716, to=3.1416, increment = 0.01745, width = 7)

        button_exit = Button(params_frame, text = 'Close', command = self.window.destroy)
        buton_set   = Button(params_frame, text = 'Set',   command = context.on_click_set_params, width = 4)

        params_frame          .grid(column = 0, row = 0, sticky = (N, W), padx = (10, 10), pady = (10, 10), columnspan = 1)
        label_linear_vel      .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_angular_vel     .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_light_threshold .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_laser_threhold  .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_max_advance     .grid(column = 2, row = 0, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_max_turn_angle  .grid(column = 2, row = 1, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)

        sp_linear_vel         .grid(column = 1, row = 0, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_angular_vel        .grid(column = 1, row = 1, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_light_threshold    .grid(column = 1, row = 2, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_laser_threshold    .grid(column = 3, row = 2, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_max_advance        .grid(column = 3, row = 0, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_turn_angle         .grid(column = 3, row = 1, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)

        label_max_steps       .grid(column = 0, row = 4, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        sp_max_steps          .grid(column = 1, row = 4, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)

        button_exit           .grid(column = 2, row = 4, sticky = (N, E), padx = (0, 0), pady = (15, 10), columnspan = 1)
        buton_set             .grid(column = 3, row = 4, sticky = (N, E), padx = (0, 5), pady = (15, 10), columnspan = 1)

        context.set_params_pop_up(self)
        context.set_initial_window_parameters()
        context.format_parameters()

        self.linear_vel       .trace_add('write', context.format_parameters)
        self.angular_vel      .trace_add('write', context.format_parameters)
        self.max_advance      .trace_add('write', context.format_parameters)
        self.max_turn_angle   .trace_add('write', context.format_parameters)
