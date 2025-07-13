from tkinter import *

class ParamsPopUp:
    def __init__(self, context):
        window = Toplevel(context.app)
        window.title('Parameters used for Motion Planner')

        params_frame            = LabelFrame(window,  text = 'Parameters')
        label_linear_vel        = Label(params_frame, text = 'Linear velocity:')
        label_angular_vel       = Label(params_frame, text = 'Angular velocity:')
        label_max_advance       = Label(params_frame, text = 'Max advance:')
        label_max_turn_angle    = Label(params_frame, text = 'Max turn angle:')
        label_light_threshold   = Label(params_frame, text = 'Light threshold:')
        label_laser_threhold    = Label(params_frame, text = 'Laser threshold:')

        sp_linear_vel      = Spinbox(params_frame, from_= 0.0,  to = 2.0, increment = 0.01, width = 8)
        sp_angular_vel     = Spinbox(params_frame, from_= 0.0,  to = 2.0, increment = 0.01, width = 8)
        sp_light_threshold = Spinbox(params_frame, from_= 0,    to = 10,  increment = 0.01, width = 8)
        sp_max_advance     = Spinbox(params_frame, from_= -100, to = 100, increment = 0.5,  width = 7)
        sp_turn_angle      = Spinbox(params_frame, from_= -180, to = 180, increment = 1,    width = 7)
        sp_laser_threshold = Spinbox(params_frame, from_= 0,    to = 10,  increment = 0.01, width = 7)

        button_exit = Button(params_frame, text = 'Close', command = window.destroy)
        buton_set   = Button(params_frame, text = 'Set',   command = window.destroy, width = 4)

        params_frame            .grid(column = 0, row = 0, sticky = (N, W), padx = (10, 10), pady = (10, 10), columnspan = 1)
        label_linear_vel        .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_angular_vel       .grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_light_threshold   .grid(column = 0, row = 2, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_laser_threhold    .grid(column = 2, row = 2, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_max_advance       .grid(column = 2, row = 0, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)
        label_max_turn_angle    .grid(column = 2, row = 1, sticky = (N, W), padx = (5, 0), pady = (15, 0), columnspan = 1)

        sp_linear_vel           .grid(column = 1, row = 0, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_angular_vel          .grid(column = 1, row = 1, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_light_threshold      .grid(column = 1, row = 2, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_laser_threshold      .grid(column = 3, row = 2, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_max_advance          .grid(column = 3, row = 0, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)
        sp_turn_angle           .grid(column = 3, row = 1, sticky = (N, W), padx = (5, 5), pady = (15, 0), columnspan = 1)

        button_exit             .grid(column = 2, row = 4, sticky = (N, E), padx = (0, 0), pady = (15, 10), columnspan = 1)
        buton_set               .grid(column = 3, row = 4, sticky = (N, E), padx = (0, 5), pady = (15, 10), columnspan = 1)