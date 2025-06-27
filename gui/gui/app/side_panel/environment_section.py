from tkinter import *
from tkinter import ttk


class EnvironmentSection:
    def __init__(self, context):
        self.context = context

        self.check_fast_mode  = IntVar()
        environment_list = context.controller.get_environment_list()
        behavior_list    = context.controller.get_param('behavior_list')
        max_steps        = StringVar(value = context.controller.get_param('max_steps'))

        self.label_settings      = Label(context.side_frame, text = "Settings")
        self.label_enviroments   = Label(context.side_frame, text = "Environment:")
        self.label_behavior		 = Label(context.side_frame, text = "Behavior:")
        self.label_max_steps     = Label(context.side_frame, text = "Max steps:")
        self.label_light_x       = Label(context.side_frame, text = "Light X:")
        self.label_light_y       = Label(context.side_frame, text = "Light Y:")
        self.label_curr_step     = Label(context.side_frame, text = "Current step:")
        self.label_configuration = Label(context.side_frame, text = "Configurations:")
        self.label_light_pose_x  = Label(context.side_frame ,text = "Click Right", justify='center')
        self.label_light_pose_y  = Label(context.side_frame ,text = "Click Right",  justify='center')
        self.label_steps_var     = Label(context.side_frame ,text = "0", justify='center')
        
        self.steps_entry = Entry(context.side_frame, validate = 'key', textvariable = max_steps, width = 5)
        self.environment_cb    = ttk.Combobox(context.side_frame,  values = environment_list, width = 16)
        self.behavior_list_cb  = ttk.Combobox(context.side_frame, values = behavior_list,  width = 16)

        self.ck_button_fast    = Checkbutton(context.side_frame, text="Fast mode",
                                                variable = self.check_fast_mode, 
                                                command = self.on_change_fast_mode)
        self.ck_button_sensors = Checkbutton(context.side_frame, text="Show sensors")
        self.ck_add_noise      = Checkbutton(context.side_frame, text="Add Noise")
        self.ck_button_load    = Checkbutton(context.side_frame, text="Load Objects")

        self.label_settings      .grid(column = 0, row = 0,  sticky = (N, W), padx = (5, 0))
        self.label_enviroments   .grid(column = 0, row = 1,  sticky = (N, W), padx = (5, 0))
        self.label_behavior      .grid(column = 0, row = 2,  sticky = (N, W), padx = (5, 0))
        self.label_max_steps     .grid(column = 0, row = 3,  sticky = (N, W), padx = (5, 0))
        self.label_light_x       .grid(column = 0, row = 4,  sticky = (N, W), padx = (5, 0))
        self.label_light_y       .grid(column = 0, row = 5,  sticky = (N, W), padx = (5, 0))
        self.label_curr_step     .grid(column = 0, row = 6,  sticky = (N, W), padx = (5, 0))
        self.label_configuration .grid(column = 0, row = 7,  sticky = (N, W), padx = (5, 0))
        self.label_light_pose_x  .grid(column = 1, row = 4,  sticky = (N, W), padx = (5, 0))
        self.label_light_pose_y  .grid(column = 1, row = 5,  sticky = (N, W), padx = (5, 0))
        self.label_steps_var     .grid(column = 1, row = 6,  sticky = (N, W), padx = (5, 0))

        self.steps_entry         .grid(column = 1, row = 3,  sticky = (N, W), padx = (5, 0))
        self.environment_cb      .grid(column = 1, row = 1,  sticky = (N, W), padx = (5, 0))
        self.behavior_list_cb    .grid(column = 1, row = 2,  sticky = (N, W), padx = (5, 0))

        self.ck_button_fast      .grid(column = 1, row = 7,  sticky = (N, W), padx = (5, 0))
        self.ck_button_sensors   .grid(column = 1, row = 8,  sticky = (N, W), padx = (5, 0))
        self.ck_add_noise        .grid(column = 1, row = 9,  sticky = (N, W), padx = (5, 0))
        self.ck_button_load      .grid(column = 1, row = 10, sticky = (N, W), padx = (5, 0))

        self.environment_cb.current(0)
        self.behavior_list_cb.current(0)

        context.set_env_section(self)

    def on_change_fast_mode(self):
        self.context.set_fast_mode(self.check_fast_mode.get())
