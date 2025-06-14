from tkinter import *
from tkinter import ttk

class EnvironmentSection:
    def __init__(self, parent_panel):
        side       = parent_panel.side_panel
        controller = parent_panel.controller

        self.label_settings      = Label(side, text = "Settings")
        self.label_enviroments   = Label(side, text = "Environment:")
        self.label_behavior		 = Label(side, text = "Behavior:")
        self.label_max_steps     = Label(side, text = "Max steps:")
        self.label_light_x       = Label(side, text = "Light X:")
        self.label_light_y       = Label(side, text = "Light Y:")
        self.label_curr_step     = Label(side, text = "Current step:")
        self.label_configuration = Label(side, text = "Configurations:")
        self.label_light_pose_x  = Label(side ,text = "Click Right", justify='center')
        self.label_light_pose_y  = Label(side ,text = "Click Right",  justify='center')
        self.label_steps_var     = Label(side ,text = "0", justify='center')
        
        self.steps_entry = Entry(side, validate = 'key', textvariable = StringVar(value="100"),
                                        width = 5)
        self.env_cb = ttk.Combobox(side,  values = controller.get_environment_list(), width = 16)
        self.bh_cb   = ttk.Combobox(side, values = controller.get_behavior_list(),    width = 16)

        self.ck_button_fast    = Checkbutton(side, text="Fast mode")
        self.ck_button_sensors = Checkbutton(side, text="Show sensors")
        self.ck_add_noise      = Checkbutton(side, text="Add Noise")
        self.ck_button_load    = Checkbutton(side, text="Load Objects")

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
        self.env_cb              .grid(column = 1, row = 1,  sticky = (N, W), padx = (5, 0))
        self.bh_cb               .grid(column = 1, row = 2,  sticky = (N, W), padx = (5, 0))

        self.ck_button_fast      .grid(column = 1, row = 7,  sticky = (N, W), padx = (5, 0))
        self.ck_button_sensors   .grid(column = 1, row = 8,  sticky = (N, W), padx = (5, 0))
        self.ck_add_noise        .grid(column = 1, row = 9,  sticky = (N, W), padx = (5, 0))
        self.ck_button_load      .grid(column = 1, row = 10, sticky = (N, W), padx = (5, 0))

        self.env_cb.current(0)
        self.bh_cb.current(0)