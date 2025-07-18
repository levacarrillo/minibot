from tkinter import *
from tkinter import ttk


class EnvironmentSection:
    def __init__(self, context):
        context = context

        fast_mode    = IntVar()
        add_noise    = IntVar()        
        show_sensors = IntVar()
        load_objects = IntVar()

        # environment_list = context.controller.get_environment_list()
        # behavior_list    = context.controller.get_param('behavior_list')
        # max_steps        = StringVar(value = context.controller.get_param('max_steps'))

        environment_list = []
        behavior_list    = []
        max_steps        = StringVar(value = 35)

        label_settings      = Label(context.side_frame,  text = "Settings", font = ('arial', 11, 'bold'))
        label_enviroments   = Label(context.side_frame,  text = "Environment:")
        label_behavior		 = Label(context.side_frame, text = "Behavior:")
        label_max_steps     = Label(context.side_frame,  text = "Max steps:")
        label_light_x       = Label(context.side_frame,  text = "Light X:")
        label_light_y       = Label(context.side_frame,  text = "Light Y:")
        label_curr_step     = Label(context.side_frame,  text = "Current step:")
        label_configuration = Label(context.side_frame,  text = "Configurations:")
        label_light_pose_x  = Label(context.side_frame,  text = "Click Right", justify='center')
        label_light_pose_y  = Label(context.side_frame,  text = "Click Right",  justify='center')
        label_steps         = Label(context.side_frame,  text = "0", justify='center')
        
        steps_entry = Entry(context.side_frame, validate = 'key', textvariable = max_steps, width = 5)
        environment_cb    = ttk.Combobox(context.side_frame, values = environment_list, width = 16)
        behavior_list_cb  = ttk.Combobox(context.side_frame, values = behavior_list,  width = 16)

        ck_button_fast    = Checkbutton(context.side_frame, text="Fast mode",
                                        variable = fast_mode,
                                        command = lambda: context.on_check_fast_mode(fast_mode.get()))
        ck_button_sensors = Checkbutton(context.side_frame, text="Show sensors",
                                        variable = show_sensors,
                                        command = lambda: on_check_show_sensors(show_sensors.get()))
        ck_add_noise      = Checkbutton(context.side_frame, text="Add Noise", 
                                        variable = add_noise,
                                        command = lambda: context.on_check_noise(add_noise.get()))
        ck_button_load    = Checkbutton(context.side_frame, text="Load Objects",
                                        variable = load_objects,
                                        command = lambda: context.on_check_load_objects(load_objects.get()))

        label_settings      .grid(column = 0, row = 0,  sticky = (N, W), padx = (5, 0))
        label_enviroments   .grid(column = 0, row = 1,  sticky = (N, W), padx = (5, 0))
        label_behavior      .grid(column = 0, row = 2,  sticky = (N, W), padx = (5, 0))
        label_max_steps     .grid(column = 0, row = 3,  sticky = (N, W), padx = (5, 0))
        label_light_x       .grid(column = 0, row = 4,  sticky = (N, W), padx = (5, 0))
        label_light_y       .grid(column = 0, row = 5,  sticky = (N, W), padx = (5, 0))
        label_curr_step     .grid(column = 0, row = 6,  sticky = (N, W), padx = (5, 0))
        label_configuration .grid(column = 0, row = 7,  sticky = (N, W), padx = (5, 0))
        label_light_pose_x  .grid(column = 1, row = 4,  sticky = (N, W), padx = (5, 0))
        label_light_pose_y  .grid(column = 1, row = 5,  sticky = (N, W), padx = (5, 0))
        label_steps         .grid(column = 1, row = 6,  sticky = (N, W), padx = (5, 0))

        steps_entry         .grid(column = 1, row = 3,  sticky = (N, W), padx = (5, 0))
        environment_cb      .grid(column = 1, row = 1,  sticky = (N, W), padx = (5, 0))
        behavior_list_cb    .grid(column = 1, row = 2,  sticky = (N, W), padx = (5, 0))

        ck_button_fast      .grid(column = 1, row = 7,  sticky = (N, W), padx = (5, 0))
        ck_button_sensors   .grid(column = 1, row = 8,  sticky = (N, W), padx = (5, 0))
        ck_add_noise        .grid(column = 1, row = 9,  sticky = (N, W), padx = (5, 0))
        ck_button_load      .grid(column = 1, row = 10, sticky = (N, W), padx = (5, 0))

        # environment_cb.set('EMPTY')
        # behavior_list_cb.current(0)

        environment_cb.bind("<<ComboboxSelected>>", context.plot_map)

        context.set_env_section(self)
