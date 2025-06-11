from tkinter import *
from gui.app.controller import Controller

class GUIAdapter:
    def __init__(self, controller = Controller):
        self.root = Tk()
        self.root.title('Mobile Robot Simulator')

        self.canvas_x = 500
        self.canvas_y = 500

        self.content    = Frame(self.root)
        self.frame      = Frame(self.content, borderwidth = 5, relief = "flat", width = 900, height = 900)
        self.w = Canvas(self.frame, width = self.canvas_x, height = self.canvas_y, bg="#FFFFFF")

        # MENU WIDGETS
        self.main_bar       = Menu(self.root)
        self.settings       = Menu(self.main_bar, tearoff=0)
        self.submenu_canvas = Menu(self.settings, tearoff=0)
        self.help           = Menu(self.main_bar, tearoff=0)
        self.exit           = Menu(self.settings, tearoff=0)

        self.submenu_canvas.add_command(label = " 600 x 600 ", command= lambda : self.resize_canvas(600, 600))
        self.submenu_canvas.add_command(label = " 700 x 700 ", command= lambda : self.resize_canvas(700, 700))
        self.submenu_canvas.add_command(label = " 800 x 800 ", command= lambda : self.resize_canvas(800, 800))

        self.help.add_command(label=" User Guide")
        self.help.add_command(label=" ROS 2 Nodes")
        self.help.add_command(label=" Topological Map")

        self.main_bar.add_cascade(label=" Settings",    menu=self.settings)
        self.settings.add_cascade(label=" Canvas size", menu=self.submenu_canvas)
        self.main_bar.add_cascade(label=" Help",        menu=self.help)
        self.settings.add_command(label=" Exit", command = self.root.quit)

        # MAIN FRAME
        self.right_menu = Frame(self.content, borderwidth = 5, relief = "flat", width = 300, height = 900)
        self.root.config(menu=self.main_bar)


        self.w      .pack()
        self.content.pack(fill=BOTH, expand=True)

        self.frame      .grid(column = 0, row = 0, columnspan = 3, rowspan = 2, sticky = (N, S, E, W))
        self.right_menu .grid(column = 3, row = 0, columnspan = 3, rowspan = 2, sticky = (N, S, E, W))

        # BEHAVIOR'S SECTION
        self.label_settings      = Label(self.right_menu, text = "Settings")
        self.label_enviroments   = Label(self.right_menu, text = "Environment:")
        self.label_steps         = Label(self.right_menu, text = "Steps:")
        self.label_behavior		 = Label(self.right_menu, text = "Behavior:")
        self.label_light_x       = Label(self.right_menu, text = "Light X:")
        self.label_light_t       = Label(self.right_menu, text = "Light Y:")
        self.label_stepsExcec    = Label(self.right_menu, text = "Steps:")
        self.label_configuration = Label(self.right_menu, text = "Configurations:")
        
        self.steps_entry = Entry(self.right_menu, validate='key')
        self.enviroment_option = OptionMenu(self.right_menu, StringVar(value="Empty"), *controller.get_enviroments(self))
        self.behavior_option   = OptionMenu(self.right_menu, StringVar(value="NO SELECTED"), *controller.get_behavior_list(self))
        self.label_light_x_var = Label(self.right_menu ,text = "Click Right", justify='center')
        self.label_light_y_var = Label(self.right_menu ,text = "Click Right", justify='center')
        self.label_steps_var   = Label(self.right_menu ,text = "0", justify='center')
        self.ck_button_fast    = Checkbutton(self.right_menu, text="Fast mode")
        self.ck_button_sensors = Checkbutton(self.right_menu, text="Show sensors")
        self.ck_add_noise      = Checkbutton(self.right_menu, text="Add Noise")
        self.ck_button_load    = Checkbutton(self.right_menu, text="Load Objects")

        self.label_settings      .grid(column = 0, row = 0,  sticky = (N, W), padx = (5,5))
        self.label_enviroments   .grid(column = 0, row = 1,  sticky = (N, W), padx = (10,5))
        self.label_steps         .grid(column = 0, row = 2,  sticky = (N, W), padx = (10,5))
        self.label_behavior      .grid(column = 0, row = 3,  sticky = (N, W), padx = (10,5))
        self.label_light_x       .grid(column = 0, row = 4,  sticky = (N, W), padx = (10,5))
        self.label_light_t       .grid(column = 0, row = 5,  sticky = (N, W), padx = (10,5))
        self.label_stepsExcec    .grid(column = 0, row = 6,  sticky = (N, W), padx = (10,5))
        self.label_configuration .grid(column = 0, row = 7,  sticky = (N, W), padx = (10,5))
        self.enviroment_option   .grid(column = 1, row = 1,  sticky = (N, W), padx = (10,5))
        self.steps_entry         .grid(column = 1, row = 2,  sticky = (N, W), padx = (10,5))
        self.behavior_option     .grid(column = 1, row = 3,  sticky = (N, W), padx = (10,5))
        self.label_light_x_var   .grid(column = 1, row = 4,  sticky = (N, W), padx = (10,5))
        self.label_light_y_var   .grid(column = 1, row = 5,  sticky = (N, W), padx = (10,5))
        self.label_steps_var     .grid(column = 1, row = 6,  sticky = (N, W), padx = (10,5))
        self.ck_button_fast      .grid(column = 1, row = 7,  sticky = (N, W), padx = (10,5))
        self.ck_button_sensors   .grid(column = 1, row = 8,  sticky = (N, W), padx = (10,5))
        self.ck_add_noise        .grid(column = 1, row = 9,  sticky = (N, W), padx = (10,5))
        self.ck_button_load      .grid(column = 1, row = 10, sticky = (N, W), padx = (10,5))

        # POSITIONAL SETTINGS
        self.label_robot      = Label(self.right_menu, text = "Robot")
        self.label_pose_x     = Label(self.right_menu, text = "Pose X:")
        self.label_pose_y     = Label(self.right_menu, text = "Pose Y:")
        self.label_angle      = Label(self.right_menu, text = "Angle:")
        self.label_radio      = Label(self.right_menu, text = "Radio:")
        self.label_advance    = Label(self.right_menu, text = "Magnitude Advance:")
        self.label_turn_angle = Label(self.right_menu, text = "Turn Angle:")

        self.entry_pose_x     = Entry(self.right_menu)
        self.entry_pose_y     = Entry(self.right_menu)
        self.entry_angle      = Entry(self.right_menu)
        self.entry_radio      = Entry(self.right_menu)
        self.entry_advance    = Entry(self.right_menu)
        self.entry_turn_angle = Entry(self.right_menu)
        self.button_set_zero  = Button(self.right_menu, width = 8, text = "Angle Zero")

        self.label_robot      .grid(column = 4, row = 0, sticky = (N, W), padx = (5,5))     
        self.label_pose_x     .grid(column = 4, row = 1, sticky = (N, W), padx = (10,5))
        self.label_pose_y     .grid(column = 4, row = 2, sticky = (N, W), padx = (10,5))
        self.label_angle      .grid(column = 4, row = 3, sticky = (N, W), padx = (10,5))
        self.label_radio      .grid(column = 4, row = 5, sticky = (N, W), padx = (10,5))
        self.label_advance    .grid(column = 4, row = 6, sticky = (N, W), padx = (10,5))
        self.label_turn_angle .grid(column = 4, row = 7, sticky = (N, W), padx = (10,5))

        self.entry_pose_x     .grid(column = 5, row = 1, sticky = (N, W), padx = (10,5))
        self.entry_pose_y     .grid(column = 5, row = 2, sticky = (N, W), padx = (10,5))
        self.entry_angle      .grid(column = 5, row = 3, sticky = (N, W), padx = (10,5))
        self.entry_radio      .grid(column = 5, row = 5, sticky = (N, W), padx = (10,5))
        self.entry_advance    .grid(column = 5, row = 6, sticky = (N, W), padx = (10,5))
        self.entry_turn_angle .grid(column = 5, row = 7, sticky = (N, W), padx = (10,5))
        self.button_set_zero  .grid(column = 4, row = 4, columnspan = 2, sticky = (N, W), padx = 5)


        # SENSORS SETTINGS
        self.label_sensors       = Label(self.right_menu, text = "Sensors")
        self.label_num_sensors   = Label(self.right_menu, text = "Num Sensors:")
        self.label_origing_angle = Label(self.right_menu, text = "Origin angle:" )
        self.label_range         = Label(self.right_menu, text = "Range:")
        self.label_value         = Label(self.right_menu, text = "Value:")

        self.entry_num_sensors   = Entry(self.right_menu)
        self.entry_origin_angle  = Entry(self.right_menu)
        self.entry_range         = Entry(self.right_menu)
        self.entry_value         = Entry(self.right_menu)

        self.label_sensors       .grid(column = 0, row = 12, sticky = (N, W), padx = (5,5))     
        self.label_num_sensors   .grid(column = 0, row = 13, sticky = (N, W), padx = (10,5))
        self.label_origing_angle .grid(column = 0, row = 14, sticky = (N, W), padx = (10,5))
        self.label_range         .grid(column = 0, row = 15, sticky = (N, W), padx = (10,5))
        self.label_value         .grid(column = 0, row = 16, sticky = (N, W), padx = (10,5))
        self.entry_num_sensors   .grid(column = 1, row = 13, sticky = (N, W), padx = (9,5))
        self.entry_origin_angle  .grid(column = 1, row = 14, sticky = (N, W), padx = (5,5))
        self.entry_range         .grid(column = 1, row = 15, sticky = (N, W), padx = (5,5))
        self.entry_value         .grid(column = 1, row = 16, sticky = (N, W), padx = (5,5))


        # MAIN BUTTONS
        self.label_velocity  = Label(self.right_menu,  text = "Execution velocity:")
        self.label_simulator = Label (self.right_menu, text = "Simulator")
        self.slider_velocity = Scale(self.right_menu,  from_=1, to=3, orient=HORIZONTAL ,length=150)

        self.button_stop             = Button(self.right_menu, width = 20, text = "Stop")
        self.button_run              = Button(self.right_menu, width = 20, text = "Run simulation")
        self.button_plot_topological = Button(self.right_menu, width = 20, text = "Plot Topological")

        self.label_velocity	         .grid(column = 4, row = 8 , sticky = (N, W), padx = (10,5))
        self.slider_velocity         .grid(column = 4, row = 9 , columnspan = 2, rowspan = 2, sticky = (N, W), padx = 5)
        self.label_simulator         .grid(column = 4, row = 12, sticky = (N, W), padx = (5,5))
        self.button_run              .grid(column = 4, row = 14, sticky = (N, W), padx = (10,5))
        self.button_stop             .grid(column = 4, row = 15, sticky = (N, W), padx = (10,5))
        self.button_plot_topological .grid(column = 4, row = 13, sticky = (N, W), padx = (10,5))

    def run(self):
        self.root.mainloop()

    def resize_canvas(self, x, y):
        self.canvasX = x
        self.canvasY = y
        self.w.configure(width = self.canvas_x, height = self.canvas_y)