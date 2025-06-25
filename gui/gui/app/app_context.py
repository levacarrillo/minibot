class AppContext:
    def __init__(self, app, color, content, controller):
        self.app          = app
        self.color        = color
        self.content      = content
        self.controller   = controller

        self.canvas       = None
        self.canvas_size  = None
        self.canvas_scale = None
        self.canvas_panel = None

        self.side_frame   = None

        self.grid         = None
        self.light        = None
        self.robot        = None

        self.env_section     = None 
        self.sensors_section = None 
        self.robot_section   = None 
        self.buttons_section = None 

        self.simulation_running = False

    def set_canvas_panel(self, canvas_panel):
        self.canvas_panel = canvas_panel
        self.canvas = canvas_panel.canvas
        self.canvas_size  = canvas_panel.size
        self.canvas_scale = canvas_panel.scale

    def set_canvas_size(self, canvas_size):
        self.canvas_size  = canvas_size

    # SETTERS FOR MAIN FRAMES
    def set_side_frame(self, side_frame):
        self.side_frame = side_frame

    # SETTERS FOR SECTIONS
    def set_env_section(self, env_section):
        self.env_section = env_section

    def set_sensors_section(self, sensors_section):
        self.sensors_section = sensors_section

    def set_robot_section(self, robot_section):
        self.robot_section   = robot_section

    def set_buttons_section(self, buttons_section):
        self.buttons_section = buttons_section

    # SETTERS FOR CANVA'S COMPONENTS
    def set_grid(self, grid):
        self.grid = grid

    def set_light(self, light):
        self.light = light

    def set_robot(self, robot):
        self.robot = robot