class AppContext:
    def __init__(self, app, color, content, controller):
        self.app          = app
        self.color        = color
        self.content      = content
        self.controller   = controller

        self.canvas       = None
        self.canvas_size  = None
        self.canvas_scale = None
        self.grid         = None
        self.light        = None

        self.side_panel   = None
        self.canvas_panel = None

        self.side_frame      = None
        self.env_section     = None 
        self.sensors_section = None 
        self.robot_section   = None 
        self.buttons_section = None 

    def set_side_frame(self, side_frame):
        self.side_frame = side_frame
    
    def set_side_panel(self, side_panel):
        self.env_section     = side_panel.env_section
        self.sensors_section = side_panel.sensors_section
        self.robot_section   = side_panel.robot_section
        self.buttons_section = side_panel.buttons_section

    def set_canvas_panel(self, canvas_panel):
        self.canvas = canvas_panel.canvas
        self.canvas_panel = canvas_panel
        self.canvas_size  = canvas_panel.size
        self.canvas_scale = canvas_panel.scale
    
    def set_grid(self, grid):
        self.grid = grid

    def set_light(self, light):
        self.light = light
