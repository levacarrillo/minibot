from tkinter import Menu

class MenuBar:
    def __init__(self, app):
        self.canvas = app.canvas_panel

        menu_bar = Menu(app)
        settings       = Menu(menu_bar, tearoff = 0)
        help           = Menu(menu_bar, tearoff = 0)

        submenu_canvas = Menu(settings, tearoff = 0)
        exit           = Menu(settings, tearoff = 0)

        submenu_canvas.add_command(label = " 600 x 600 ",
                                            command = lambda : self.resize_canvas(600, 600))
        submenu_canvas.add_command(label = " 700 x 700 ",
                                            command = lambda : self.resize_canvas(700, 700))
        submenu_canvas.add_command(label = " 800 x 800 ",
                                            command = lambda : self.resize_canvas(800, 800))

        help.add_command(label = " User Guide")
        help.add_command(label = " ROS 2 Nodes")

        menu_bar.add_cascade(label = " Settings",    menu = settings)
        settings.add_cascade(label = " Canvas size", menu = submenu_canvas)
        menu_bar.add_cascade(label = "Help",         menu = help)
        settings.add_command(label = " Plot topological")
        settings.add_command(label = " Exit", command     = app.quit)
        app.config(menu=menu_bar)

    def resize_canvas(self, size_x, size_y):
        self.canvas.size = { 'x': size_x, 'y': size_y }
        self.canvas.canvas.configure(width = size_x, height = size_y)
        self.canvas.print_grid()
        self.canvas.light.plot()
        self.canvas.robot.plot()
