from tkinter import Menu, IntVar


class MenuBar:
    def __init__(self, context):
        menu_bar = Menu(context.app)
        settings       = Menu(menu_bar, tearoff = 0)
        help           = Menu(menu_bar, tearoff = 0)

        submenu_canvas = Menu(settings, tearoff = 0)
        exit           = Menu(settings, tearoff = 0)

        submenu_canvas.add_command(label = " 500 x 500 ",
                                    command = lambda : context.resize_canvas(500, 500))
        submenu_canvas.add_command(label = " 600 x 600 ",
                                    command = lambda : context.resize_canvas(600, 600))
        submenu_canvas.add_command(label = " 700 x 700 ",
                                    command = lambda : context.resize_canvas(700, 700))
        submenu_canvas.add_command(label = " 800 x 800 ",
                                    command = lambda : context.resize_canvas(800, 800))

        help.add_command(label = " User Guide")
        help.add_command(label = " ROS 2 Nodes")

        menu_bar.add_cascade(label = " Settings",    menu = settings)
        settings.add_cascade(label = " Canvas size", menu = submenu_canvas)
        menu_bar.add_cascade(label = "Help",         menu = help)
        settings.add_command(label = " Exit", command = context.app.quit)
        context.app.config(menu = menu_bar)
