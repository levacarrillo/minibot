from tkinter import *
from gui.app.components.canvas.route import Route
from gui.app.components.canvas.light import Light
from gui.app.components.canvas.objects import Objects
from gui.app.components.canvas.robot.robot import Robot


class CanvasPanel:
    def __init__(self, context):
        frame = Frame(context.content, borderwidth = 5, relief = "flat", width = 900, height = 900, 
                                                                    bg = context.color['background'])

        size = context.get_canvas_size()
        canvas = Canvas(frame, width = size['width'], height = size['height'],
                                                                    bg = context.color['canvas'])


        context.set_canvas(canvas)

        light = Light(context)
        robot = Robot(context)
        Route(context)
        Objects(context)

        canvas.bind("<Button-3>", context.set_light_position)
        canvas.bind("<Button-1>", context.set_robot_position)
        context.plot_map()

        frame.grid(column = 0, row = 0, columnspan = 3, rowspan = 2, sticky = (N, S, E, W))                          
        canvas.pack()
