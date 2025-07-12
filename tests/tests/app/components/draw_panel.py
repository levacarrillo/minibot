from tkinter import *


class DrawPanel:
    def __init__(self, context):
        self.width  = 220
        self.height = 220
        self.canvas = Canvas(context.content, width = self.width, height = self.height)

        head_cte = 10
        self.robot_radius = 20

        # ROBOT BODY
        self.canvas.create_oval(self.width  / 2 - self.robot_radius,
                                self.height / 2 - self.robot_radius,
                                self.width  / 2 + self.robot_radius,
                                self.height / 2 + self.robot_radius)

        # HOKUYO
        self.canvas.create_oval(self.width  / 2 - self.robot_radius / 5,
                                self.height / 2 - self.robot_radius / 5,
                                self.width  / 2 + self.robot_radius / 5,
                                self.height / 2 + self.robot_radius / 5,
                                fill = 'black')
        # HEAD
        coords = [
            self.width  / 2,
            self.height / 2 - self.robot_radius,
            self.width  / 2 - self.robot_radius + 8,
            self.height / 2 - head_cte,
            self.width  / 2 + self.robot_radius - 8,
            self.height / 2 - head_cte]

        self.canvas.create_polygon(coords)
        self.canvas.grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0), pady = (10, 10))
        
        context.set_draw_panel(self)
