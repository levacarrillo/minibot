from tkinter import *
from PIL import ImageTk

class CanvasPanel:
    def __init__(self, app):
        self.color = app.colors
        app.frame = Frame(app.content)
        app.frame = Frame(app.content, borderwidth = 5, relief = "flat", width = 900, height = 900, bg=self.color.background)

        self.grid = []
        self.scale_x = 1
        self.scale_y = 1
        self.canvas_size_x = 500 # PIXELS
        self.canvas_size_y = 500 # PIXELS

        self.canvas = Canvas(app.frame, width = self.canvas_size_x, height = self.canvas_size_y, bg=self.color.canvas)
        self.light_img = PhotoImage( file = '/home/knight/ros2_ws/src/minibot/gui/gui/resources/light.png')
        self.light_img.zoom(50, 50)

        self.light = False
        self.light_pose_x = 0
        self.light_pose_y = 0


        self.canvas.bind("<Button-3>", self.right_click)
        self.canvas.bind("<Button-1>", self.left_click)
        
        self.canvas.pack()
        app.frame.grid(column = 0, row = 0, columnspan = 3, rowspan = 2, sticky = (N, S, E, W))

    def right_click(self, e_point):
        if self.light:
            self.canvas.delete(self.light)
        self.light = self.canvas.create_image(e_point.x, e_point.y, image = self.light_img)

        self.light_pose_x = self.scale_x * e_point.x / self.canvas_size_x
        self.light_pose_y = self.scale_y - (( self.scale_y * e_point.y ) / self.canvas_size_y)

        # self.label_light_x_var.config(text=str(self.light_pose_x)[:4])
        # self.label_light_y_var.config(text=str(self.light_pose_y)[:4])

    def left_click(self, e_point):
        print('Clicked left')

    def print_grid(self, line_per_meters = 10):
        for i in self.grid:
            self.canvas.delete(i)
        self.grid =[]

        for i in range(0, int(self.scale_x) * line_per_meters):
            self.grid.append(self.canvas.create_line(i * self.canvas_size_x / (self.scale_x * line_per_meters), 0, i * self.canvas_size_x / (self.scale_x * line_per_meters), self.canvas_size_y,  dash=(4, 4), fill=self.color.grid))
        for i in range(0, int(self.scale_y) * line_per_meters):
            self.grid.append(self.canvas.create_line(0, i * self.canvas_size_y / (self.scale_y * line_per_meters), self.canvas_size_x, i * self.canvas_size_y / (self.scale_y * line_per_meters),   dash=(4, 4), fill=self.color.grid))