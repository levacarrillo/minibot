from tkinter import *
from gui.app.canvas.robot import Robot
from gui.app.canvas.light import Light

class CanvasPanel:
    def __init__(self, app):
        self.color      = app.color
        self.controller = app.controller
        self.env_section   = app.side_panel.env_section
        self.robot_section = app.side_panel.robot_section

        app.frame = Frame(app.content)
        app.frame = Frame(app.content, borderwidth = 5, relief = "flat", width = 900,
                                        height = 900, bg=self.color['background'])

        self.grid = []
        self.scale = { 'x': 1, 'y': 1 }
        self.size  = { 'x': 500, 'y': 500 } # PIXELS

        self.canvas = Canvas(app.frame, width = self.size['x'], height = self.size['y'],
                                        bg=self.color['canvas'])

        self.light = Light(self)
        self.robot = Robot(self)

        self.canvas.bind("<Button-3>", self.right_click)
        self.canvas.bind("<Button-1>", self.left_click)
        
        app.frame.grid(column = 0, row = 0, columnspan = 3, rowspan = 2, sticky = (N, S, E, W))
        self.canvas.pack()

    def print_grid(self, line_per_meters = 10):
        for i in self.grid:
            self.canvas.delete(i)
        self.grid =[]

        for X in self.scale:
            edge = self.controller.get_edge(self.size[X], self.scale[X], line_per_meters)
            for i in range(0, int(self.scale[X]) * line_per_meters):
                self.grid.append(
                    self.canvas.create_line(
                        i * edge if X == 'x' else 0,
                        i * edge if X != 'x' else 0,
                        i * edge if X == 'x' else self.size[X],
                        i * edge if X != 'x' else self.size[X],
                        dash=(4, 4),
                        fill=self.color['grid']
                    )
                )

    def right_click(self, e_point):
        self.light.plot(e_point.x, e_point.y)
        e_point.y = self.size['y'] - e_point.y # CHANGING REFERENCE SYSTEM

        self.env_section.label_light_pose_x.config(text=self.controller.pixels_to_m(
                                                                            self.scale['x'],
                                                                            self.size ['x'],
                                                                            e_point.x
                                                                            )
                                                                        )
        self.env_section.label_light_pose_y.config(text=self.controller.pixels_to_m(
                                                                            self.scale['y'],
                                                                            self.size['y'],
                                                                            e_point.y
                                                                            )
                                                                        )


    def left_click(self, e_point):
        self.robot.plot(e_point.x, e_point.y)
        e_point.y = self.size['y'] - e_point.y # CHANGING REFERENCE SYSTEM

        self.robot_section.entry_pose_x.delete(0, END)
        self.robot_section.entry_pose_y.delete(0, END)

        self.robot_section.entry_pose_x.insert(0, self.controller.pixels_to_m(
                                                                    self.scale['x'],
                                                                    self.size['x'],
                                                                    e_point.x
                                                                    )
                                                                )
        self.robot_section.entry_pose_y.insert(0, self.controller.pixels_to_m(
                                                                    self.scale['y'],
                                                                    self.size['y'],
                                                                    e_point.y
                                                                    )
                                                                )

