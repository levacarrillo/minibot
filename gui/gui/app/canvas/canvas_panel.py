from tkinter import *
from gui.app.canvas.grid  import Grid
from gui.app.canvas.light import Light
from gui.app.canvas.robot.robot import Robot
from gui.app.canvas.animation import Animation


class CanvasPanel:
    def __init__(self, context):
        self.app = context.app
        content  = context.content
        color    = context.color
        self.context         = context
        self.controller      = context.controller
        self.env_section     = context.env_section
        self.robot_section   = context.robot_section
        self.buttons_section = context.buttons_section

        self.app.frame = Frame(content, borderwidth = 5, relief = "flat", width = 900,
                                        height = 900, bg=color['background'])

        self.scale = self.controller.set_canvas_scale(1, 1)
        self.size  = self.controller.set_canvas_size(500, 500) # PIXELS

        self.canvas = Canvas(self.app.frame, width = self.size['x'], 
                                        height = self.size['y'], bg=color['canvas'])

        context.set_canvas_panel(self)

        self.grid  = Grid(context)
        self.light = Light(context)
        self.robot = Robot(context)

        self.canvas.bind("<Button-3>", self.right_click)
        self.canvas.bind("<Button-1>", self.left_click)
        
        self.app.frame.grid(column = 0, row = 0, columnspan = 3, rowspan = 2, 
                                                        sticky = (N, S, E, W))
        self.canvas.pack()

        Animation(context)

    def resize(self, new_size_x, new_size_y):
        print(f"New canva's size: {new_size_x}x{new_size_y}")
        new_size = self.controller.set_canvas_size(new_size_x, new_size_y)
        self.context.set_canvas_size(new_size)

        self.light.plot()
        self.grid.plot()
        self.robot.plot()
        # self.robot.route.trace(trace_init_pose, trace_final_pose)
        self.canvas.configure(width = new_size_x, height = new_size_y)
 
    def right_click(self, e_point):
        light_position = self.controller.set_position(e_point.x, e_point.y)
        self.light.plot(light_position)

        e_point.y = self.size['y'] - e_point.y # CHANGING REFERENCE SYSTEM
        
        label_pos_x, label_pos_y = self.controller.px_point_to_m(e_point.x, e_point.y)
        self.env_section.label_light_pose_x.config(text = label_pos_x)
        self.env_section.label_light_pose_y.config(text = label_pos_y)

        self.context.simulation_running = True

    def left_click(self, e_point):
        robot_position = self.controller.set_position(e_point.x, e_point.y)
        self.robot.plot(robot_position)

        e_point.y = self.size['y'] - e_point.y # CHANGING REFERENCE SYSTEM

        self.robot_section.entry_pose_x.delete(0, END)
        self.robot_section.entry_pose_y.delete(0, END)

        pose_x, pose_y = self.controller.px_point_to_m(e_point.x, e_point.y)

        self.robot_section.entry_pose_x.insert(0, pose_x)
        self.robot_section.entry_pose_y.insert(0, pose_y)
