from tkinter import *
from gui.app.components.canvas.grid  import Grid
from gui.app.components.canvas.light import Light
from gui.app.components.canvas.objects import Objects
# from gui.app.canvas.robot.robot import Robot
# from gui.app.canvas.animation import Animation


class CanvasPanel:
    def __init__(self, context):
        frame = Frame(context.content, borderwidth = 5, relief = "flat", width = 900, height = 900, 
                                                                    bg = context.color['background'])

        size = context.get_canvas_size()
        canvas = Canvas(frame, width = size['width'], height = size['height'], 
                                                                    bg = context.color['canvas'])


        context.set_canvas(canvas)

        Grid(context)
        light = Light(context)
        # robot = Robot(context)
        Objects(context)

        canvas.bind("<Button-3>", light.plot)
        # self.canvas.bind("<Button-1>", self.left_click)
        context.plot_map()
        frame.grid(column = 0, row = 0, columnspan = 3, rowspan = 2, 
                                                        sticky = (N, S, E, W))                          
        canvas.pack()

        # Animation(context)
 
     # def resize(self, new_size_x, new_size_y):
    #     print(f"New canva's size: {new_size_x}x{new_size_y}")
    #     self.context.set_canvas_size(new_size_x, new_size_y)

    #     self.light.plot()
    #     self.robot.plot()
    #     self.canvas.configure(width = new_size_x, height = new_size_y)
    #     self.context.plot_map()

    # def right_click(self, e_point):
    #     light_position = self.controller.set_position(e_point.x, e_point.y)
    #     self.light.plot(light_position)

    #     e_point.y = self.size['y'] - e_point.y # CHANGING REFERENCE SYSTEM
        
    #     label_pos_x, label_pos_y = self.controller.px_point_to_m(e_point.x, e_point.y)
    #     self.context.panel_update_value('label_light_pose_x', label_pos_x)
    #     self.context.panel_update_value('label_light_pose_y', label_pos_y)

    #     if self.robot.get_pose() is not None:
    #         self.context.disable_button_run()
    #         self.context.run_simulation()

    # def left_click(self, e_point):
    #     robot_position = self.controller.set_position(e_point.x, e_point.y)
    #     self.robot.plot(robot_position)

    #     e_point.y = self.size['y'] - e_point.y # CHANGING REFERENCE SYSTEM

    #     pose_x, pose_y = self.controller.px_point_to_m(e_point.x, e_point.y)

    #     self.context.panel_update_value('entry_pose_x', pose_x)
    #     self.context.panel_update_value('entry_pose_y', pose_y)

    #     if self.light.get_position() is not None:
    #         self.context.enable_button_run()