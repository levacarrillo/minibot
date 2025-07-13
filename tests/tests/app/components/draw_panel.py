from tkinter import *


class DrawPanel:
    def __init__(self, context):
        self.edge = 220
        self.canvas = Canvas(context.content, width  = self.edge, 
                                              height = self.edge)

        # ROBOT BODY
        self.canvas.create_polygon(context.get_head_coords(self.edge))
        self.canvas.create_oval(  *context.get_body_coords(self.edge))
        self.canvas.create_oval(   context.get_hokuyo_coords(self.edge),
                                                        fill = 'black')

        self.canvas.grid(column = 0, row = 1, sticky = (N, W),
                                        padx = (5, 0), pady = (10, 10))
        context.set_draw_panel(self)

    def plot_spot_light(self, coords, color = '#d9d9d9'):
        self.canvas.create_oval(*coords, fill = color, outline = 'black',
                                                    tag = 'spot_lights')
    
    def plot_laser(self, coords):
        self.canvas.create_line(*coords, fill = 'red', tag = 'laser')

    def clear(self):
        self.canvas.delete('laser')
        self.canvas.delete('spot_lights')
