from tkinter import PhotoImage
from PIL import ImageTk


class Light:
    def __init__(self, context):
        self.context = context
        self.position = None
        self.image = None
        self.img   = PhotoImage(file = context.get_file_path('light.png'))
        self.img.zoom(50, 50)
        context.set_light(self)

    def plot(self, e_point = None):
        self.context.canvas.delete('light')
        if e_point:
            self.position = self.context.set_light_position(e_point.x, e_point.y)
        else:
            self.position = self.context.remap_position(self.position)

        self.image = self.context.canvas.create_image(self.position['x'], self.position['y'], 
                                                                image = self.img, tag = 'light')
    def get_position(self):
        return self.position

    def exists(self):
        return True if self.position else False