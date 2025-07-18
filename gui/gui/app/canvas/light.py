from tkinter import PhotoImage
from PIL import ImageTk


class Light:
    def __init__(self, context):
        self.canvas     = context.canvas
        # self.controller = context.controller
  
        self.position = None
        self.image = False
        # self.img   = PhotoImage(file = self.controller.get_file_path('light.png'))
        # self.img.zoom(50, 50)
        context.set_light(self)

    def plot(self, position = None):
        if position is not None:
            self.position = position
        # elif self.position is not None:
        #     self.position = self.controller.remap_position(self.position)

        if self.image:
            self.canvas.delete(self.image)

        if self.position is not None:
            self.image = self.canvas.create_image(self.position['x'], self.position['y'], 
                                                                        image = self.img)
    def get_position(self):
        return self.position

    def delete(self):
        self.canvas.delete(self.image)
