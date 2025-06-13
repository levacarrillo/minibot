from tkinter import PhotoImage
from PIL import ImageTk

class Light:
    def __init__(self, parent_panel):
        self.canvas = parent_panel.canvas
        controller  = parent_panel.controller        

        self.image = False
        self.img = PhotoImage( file = controller.get_file_path('light.png'))
        self.img.zoom(50, 50)

    def plot(self, pose_x, pose_y):
        if self.image:
            self.canvas.delete(self.image)
        self.image = self.canvas.create_image(pose_x, pose_y, image = self.img)

    def delete(self):
        self.canvas.delete(self.image)