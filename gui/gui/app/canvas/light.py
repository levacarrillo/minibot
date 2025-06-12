from tkinter import PhotoImage
from PIL import ImageTk

class Light:
    def __init__(self, parent_panel):
        self.canvas = parent_panel.canvas
        controller  = parent_panel.controller        

        self.plotted = False
        self.pose_x = '0.00'
        self.pose_y = '0.00'
        self.img = PhotoImage( file = controller.get_file_path('light'))
        self.img.zoom(50, 50)

    def plot(self, pose_x, pose_y):
        self.plotted = self.canvas.create_image(pose_x, pose_y, image = self.img)

    def delete(self):
        self.canvas.delete(self.plotted)