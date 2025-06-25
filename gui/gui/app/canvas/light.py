from tkinter import PhotoImage
from PIL import ImageTk

class Light:
    def __init__(self, canvas_panel):
        self.controller   = canvas_panel.controller
        self.canvas       = canvas_panel.canvas
  
        self.pose  = self.controller.set_pose(0, 0, 0)
        self.image = False
        self.img   = PhotoImage(file = self.controller.get_file_path('light.png'))
        self.img.zoom(50, 50)


    def plot(self, pose):
        self.pose = pose
        if self.image:
            self.canvas.delete(self.image)
        self.image = self.canvas.create_image(self.pose['x'], self.pose['y'], image = self.img)
        
    def get_pose(self):
        return self.pose

    def delete(self):
        self.canvas.delete(self.image)
