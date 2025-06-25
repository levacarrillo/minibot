from tkinter import PhotoImage
from PIL import ImageTk

class Light:
    def __init__(self, context):
        controller   = context.controller
        self.canvas       = context.canvas
  
        self.pose  = controller.set_pose(0, 0, 0)
        self.image = False
        self.img   = PhotoImage(file = controller.get_file_path('light.png'))
        self.img.zoom(50, 50)
        context.set_light(self)


    def plot(self, pose):
        self.pose = pose
        if self.image:
            self.canvas.delete(self.image)
        self.image = self.canvas.create_image(self.pose['x'], self.pose['y'], image = self.img)
        
    def get_pose(self):
        return self.pose

    def delete(self):
        self.canvas.delete(self.image)
