from tkinter import PhotoImage
from PIL import ImageTk

class Light:
    def __init__(self, canvas_panel):
        self.controller   = canvas_panel.controller
        self.canvas_panel = canvas_panel
        self.canvas       = canvas_panel.canvas
        self.size         = canvas_panel.size
  
        self.pose  = self.controller.set_pose(-1, -1, 0) # OUT OF CANVAS
        self.image = False
        self.img   = PhotoImage(file = self.controller.get_file_path('light.png'))
        self.img.zoom(50, 50)

    def plot(self, pose_x = None, pose_y = None):
        if (pose_x and pose_y) is None:
            new_size = self.canvas_panel.size
            self.pose = self.controller.scale_pose(self.size, new_size, self.pose)
        else:
            self.pose = self.controller.set_pose(pose_x, pose_y, 0)
        
        self.size = self.canvas_panel.size
        if self.image:
            self.canvas.delete(self.image)
        self.image = self.canvas.create_image(self.pose['x'], self.pose['y'], image = self.img)

    def delete(self):
        self.canvas.delete(self.image)