import tkinter as tk
from tkinter import Frame, BOTH
from gui.config.colors import colors
from gui.app.menu.menu_bar import MenuBar
from gui.app.canvas.canvas_panel import CanvasPanel
from gui.app.side_panel.side_panel import SidePanel


class App(tk.Tk):
    def __init__(self, controller):
        super().__init__()
        self.title("Mobile Robot 2D Simulator v.2")

        self.color = colors
        self.controller = controller
        self.content = Frame(self)
        
        self.controller.update_params()
        self.side_panel = SidePanel(self)
        self.canvas_panel = CanvasPanel(self)
        self.content.pack(fill=BOTH, expand=True)
        self.menu_bar = MenuBar(self)

        self.after(20, self.update_loop)

    def update_loop(self):
        if self.side_panel.buttons_section.simulation_running:
            goal_pose = self.controller.get_goal_pose()
            if goal_pose is not None:
                self.canvas_panel.robot.move(goal_pose['distance'], goal_pose['angle'])
        self.after(20, self.update_loop)
    
    def run(self):
        self.canvas_panel.grid.plot()
        self.mainloop()
