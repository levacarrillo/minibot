import tkinter as tk
from tkinter import Frame, BOTH
from gui.app.controller import Controller
from gui.app.colors import Colors
from gui.app.menu_bar import MenuBar
from gui.app.canvas_panel import CanvasPanel
from gui.app.right_panel.right_panel import RightPanel


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mobile Robot 2D Simulator v.2")

        self.controller = Controller()
        self.colors = Colors()
        self.content = Frame(self)

        self.menu_bar = MenuBar(self)
        self.canvas_panel = CanvasPanel(self)
        self.right_panel = RightPanel(self)
        self.content.pack(fill=BOTH, expand=True)
    
    def run(self):
        self.canvas_panel.print_grid()
        self.mainloop()