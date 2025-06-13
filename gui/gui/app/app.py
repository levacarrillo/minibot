import tkinter as tk
from tkinter import Frame, BOTH
from gui.app.colors import colors
from gui.app.menu_bar import MenuBar
from gui.domain.service import Service
from gui.app.controller import Controller
from gui.app.canvas.canvas_panel import CanvasPanel
from gui.app.side_panel.side_panel import SidePanel


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mobile Robot 2D Simulator v.2")

        self.color = colors
        self.service = Service()
        self.controller = Controller()
        self.content = Frame(self)

        self.side_panel = SidePanel(self)
        self.canvas_panel = CanvasPanel(self)
        self.content.pack(fill=BOTH, expand=True)
        self.menu_bar = MenuBar(self)
    
    def run(self):
        self.canvas_panel.print_grid()
        self.mainloop()