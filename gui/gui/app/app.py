import tkinter as tk
from tkinter import Frame, BOTH
from gui.config.colors import *
from gui.app.menu.menu_bar import *
from gui.app.canvas.canvas_panel import *
from gui.app.app_context import AppContext
from gui.app.side_panel.side_panel import *


class App(tk.Tk):
    def __init__(self, controller):
        super().__init__()
        self.title("Mobile Robot 2D Simulator v.2")

        content = Frame(self)

        context = AppContext(
            app        = self,
            color      = colors,
            content    = content,
            controller = controller
        )
        
        controller.update_params()
        SidePanel(context)
        CanvasPanel(context)
        MenuBar(context)
        content.pack(fill=BOTH, expand=True)
    
    def run(self):
        self.mainloop()
