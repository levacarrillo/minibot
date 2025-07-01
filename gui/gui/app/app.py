import tkinter as tk
from tkinter import Frame, BOTH
from gui.app.app_context import AppContext
from gui.config.colors import colors
from gui.app.menu.menu_bar import MenuBar
from gui.app.canvas.canvas_panel import CanvasPanel
from gui.app.side_panel.side_panel import SidePanel


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
        # context.grid.plot()
    
    def run(self):
        self.mainloop()
