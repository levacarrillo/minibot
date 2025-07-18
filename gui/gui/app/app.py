from tkinter import *
from gui.config.colors import *
from gui.app.components.menu.menu_bar import *
from gui.app.canvas.canvas_panel import *
from gui.app.app_context import AppContext
from gui.app.components.side_panel.side_panel import SidePanel


class App(Tk):
    def __init__(self, controller):
        super().__init__()
        self.title("Mobile Robot 2D Simulator v.2")

        content = Frame(self).place()

        context = AppContext(
            app        = self,
            color      = colors,
            content    = content,
            controller = controller
        )
        
        controller.update_params()
        MenuBar(context)
        SidePanel(context)
        CanvasPanel(context)
    
    def run(self):
        self.mainloop()
