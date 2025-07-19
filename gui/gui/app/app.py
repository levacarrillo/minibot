from tkinter import *
from gui.config.colors import colors
from gui.app.app_context import AppContext
from gui.app.components.menu.menu_bar import MenuBar
from gui.app.components.canvas.canvas_panel import CanvasPanel
from gui.app.components.side_panel.side_panel import SidePanel


class App(Tk):
    def __init__(self, service, node, file_manager):
        super().__init__()
        self.title("Mobile Robot 2D Simulator v.2")

        content = Frame(self).place()

        context = AppContext(
            app      = self,
            color    = colors,
            content  = content,
            service  = service,
            ros      = node,
            file_manager  = file_manager
        )
        
        context.update_params()

        MenuBar(context)
        CanvasPanel(context)
        SidePanel(context)
    
        context.loop()

    def run(self):
        self.mainloop()
