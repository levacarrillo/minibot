from tkinter import Tk, Frame
from tests.app.app_context import AppContext
from tests.app.components.status_panel import *
from tests.app.components.behaviors_panel import *
from tests.app.components.cmd_pose_panel import *
from tests.app.components.cmd_vel_panel import *
from tests.app.components.draw_panel import *


class App(Tk):
    def __init__(self, controller):
        super().__init__()
        self.title('Mobile Robot GUI for testing')

        content = Frame(self).place()

        context = AppContext(
            app        = self,
            content    = content,
            controller = controller
        )

        StatusPanel(context)
        DrawPanel(context)
        CmdVelPanel(context)
        CmdPosePanel(context)
        BehaviorsPanel(context)

        context.loop()

    def run(self):
        self.mainloop()
