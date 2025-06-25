from tkinter import *
from gui.app.side_panel.environment_section import EnvironmentSection
from gui.app.side_panel.sensors_section import SensorsSection
from gui.app.side_panel.robot_section import RobotSection
from gui.app.side_panel.buttons_section import ButtonsSection


class SidePanel:
    def __init__(self, context):
        side_frame = Frame(context.content, borderwidth = 5, relief = "flat", width = 300)
        context.set_side_frame(side_frame)

        self.env_section = EnvironmentSection(context)
        self.sensors_section = SensorsSection(context)
        self.robot_section   = RobotSection(context)
        self.buttons_section = ButtonsSection(context)

        context.set_side_panel(self)
        side_frame.grid(column = 3, row = 0, columnspan = 3, rowspan = 2, 
                                sticky = (N, S, E, W))