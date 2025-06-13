from tkinter import *
from gui.app.side_panel.environment_section import EnvironmentSection
from gui.app.side_panel.sensors_section import SensorsSection
from gui.app.side_panel.robot_section import RobotSection
from gui.app.side_panel.buttons_section import ButtonsSection

class SidePanel:
    def __init__(self, app):
        self.controller = app.controller
        self.side_panel = Frame(
            app.content,
            borderwidth = 5,
            relief = "flat",
            width = 300
        )
        self.env_section = EnvironmentSection(self)
        SensorsSection(self)
        self.robot_section = RobotSection(self)
        ButtonsSection(self)

        self.side_panel.grid(
            column = 3,
            row = 0,
            columnspan = 3,
            rowspan = 2,
            sticky = (N, S, E, W)
        )