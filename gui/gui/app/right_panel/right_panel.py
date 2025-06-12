from tkinter import *
from gui.app.right_panel.environment_section import EnvironmentSection
from gui.app.right_panel.sensors_section import SensorsSection
from gui.app.right_panel.robot_section import RobotSection
from gui.app.right_panel.buttons_section import ButtonsSection

class RightPanel:
    def __init__(self, app):
        controller = app.controller
        self.right_menu = Frame(app.content, borderwidth = 5, relief = "flat", width = 300)
        EnvironmentSection(self.right_menu, controller)
        SensorsSection(self.right_menu, controller)
        RobotSection(self.right_menu, controller)
        ButtonsSection(self.right_menu, controller)

        self.right_menu.grid(column = 3, row = 0, columnspan = 3, rowspan = 2, sticky = (N, S, E, W))