from tkinter import *
from gui.app.side_panel.environment_section import EnvironmentSection
from gui.app.side_panel.sensors_section import SensorsSection
from gui.app.side_panel.robot_section import RobotSection
from gui.app.side_panel.buttons_section import ButtonsSection


class SidePanel:
    def __init__(self, context):
        side_frame = Frame(context.content, borderwidth = 5, relief = "flat", width = 300)
        context.set_side_frame(side_frame)

        EnvironmentSection(context)
        SensorsSection(context)
        RobotSection(context)
        ButtonsSection(context)
        side_frame.grid(column = 3, row = 0, columnspan = 3, rowspan = 2, sticky = (N, S, E, W))