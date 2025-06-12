from tkinter import *

class ButtonsSection:
    def __init__(self, right_menu, controller):
        self.label_simulator = Label (right_menu, text = "Simulator")
        self.label_velocity  = Label(right_menu,  text = "Execution velocity:")
        self.slider_velocity = Scale(right_menu,  from_=1, to=3, orient = HORIZONTAL, length=162)

        self.button_run      = Button(right_menu, width = 17, text = "Run simulation")
        self.button_stop     = Button(right_menu, width = 17, text = "Stop simulation")

        self.label_simulator  .grid(column = 4, row = 12, sticky = (N, W), padx = (5, 0))
        self.label_velocity	  .grid(column = 4, row = 15, sticky = (N, W), padx = (5, 0))
        self.slider_velocity  .grid(column = 4, row = 16, columnspan = 2, rowspan = 1, sticky = (N, W), padx = 5)
        self.button_run       .grid(column = 4, row = 13, sticky = (N, W), padx = (5, 0))
        self.button_stop      .grid(column = 4, row = 14, sticky = (N, W), padx = (5, 0))