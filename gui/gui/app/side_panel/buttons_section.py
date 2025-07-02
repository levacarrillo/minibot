from tkinter import *


class ButtonsSection:
    def __init__(self, context):
        self.context    = context
        self.controller = context.controller

        self.label_simulator = Label(context.side_frame, text = "Simulator")
        self.label_velocity  = Label(context.side_frame,  text = "Execution velocity:")
        self.slider_velocity = Scale(context.side_frame,  from_= 1, to=3, orient = HORIZONTAL,
                                        length = 162, command = self.slider_value)

        self.button_run      = Button(context.side_frame, width = 17, text = "Run simulation", 
                                                command = self.run_simulation, state = DISABLED)
        self.button_stop     = Button(context.side_frame, width = 17, text = "Stop simulation",
                                                command = self.stop_simulation, state = DISABLED)

        self.label_simulator  .grid(column = 4, row = 12, sticky = (N, W), padx = (5, 0))
        self.label_velocity	  .grid(column = 4, row = 15, sticky = (N, W), padx = (5, 0))
        self.button_run       .grid(column = 4, row = 13, sticky = (N, W), padx = (5, 0))
        self.button_stop      .grid(column = 4, row = 14, sticky = (N, W), padx = (5, 0))
        self.slider_velocity  .grid(column = 4, row = 16, columnspan = 2, rowspan = 1,
                                                        sticky = (N, W), padx = 5)
        context.set_buttons_section(self)

    def run_simulation(self):
        self.context.disable_button_run()
        self.context.run_simulation()

    def stop_simulation(self):
        self.controller.finish_movement()
        self.context.simulation_running = False
        self.context.enable_button_run()

    def slider_value(self, value):
        self.context.set_velocity_slider(value)

