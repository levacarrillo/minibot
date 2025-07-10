from tkinter import *


class ButtonsSection:
    def __init__(self, context):
        self.context    = context
        self.controller = context.controller

        self.label_simulator = Label(context.side_frame, text = "Simulator")

        self.plot_topological= Button(context.side_frame, width = 18, text = "Plot topological", 
                                        command = self.plot_topological, state = DISABLED)
        self.button_run      = Button(context.side_frame, width = 18, text = "Run simulation", 
                                        command = self.run_simulation, state = DISABLED)
        self.button_run_last = Button(context.side_frame, width = 18, text = "Run last simulation",
                                        command = self.run_last_simulation, state = NORMAL)
        self.button_stop     = Button(context.side_frame, width = 18, text = "Stop simulation",
                                        command = self.stop_simulation, state = DISABLED)

        self.label_simulator  .grid(column = 4, row = 12, sticky = (N, W), padx = (5, 0))
        self.plot_topological .grid(column = 4, row = 13, sticky = (N, W), padx = (5, 0))
        self.button_run       .grid(column = 4, row = 14, sticky = (N, W), padx = (5, 0))
        self.button_run_last  .grid(column = 4, row = 15, sticky = (N, W), padx = (5, 0))
        self.button_stop      .grid(column = 4, row = 16, sticky = (N, W), padx = (5, 0))

        context.set_buttons_section(self)

    def plot_topological(self):
        self.context.plot_topological_map()

    def run_simulation(self):
        self.context.disable_button_run()
        self.context.run_simulation()
    
    def run_last_simulation(self):
        self.context.last_simulation()

    def stop_simulation(self):
        self.controller.finish_movement()
        self.context.simulation_running = False
        self.context.enable_button_run()
