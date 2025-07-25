from tkinter import *


class ButtonsSection:
    def __init__(self, context):
        label_simulator  = Label(context.side_frame, text = "Simulator", font = ('arial', 11, 'bold'))
        plot_topological = Button(context.side_frame, width = 18, text = "Plot topological", 
                                            command = context.plot_topological_map)
        button_run       = Button(context.side_frame, width = 18, text = "Run simulation", 
                                            command = context.run_simulation)
        button_run_last  = Button(context.side_frame, width = 18, text = "Run last simulation",
                                            command = context.last_simulation)
        button_stop      = Button(context.side_frame, width = 18, text = "Stop simulation",
                                            command = context.stop_simulation)
        
        self.plot_topological = plot_topological
        self.button_run       = button_run
        self.button_stop      = button_stop
        self.button_last      = button_run_last

        label_simulator  .grid(column = 4, row = 12, sticky = (N, W), padx = (5, 0), pady = (0, 5))
        plot_topological .grid(column = 4, row = 13, sticky = (N, W), padx = (5, 0))
        button_run       .grid(column = 4, row = 14, sticky = (N, W), padx = (5, 0))
        button_run_last  .grid(column = 4, row = 15, sticky = (N, W), padx = (5, 0))
        button_stop      .grid(column = 4, row = 16, sticky = (N, W), padx = (5, 0))

        context.set_buttons_section(self)
