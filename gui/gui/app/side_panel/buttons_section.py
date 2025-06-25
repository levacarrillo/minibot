from tkinter import *


class ButtonsSection:
    def __init__(self, parent_panel):
        side               = parent_panel.side_panel
        self.controller    = parent_panel.controller
        self.env_section   = parent_panel.env_section
        self.robot_section = parent_panel.robot_section
        self.sensors_section    = parent_panel.sensors_section
        self.simulation_running = False

        self.label_simulator = Label (side, text = "Simulator")
        self.label_velocity  = Label(side,  text = "Execution velocity:")
        self.slider_velocity = Scale(side, from_= 1, to=3, orient = HORIZONTAL,
                                                length = 162)

        self.button_run      = Button(side, width = 17, text = "Run simulation", 
                                                command = self.run_simulation)
        self.button_stop     = Button(side, width = 17, text = "Stop simulation",
                                                command = self.stop_simulation)

        self.label_simulator  .grid(column = 4, row = 12, sticky = (N, W), padx = (5, 0))
        self.label_velocity	  .grid(column = 4, row = 15, sticky = (N, W), padx = (5, 0))
        self.button_run       .grid(column = 4, row = 13, sticky = (N, W), padx = (5, 0))
        self.button_stop      .grid(column = 4, row = 14, sticky = (N, W), padx = (5, 0))
        self.slider_velocity  .grid(column = 4, row = 16, columnspan = 2, rowspan = 1,
                                                sticky = (N, W), padx = 5)
    
    def run_simulation(self):
        # params = {
        #     "behavior"       : self.env_section.behavior_list_cb.get(),
        #     "run_behavior"   : True,
        #     "step"           : int(0),
        #     "max_steps"      : int(self.env_section.steps_entry.get()),
        #     "max_advance"    : float(self.robot_section.entry_advance.get()),
        #     "max_turn_angle" : float(self.robot_section.entry_turn_angle.get()),
        #     "light_threshold": float(self.sensors_section.entry_light.get()),
        #     "laser_threshold": float(self.sensors_section.entry_laser.get())
        # }
        
        self.simulation_running = True # CHECK AFTER
        # self.controller.run_simulation(params)

    def stop_simulation(self):
        self.controller.finish_movement()
        self.simulation_running = False
