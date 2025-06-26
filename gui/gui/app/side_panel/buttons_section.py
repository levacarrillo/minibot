from tkinter import *


class ButtonsSection:
    def __init__(self, context):
        self.context    = context
        side            = context.side_frame
        self.controller = context.controller

        self.label_simulator = Label (side, text = "Simulator")
        self.label_velocity  = Label(side,  text = "Execution velocity:")
        self.slider_velocity = Scale(side,  from_= 1, to=3, orient = HORIZONTAL,
                                        length = 162, command = self.scale_value)

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
        params = {
            "behavior"       : self.context.env_section.behavior_list_cb.get(),
            "run_behavior"   : True,
            "step"           : int(0),
            "max_steps"      : int(self.context.env_section.steps_entry.get()),
            "max_advance"    : float(self.context.robot_section.entry_advance.get()),
            "max_turn_angle" : float(self.context.robot_section.entry_turn_angle.get()),
            "light_threshold": float(self.context.sensors_section.entry_light.get()),
            "laser_threshold": float(self.context.sensors_section.entry_laser.get())
        }
        
        self.context.simulation_running = True
        self.controller.run_simulation(params)
        self.context.route.delete()

    def scale_value(self, value):
        self.context.set_velocity_slider(value)

    def stop_simulation(self):
        self.controller.finish_movement()
        self.context.simulation_running = False
