from tkinter import *
from tkinter import ttk
from tests.app.components.params_pop_up import *

class BehaviorsPanel:
    def __init__(self, context):
        frame  = LabelFrame(context.content, text = 'Behaviors')
        cb_behavior = ttk.Combobox(frame, values = [], width = 16)
        
        label_steps  = Label(frame, text = 'Steps:')
        sp_max_steps = Spinbox(frame, textvariable = StringVar(value = '99'), from_= 0, to = 150, increment = 1, width = 4)

        button_params = Button(frame, text = 'Params', command = lambda: ParamsPopUp(context))
        button_run    = Button(frame, text = 'Run', width = 4)

        frame          .grid(column = 0, row = 3, sticky = (N, W), padx = (5, 5), pady = (5, 5),  columnspan = 5)
        cb_behavior    .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        label_steps    .grid(column = 1, row = 0, sticky = (N, W), padx = (9, 0), pady = (4, 0),  columnspan = 1) 
        sp_max_steps   .grid(column = 2, row = 0, sticky = (N, W), padx = (5, 0), pady = (4, 5),  columnspan = 1)
        button_params  .grid(column = 3, row = 0, sticky = (N, W), padx = (5, 0), columnspan = 1)
        button_run     .grid(column = 4, row = 0, sticky = (N, W), padx = (5, 5), columnspan = 1)

        context.set_behaviors_panel(self)
