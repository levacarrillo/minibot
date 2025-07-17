from tkinter import *
from tkinter import ttk
from tests.app.components.params_pop_up import *


class BehaviorsPanel:
    def __init__(self, context):
        self.behavior     = StringVar()
        self.run_stop     = StringVar(value = 'Run')
        self.current_step = StringVar(value = 'Step: 0')

        frame       = LabelFrame(context.content, text = 'Behaviors')
        cb_behavior = ttk.Combobox(frame, textvariable = self.behavior, width = 18)

        label_steps   = Label(frame, textvariable = self.current_step, width = 9)

        button_params = Button(frame, text = 'Params', command = lambda: ParamsPopUp(context))
        button_run    = Button(frame, textvariable = self.run_stop, command = context.on_click_behavior, width = 4)

        self.behavior_list = cb_behavior['values']

        frame          .grid(column = 0, row = 3, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 5)
        cb_behavior    .grid(column = 0, row = 0, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 1)
        label_steps    .grid(column = 1, row = 0, sticky = (N, W), padx = (5, 0), pady = (5, 10), columnspan = 2)
        button_params  .grid(column = 3, row = 0, sticky = (N, W), padx = (5, 3), columnspan = 1)
        button_run     .grid(column = 4, row = 0, sticky = (N, W), padx = (5, 5), columnspan = 1)

        context.set_behaviors_panel(self)
