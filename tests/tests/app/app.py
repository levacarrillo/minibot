import tkinter as tk
# from tkinter import Frame, BOTH
from tkinter import *

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Mobile Robot GUI for Tests")

        # content = Frame(self)
        window_width = 390
        window_height = 310
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        x = (screen_width // 2) + (window_width // 2)
        y = (screen_height // 2) - (window_height)
        self.geometry(f"{window_width}x{window_height}+{x}+{y}")

        button_stop   = Button(text="Stop")
        button_start  = Button(text="Start")
        button_params = Button(text="Params")
        label_minibot_id = Label(
            self,
            text = 'Minibot 2',
            font=('arial', 11, "bold"),
            anchor=CENTER,
            bg="white",
            justify=CENTER,
            padx=6,
            pady=4,
            wraplength=250
        )

        button_stop.grid(column = 0, row = 1, sticky = (N, W), padx = (5, 0))
        button_start.grid(column = 1, row = 1, sticky = (N, W), padx = (5, 0))
        button_params.grid(column = 2, row = 1, sticky = (N, W), padx = (5, 0))
        label_minibot_id.grid(column = 4, row = 1, sticky = (N, W), padx = (5, 0))

        radian_var   = StringVar()
        angle_var    = StringVar(self, value="0°")
        distance_var = StringVar(self, value="0.0cm")

        label_angle = Label(self, text="Angle:")
        label_distance = Label(self, text="Distance:")

        angle_var.trace_add("write", self.plot)
        distance_var.trace_add("write", self.plot)

        radian_entry = Entry(self, state='readonly', textvariable=radian_var, width=5)
        angle_entry  = Spinbox(self, from_=-180, to=180, increment=1, textvariable=angle_var, width=5)
        distance_entry = Spinbox(self, from_=-100, to=100, increment=0.1, textvariable=distance_var, width=5)

        label_angle.grid(column = 0, row = 4,columnspan = 2, sticky = (N, W), padx = (5, 0))
        angle_entry.grid(column = 1, row = 4, columnspan = 1, sticky = (N, W), padx = (5, 0))
        radian_entry.grid(column =2 , row = 4, columnspan = 2, sticky = (N, W), padx = (5, 0))
        label_distance.grid(column = 4, row = 4, columnspan = 2, sticky = (N, W), padx = (5, 0))
        distance_entry.grid(column = 5, row = 4, columnspan = 2, sticky = (N, W), padx = (5, 0))

        # content.pack(fill=BOTH, expand=True)

    def plot(self, *args):
        print('printings')
    def run(self):
        self.mainloop()