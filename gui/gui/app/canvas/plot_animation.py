import time

class PlotAnimation:
    def __init__(self, canvas_panel):
        self.canvas = canvas_panel
        self.robot = canvas_panel.robot
        print('Motion object...')

    def run(self):
        i = 0
        while(True):
            i += 1
            print(i)
            self.robot.plot(i, i)
            print('moving...')
            time.sleep(0.0001)
            self.canvas.canvas.update()
