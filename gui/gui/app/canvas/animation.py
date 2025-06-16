import time

class Animation:
    def __init__(self, canvas_panel):
        self.canvas  = canvas_panel
        self.robot   = canvas_panel.robot
        self.controller = canvas_panel.controller


    def run(self):
        i = 0
        while(True):

            i += 1
            self.robot.plot(i, i)
            print('moving...')
            print(self.controller.get_goal_point())
            time.sleep(0.001)
            self.canvas.canvas.update()
