import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import *

class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        self.root = tk.Tk()
        self.root.title('Mobile Robot Simulator')
        # self.root.geometry("800x600")
        self.root.resizable(False, False)

        self.content   = Frame(self.root)
        self.content.pack(fill=BOTH, expand=True)

        self.canvasX= 500
        self.canvasY= 500
	

        self.frame     = Frame(self.content,borderwidth = 5, relief = "flat", width = 900, height = 900)
        self.frame    .grid(column = 0 ,row = 0 ,columnspan = 3 ,rowspan = 2 ,sticky = (N, S, E, W))

        self.w = Canvas(self.frame, width = self.canvasX, height = self.canvasY, bg="#FFFFFF")
        self.w.pack()


        self.rightMenu = Frame(self.content,borderwidth = 5, relief = "flat", width = 300, height = 900)
        self.rightMenu.grid(column = 3 ,row = 0 ,columnspan = 3 ,rowspan = 2 ,sticky = (N, S, E, W))

        self.lableEnvironment   = Label(self.rightMenu ,text = "Settings")
        self.labelFile          = Label(self.rightMenu ,text = "Environment:")
        self.labelSteps         = Label(self.rightMenu ,text = "Steps:")
        self.labelBehavior		= Label(self.rightMenu ,text = "Behavior:")
        self.labelLightX        = Label(self.rightMenu ,text = "Light X:")
        self.labelLightY        = Label(self.rightMenu ,text = "Light Y:")
        self.labelStepsExcec        = Label(self.rightMenu ,text = "Steps:")
        self.labelConfiguration = Label(self.rightMenu ,text = "Configurations:")
        
        self.lableEnvironment  .grid(column = 0 ,row = 0 ,sticky = (N, W) ,padx = (5,5))
        self.labelFile         .grid(column = 0 ,row = 1 ,sticky = (N, W) ,padx = (10,5))
        self.labelSteps        .grid(column = 0 ,row = 2 ,sticky = (N, W) ,padx = (10,5))
        self.labelBehavior     .grid(column = 0 ,row = 3 ,sticky = (N, W) ,padx = (10,5))
        self.labelLightX       .grid(column = 0 ,row = 4 ,sticky = (N, W) ,padx = (10,5))
        self.labelLightY       .grid(column = 0 ,row = 5 ,sticky = (N, W) ,padx = (10,5))
        self.labelStepsExcec   .grid(column = 0 ,row = 6 ,sticky = (N, W) ,padx = (10,5))
        self.labelConfiguration.grid(column = 0 ,row = 7 ,sticky = (N, W) ,padx = (10,5))

        self.labelRobot     = Label(self.rightMenu ,text = "Robot"             )
        self.labelPoseX     = Label(self.rightMenu ,text = "Pose X:"           )
        self.labelPoseY     = Label(self.rightMenu ,text = "Pose Y:"           )
        self.labelAngle     = Label(self.rightMenu ,text = "Angle:"            )
        self.labelRadio     = Label(self.rightMenu ,text = "Radio:"            )
        self.labelAdvance   = Label(self.rightMenu ,text = "Magnitude Advance:")
        self.labelTurnAngle = Label(self.rightMenu ,text = "Turn Angle:"       )

        self.labelRobot     .grid(column = 4 ,row = 0 ,sticky = (N, W) ,padx = (5,5))     
        self.labelPoseX     .grid(column = 4 ,row = 1 ,sticky = (N, W) ,padx = (10,5))
        self.labelPoseY     .grid(column = 4 ,row = 2 ,sticky = (N, W) ,padx = (10,5))
        self.labelAngle     .grid(column = 4 ,row = 3 ,sticky = (N, W) ,padx = (10,5))
        self.labelRadio     .grid(column = 4 ,row = 5 ,sticky = (N, W) ,padx = (10,5))
        self.labelAdvance   .grid(column = 4 ,row = 6 ,sticky = (N, W) ,padx = (10,5))
        self.labelTurnAngle .grid(column = 4 ,row = 7 ,sticky = (N, W) ,padx = (10,5))


        self.labelVelocity = Label(self.rightMenu ,text = "Execution velocity:")
        self.sliderVelocity =Scale(self.rightMenu, from_=1, to=3, orient=HORIZONTAL ,length=150)

        self.buttonSetZero  = Button(self.rightMenu ,width = 8, text = "Angle Zero")
        self.buttonSetZero  .grid(column = 5 ,row = 4 ,columnspan = 2 ,sticky = (N, W), padx = 5)

        self.labelVelocity	.grid(column = 4 ,row = 8 ,sticky = (N, W) ,padx = (10,5))
        self.sliderVelocity .grid(column = 4 ,row = 9 ,columnspan = 2 ,rowspan = 2 ,sticky = (N, W), padx = 5)

        self.lableSensors     = Label(self.rightMenu, text = "Sensors"       )
        self.labelNumSensors  = Label(self.rightMenu, text = "Num Sensors:"  )
        self.labelOrigin      = Label(self.rightMenu, text = "Origin angle:" )
        self.labelRange       = Label(self.rightMenu, text = "Range:"        )
        self.labelValue       = Label(self.rightMenu, text = "Value:"        )

        self.lableSensors       .grid(column = 0 ,row = 12  ,sticky = (N, W) ,padx = (5,5))     
        self.labelNumSensors    .grid(column = 0 ,row = 13  ,sticky = (N, W) ,padx = (10,5))
        self.labelOrigin        .grid(column = 0 ,row = 14  ,sticky = (N, W) ,padx = (10,5))
        self.labelRange         .grid(column = 0 ,row = 15  ,sticky = (N, W) ,padx = (10,5))
        self.labelValue         .grid(column = 0 ,row = 16  ,sticky = (N, W) ,padx = (10,5))

        self.lableSimulator        = Label (self.rightMenu ,text = "Simulator")
        self.buttonPlotTopological = Button(self.rightMenu ,width = 20, text = "Plot Topological")
        self.buttonRunSimulation   = Button(self.rightMenu ,width = 20, text = "Run simulation")
        self.buttonStop            = Button(self.rightMenu ,width = 20, text = "Stop")

        self.lableSimulator     .grid(column = 4 ,row = 12  ,sticky = (N, W) ,padx = (5,5))
        self.buttonPlotTopological   .grid(column = 4 ,row = 15  ,sticky = (N, W) ,padx = (10,5))
        self.buttonRunSimulation.grid(column = 4 ,row = 17 ,sticky = (N, W) ,padx = (10,5))
        self.buttonStop         .grid(column = 4 ,row = 18 ,sticky = (N, W) ,padx = (10,5))

        self.buttonMapLess = Button(self.rightMenu ,width = 5)
        self.buttonMapMore = Button(self.rightMenu ,width = 5)


        self.buttonMapLess.grid(column = 1 ,row = 19 ,columnspan=1 ,sticky = (N, W) ,padx = 5)
        self.buttonMapMore.grid(column = 1 ,row = 19 ,columnspan=1 ,sticky = (N, E) ,padx = 5)

        self.root.after(100, self.ros_spin)
        self.root.mainloop()


    def ros_spin(self):
        # print('Hello GUI')
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(100, self.ros_spin)
    

def main(args=None):
    print('INITIALIZING GUI...')
    rclpy.init(args=args)
    simulator = Simulator()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

