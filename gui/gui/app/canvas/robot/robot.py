import time
from tkinter import END
from gui.app.canvas.robot.parts import Parts
from gui.app.canvas.robot.sensors import Sensors


class Robot:
    def __init__(self, context):
        self.context    = context
        self.color      = context.color
        self.canvas     = context.canvas
        self.controller = context.controller

        self.pose    = None
        self.radius  = None

        self.body    = None
        context.set_robot(self)

        self.sensors = Sensors(context)
        self.parts = Parts(context)

    def plot(self, position = None, rotation = 0):
        entry_angle  = self.context.get_param('entry_angle')
        entry_radius = self.context.get_param('entry_radius')
        angle = self.controller.normalize_angle(entry_angle) + rotation

        self.radius = self.controller.m_to_pixels(entry_radius)
        if position is not None:
            self.pose   = self.controller.set_pose(position['x'], position['y'], angle)
        elif self.pose is not None: # VERIFY IF ROBOT EXISTS
            new_pose   = self.controller.set_pose(self.pose['x'], self.pose['y'], angle)
            self.pose   = self.controller.remap_position(new_pose)


        if self.body:
            self.delete()
            if self.sensors:  
                self.sensors.delete()

        if self.context.show_sensors:
            self.sensors.plot()

        if self.pose is not None:
            self.body = self.parts.get(name = 'body')
            self.parts.get(name = 'hokuyo')
            self.parts.get(name = 'head')
            self.parts.get(name = 'left_wheel')
            self.parts.get(name = 'right_wheel')
        
        self.context.panel_update_value('entry_angle', angle)

    def rotate(self, angle):
        self.plot(rotation = angle)

    def displace(self, advance):
        x, y = self.controller.polar_to_cartesian(advance, self.pose['angle'])
        self.canvas.move('robot', x, y)
        self.pose = self.controller.set_pose(self.pose['x'] + x, self.pose['y'] + y, 
                                                                self.pose['angle'])
        label_pos_x, label_pos_y = self.controller.px_point_to_m(self.pose['x'], self.pose['y'])

        self.context.panel_update_value('entry_pose_x', label_pos_x)
        self.context.panel_update_value('entry_pose_y', label_pos_y)

    def get_pose(self):
        return self.pose

    def delete(self):
        self.canvas.delete('robot')
