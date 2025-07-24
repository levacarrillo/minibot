class Sensors:
    def __init__(self, context):
        self.context = context
        context.set_sensors(self)

    def plot(self, laser_readings = None):
        lasers_list = self.context.get_lasers_lines()
        for laser in lasers_list:
            self.context.canvas.create_line(laser, fill = self.context.color['laser'],
                                                            tags = ('robot', 'laser'))

    def delete(self):
        self.context.canvas.delete('laser')
