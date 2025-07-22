class Route:
    def __init__(self, context):
        self.context = context

        self.list_angles    = []
        self.list_position  = []
        context.set_route(self)

    def initialize_route(self, start_position, start_angle):
        self.list_angles.append(start_angle)
        self.list_position.append(start_position)

    def trace(self, start_position, final_position, final_angle):
        line = [start_position['x'], start_position['y'], 
                final_position['x'], final_position['y']]

        self.list_angles   .append(final_angle)
        self.list_position .append(final_position)
        self.context.canvas.create_line(line, dash = (4, 4),
                    fill = self.context.color['trace'], tag = 'route')
        self.context.robot.plot()

    def is_empty(self):
        return True if len(self.list_position) == 0 else False

    def delete(self):
        self.list_angles   = []
        self.list_position = []
        self.context.canvas.delete('route')
