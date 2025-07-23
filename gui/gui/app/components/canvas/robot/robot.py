from gui.app.components.canvas.robot.sensors import Sensors


class Robot:
    def __init__(self, context):
        self.context  = context
        self.position = None
        self.radius   = None
        self.grasped  = None
        self.angle    = self.context.get_context_param('angle')

        context.set_robot(self)

        # self.sensors = Sensors(context)

    def plot(self, position = None):
        if position:
            self.position = position
        self.context.canvas.delete('robot')
        self.radius = self.context.get_context_param('radius')
        self._plot_parts()
    
    def _plot_parts(self):   
        circular_parts = self.context.get_circles_coords(self.position, self.radius)
        for part in circular_parts:
            self.context.canvas.create_oval(*part['coords'], outline = part['color'], 
                                            fill = part['color'], width = 1, tag = 'robot')
        polygon_parts = self.context.get_polygons_coords(self.position, self.radius, self.angle)
        for part in polygon_parts:
            self.context.canvas.create_polygon(part['coords'], outline = part['color'],
                                            fill = part['color'], width = 1, tag = 'robot')

    def rotate(self, rotation):
        self.angle    = self.context.get_context_param('angle') + rotation
        self.context.set_context_param('angle', self.angle)
        self.plot()

    def remap_position(self):
        self.plot(self.context.remap_position(self.position))

    def displace(self, advance):
        x, y = self.context.polar_to_cartesian(advance, self.angle)
        self.context.canvas.move('robot', x, y)
        self.position = self.context.format_to_position(self.position['x'] + x,
                                                        self.position['y'] + y)

    def get_position(self):
        return self.position
    
    def set_position(self, position):
        self.position = position

    def get_angle(self):
        return self.angle
    
    def set_angle(self, angle):
        self.angle = angle

    def grasp(self, object_name):
        if not self.context.objects.in_field:
            # print('NO OBJECTS IN FIELD')
            return False

        min_distance = self.radius + 50
        
        for obj in self.context.objects.list:
            if obj['name'] == object_name:
                d = self.context.get_magnitude_between_two_points(self.pose, obj)
                if d < min_distance:
                    self.grasped = obj['name']
                    self.context.objects.remove_object(self.grasped)
                    return True
                return False
        print(f'OBJECT: {object_name} DOES NOT EXISTS IN FIELD.')
        return False
    
    def release(self):
        if self.grasped:
            self.context.objects.add(self.grasped, self.pose, self.radius)
            self.grasped = None

    def exists(self):
        return True if self.position else False
