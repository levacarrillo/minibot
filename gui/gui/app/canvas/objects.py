class Objects:
    def __init__(self, context):
        self.color = context.color
        self.canvas = context.canvas 
        self.controller = context.controller

        self.in_field = False
        self.list = self.controller.load_objects()
        context.set_objects(self)

    def plot(self):
        self.in_field = True

        self.canvas.delete('object')
        for object in self.list:
            self.canvas.create_rectangle(object['x'] - 10,
                                         object['y'] - 10,
                                         object['x'] + 10,
                                         object['y'] + 10,
                                         fill = self.color['block'],
                                         outline = self.color['block'],
                                         tag = 'object')

            self.canvas.create_text(object['x'],
                                    object['y'], 
                                    fill = self.color['block_text'],
                                    font = 'Calibri 8 bold',
                                    text = object['name'],
                                    tag  = 'object')

    def remove_object(self, object_name_to_remove):
        for i, obj in enumerate(self.list):
            if obj['name'] == object_name_to_remove:
                del self.list[i]
                break
        self.plot()

    def add(self, name, position, robot_radius):
        obj = self.controller.get_object_released(name, position, robot_radius)
        self.list.append(obj)
        self.plot()

    def delete(self):
        self.in_field = False
        self.canvas.delete('object')
