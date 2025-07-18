class Objects:
    def __init__(self, context):
        self.context = context
        self.in_field = False
        self.list = context.load_objects()
        context.set_objects(self)

    def plot(self):
        self.in_field = True

        self.context.canvas.delete('object')
        for object in self.list:
            self.context.canvas.create_rectangle(object['x'] - 10,
                                         object['y'] - 10,
                                         object['x'] + 10,
                                         object['y'] + 10,
                                         fill = self.context.color['block'],
                                         outline = self.context.color['block'],
                                         tag = 'object')

            self.context.canvas.create_text(object['x'],
                                    object['y'], 
                                    fill = self.context.color['block_text'],
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
        # obj = self.controller.get_object_released(name, position, robot_radius)
        # self.list.append(obj)
        self.plot()

    def delete(self):
        self.in_field = False
        self.context.canvas.delete('object')
