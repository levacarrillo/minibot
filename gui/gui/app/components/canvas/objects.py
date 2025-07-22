class Objects:
    def __init__(self, context):
        self.context = context
        
        self.loaded = False
        self.object_list = context.get_object_list()
        context.set_objects(self)

    def plot(self):
        self.loaded = True
        self.context.canvas.delete('object')
        for obj in self.object_list:
            self.context.canvas.create_rectangle(obj['rectangle'],
                                                 fill = self.context.color['block'],
                                                 outline = self.context.color['block'],
                                                 tag = 'object')

            self.context.canvas.create_text(obj['x'], obj['y'], text = obj['name'],
                                            fill = self.context.color['block_text'],
                                            font = 'Calibri 8 bold', tag  = 'object')

    def remove_object(self, object_name_to_remove):
        for i, obj in enumerate(self.object_list):
            if obj['name'] == object_name_to_remove:
                del self.object_list[i]
                break
        self.plot()

    def add(self, name, position, robot_radius):
        obj = self.context.get_object_released(name, position, robot_radius)
        self.object_list.append(obj)
        self.plot()

    def exists(self):
        return self.loaded

    def delete(self):
        self.loaded = False
        self.context.canvas.delete('object')
