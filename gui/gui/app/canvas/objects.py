class Objects:
    def __init__(self, context):
        self.color = context.color
        self.canvas = context.canvas 
        self.controller = context.controller

        self.grasp_id = None
        context.set_objects(self)

    def plot(self):
        object_list = self.controller.load_objects()
        for object in object_list:
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

    def delete(self):
        self.canvas.delete('object')
