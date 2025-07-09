class Objects:
    def __init__(self, context):
        self.canvas = context.canvas 
        self.controller = context.controller

        self.objects_data = []
        self.grasp_id = None
        context.set_objects(self)

    def plot(self):
        r1 = self.controller.m_to_pixels('0.2') - 10
        r2 = self.controller.m_to_pixels('0.1') - 10
        r3 = self.controller.m_to_pixels('0.2') + 10
        r4 = self.controller.m_to_pixels('0.1') + 10
        rectangle = self.canvas.create_rectangle(r1, r2, r3, r4,fill="#9FFF3D",outline="#9FFF3D", tag = 'object')
        text = self.canvas.create_text(r1, r2, fill="#9E4124",font="Calibri 10 bold",text='Block_A', tag = 'object')
        obj = {
            'name': 'Block_A',
            'x': 0.2,
            'y': 0.1,
            'rectangle': rectangle,
            'text': None
        }
        self.objects_data.append(obj)

    def delete(self):
        for object in self.objects_data:
            self.canvas.delete('object')
