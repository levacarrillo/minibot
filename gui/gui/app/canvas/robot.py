class Robot:
    def __init__(self, parent_panel):
        self.color   = parent_panel.color
        self.canvas  = parent_panel.canvas
        service      = parent_panel.service
        entry_radio  = parent_panel.robot_section.entry_radio
        self.radio   =  service.m_to_pixels(parent_panel.scale_x, parent_panel.size_x, entry_radio.get())
        self.canva   = False


    def plot(self, pose_x, pose_y):
        if self.canva:
            self.canvas.delete(self.canva)
        self.canva = self.canvas.create_oval(
            pose_x - self.radio,
            pose_y - self.radio,
            pose_x + self.radio,
            pose_y + self.radio,
            outline = self.color['robot'],
            fill = self.color['robot'],
            width=1
        )

    def delete(self):
        self.canvas.delete(self.canva)