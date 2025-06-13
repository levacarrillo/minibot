class Service():
    def __init__(self):
        self.simulation_runnning = False

    def get_edge(self, size, scale, line_per_meters):
        return size / (scale * line_per_meters)

    def pixels_to_m(self, scale, size, point):
        return str(scale * point / size)[:4]

    def m_to_pixels(self, scale, size, length):
        return (float(length) * size) / scale