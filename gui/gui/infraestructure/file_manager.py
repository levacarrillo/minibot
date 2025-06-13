import os

class FileManager:
    def __init__(self):
        self.resources = os.path.join(os.path.dirname(__file__), '..', 'resources')

    def get_file_path(self, file_name):
        return os.path.join(self.resources, file_name)
