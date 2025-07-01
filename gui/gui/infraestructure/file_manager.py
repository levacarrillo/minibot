import os


class FileManager:
    def __init__(self):
        self.resources = os.path.join(os.path.dirname(__file__), '..', 'resources')
        self.maps      = os.path.join(os.path.dirname(__file__), '../../../../..', 'share', 'gui', 'maps')

    def get_file_path(self, file_name):
        return os.path.join(self.resources, file_name)

    def get_environment_list(self):
        print(self.maps)
        map = []
        for file in os.listdir(self.maps):
            if file.endswith('.wrl'):
                map.append(file.replace('.wrl', '').upper())
        return sorted(map)


    def get_map(self, map_name):
        map = os.path.join(self.maps, map_name)
