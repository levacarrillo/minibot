import os


class FileManager:
    def __init__(self):
        self.resources = os.path.join(os.path.dirname(__file__), '..', 'resources')
        self.maps_path = os.path.join(os.path.dirname(__file__), '../../../../..', 'share', 'gui', 'maps')
        self.map_list = None
    
    def __del__(self):
        print('CLOSING RESOURCE...')

    def get_file_path(self, file_name):
        return os.path.join(self.resources, file_name)

    def get_environment_list(self):
        self.map_list = []
        for file in os.listdir(self.maps_path):
            if file.endswith('.wrl'):
                self.map_list.append(file.replace('.wrl', '').upper())
        return sorted(self.map_list)


    def get_map(self, map_name):
        map_file = open(os.path.join(self.maps_path, map_name.lower() + '.wrl'), 'r')
        return map_file

