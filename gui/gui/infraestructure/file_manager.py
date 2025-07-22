import os


class FileManager:
    def __init__(self):
        self.resources = os.path.join(os.path.dirname(__file__), '..', 'resources')
        self.maps_path = os.path.join(os.path.dirname(__file__), '../../../../..', 'share', 'gui', 'maps')
        self.objects_path = os.path.join(os.path.dirname(__file__), '../../../../..', 'share', 'gui', 'objects')
        self.map_file = None
        self.objects_file = None

    def __del__(self):
        self.map_file.close()
        self.objects_file.close()

    def get_path(self, file_name):
        return os.path.join(self.resources, file_name)
    
    def check_for_topological_map(self, file_name):
        ext = '.top'
        if os.path.exists(os.path.join(self.maps_path, file_name.lower() + ext)):
            return True
        else:
            return False

    def get_environment_list(self):
        map_list = []
        for file in os.listdir(self.maps_path):
            if file.endswith('.wrl'):
                map_list.append(file.replace('.wrl', '').upper())
        return sorted(map_list)

    def get_map(self, map_name, topological = False):
        ext = '.top' if topological else '.wrl'
        try:
            self.map_file = open(os.path.join(self.maps_path, map_name.lower() + ext), 'r')
        except FileNotFoundError:
            self.map_file = None
            print(f'ERROR: THE FILE {map_name.lower()}{ext} DOES NOT EXISTS.')
        except Exception as e:
            self.map_file = None
            print(f'AN UNEXPECTED ERROR HAS OCCURRED: {e}')
        finally:
            return self.map_file

    def get_objects_file(self):
        self.objects_file = open(os.path.join(self.objects_path, 'objects.txt'), 'r')
        return self.objects_file