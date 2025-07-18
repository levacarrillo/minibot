import rclpy
import threading
from gui.app.app import App
from gui.infraestructure.ros import Ros
from gui.infraestructure.file_manager import FileManager
from gui.domain.service import Service


def main(args = None):
    rclpy.init(args = args)
    node = Ros()
    service = Service()
    file_manager = FileManager()
    app = App(service, node, file_manager)

    def ros_spin():
        while True:
            rclpy.spin_once(node, timeout_sec = 0.1)

    ros_thread = threading.Thread(target = ros_spin, daemon = True)
    ros_thread.start()
    app.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
