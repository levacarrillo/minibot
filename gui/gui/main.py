import rclpy
import threading
from gui.app.app import App
from gui.controllers.controller import Controller
from gui.infraestructure.ros import Ros


def main(args=None):
    rclpy.init(args=args)
    node = Ros()
    controller = Controller(node)
    app = App(controller)

    def ros_spin():
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = threading.Thread(target = ros_spin, daemon = True)
    ros_thread.start()
    app.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
