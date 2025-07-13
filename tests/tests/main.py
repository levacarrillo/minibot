import rclpy
import threading
from tests.app.app import App
from tests.infraestructure.ros import Ros
from tests.domain.service import Service


def main(args = None):
    rclpy.init(args = args)
    node = Ros()
    service = Service()
    app = App(service, node)

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
