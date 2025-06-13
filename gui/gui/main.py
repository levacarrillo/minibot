import rclpy
from gui.app.app import App
from gui.infraestructure.simulator_node import SimulatorNode


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    app = App()

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        app.after(100, ros_spin)

    app.after(100, ros_spin)
    app.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
