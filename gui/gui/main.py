import rclpy
from gui.app.gui_adapter import GUIAdapter
from gui.app.simulator_node import SimulatorNode
# from gui.domain.service import Service


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    # service = Service()
    # gui = GUIAdapter(service)
    gui = GUIAdapter()

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        gui.root.after(100, ros_spin)

    gui.root.after(100, ros_spin)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
