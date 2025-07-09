import rclpy
from tests.app.app import App

def main(args = None):
    rclpy.init(args = args)

    app = App()

    app.run()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
