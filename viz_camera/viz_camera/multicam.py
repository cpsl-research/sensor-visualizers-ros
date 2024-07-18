import rclpy
from rclpy.node import Node


class MultiCamVisualizer(Node):
    pass



def main(args=None):
    rclpy.init(args=args)

    visualizer = MultiCamVisualizer()

    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()