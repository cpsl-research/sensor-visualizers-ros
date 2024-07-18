import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray
from visualization_msgs.msg import Marker, MarkerArray



class SingleCamVisualizer(Node):
    pass


def main(args=None):
    rclpy.init(args=args)

    visualizer = SingleCamVisualizer()

    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
