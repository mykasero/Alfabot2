import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

IMAGE_TOPIC = "image_raw/compressed"


class testnode(Node):
    def __init__(self):

        super().__init__("QR_detector")

        self.get_logger().info("Node init ...")
        self.compressed_image_sub = self.create_subscription(CompressedImage, IMAGE_TOPIC,
                                                             self.compressed_image_sub_callback, 10)

        # Internal status
        self.last_compressed_image = CompressedImage()  # last CompressedImage received from the camera
        self.get_logger().info("Node init complete.")

    def compressed_image_sub_callback(self, compressed_image_msg):
        self.get_logger().info("Got an Image")


def main(args=None):
    """
    Main function of the QR_detector node.
    """
    np.set_printoptions(suppress=True)  # Prevent numpy scientific notation
    rclpy.init(args=args)

    Testnode = testnode()
    rclpy.spin(Testnode)
    Testnode.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()