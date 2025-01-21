import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # Depth 카메라 토픽 이름
            self.depth_callback,
            10
        )
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        # ROS 메시지를 OpenCV 이미지로 변환
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 특정 픽셀의 깊이 값 확인 (예: 중앙 픽셀)
        height, width = depth_image.shape
        center_depth = depth_image[height // 2, width // 2]

        self.get_logger().info(f"Center Pixel Depth: {center_depth} meters")

def main(args=None):
    rclpy.init(args=args)
    depth_reader = DepthReader()
    rclpy.spin(depth_reader)
    depth_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
