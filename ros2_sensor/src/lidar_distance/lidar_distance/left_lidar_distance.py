import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class LeftLidarDistance(Node):
    def __init__(self):
        super().__init__('left_lidar_distance')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/iris/left_lidar_plugin/out',
            self.lidar_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def lidar_callback(self, msg):
        if len(msg.data) < 16:
            return

        data = msg.data[:16]
        x = struct.unpack('f', data[0:4])[0]  # x 좌표 값 추출
        self.get_logger().info(f'📏 Left Distance: {x:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    node = LeftLidarDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
