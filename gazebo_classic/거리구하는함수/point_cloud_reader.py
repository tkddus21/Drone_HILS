import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudROI(Node):
    def __init__(self):
        super().__init__('point_cloud_roi')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.cloud_callback,
            10
        )

    def cloud_callback(self, msg):
        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[0], point[1], point[2]
            # 중앙 영역의 ROI 조건: x, y, z 값이 특정 범위 내에 있을 경우만 추가
            if -0.1 < x < 0.1 and -0.1 < y < 0.1 and z > 0:
                points.append(z)

        if points:
            # 중앙 영역의 평균 거리 계산
            avg_distance = sum(points) / len(points)
            self.get_logger().info(f"Average Distance: {avg_distance:.2f} meters")
        else:
            self.get_logger().info("No points detected in ROI")

def main(args=None):
    rclpy.init(args=args)
    point_cloud_roi = PointCloudROI()
    rclpy.spin(point_cloud_roi)
    point_cloud_roi.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
