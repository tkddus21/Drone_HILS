import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/krristudent/HITL/ros2_sensor/install/lidar_distance'
