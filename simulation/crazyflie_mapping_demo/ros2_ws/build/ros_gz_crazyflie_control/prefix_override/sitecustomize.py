import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/weixuan/crazyflie_mapping_demo/ros2_ws/install/ros_gz_crazyflie_control'
