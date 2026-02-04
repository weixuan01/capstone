import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/weixuan/capstone/simulation/crazyflie_mapping_demo/ros2_ws/install/crazyflie_ros2_multiranger_simple_mapper'
