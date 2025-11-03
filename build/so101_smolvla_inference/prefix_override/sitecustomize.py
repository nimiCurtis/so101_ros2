import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anton/ros2_ws/src/so101_ros2/install/so101_smolvla_inference'
