import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/atwalm/cs_classes/robotics/ros-lab-2/install/py_pubsub'
