import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/Storage/Hackathons/IIITM/raged-ros2/install/control'
