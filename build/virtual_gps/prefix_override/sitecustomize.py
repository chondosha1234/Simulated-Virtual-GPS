import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/olivia-neights/Simulated-Virtual-GPS/install/virtual_gps'
