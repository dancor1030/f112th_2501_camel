import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/laxted/f112th_2501_camel/install/f112th_2501_control_teleop'