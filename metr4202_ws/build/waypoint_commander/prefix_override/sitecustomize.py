import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/brog/metr4202/metr4202_ws/install/waypoint_commander'
