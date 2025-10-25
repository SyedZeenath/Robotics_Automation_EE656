import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/master26/Robotics_Automation_EE656/Assignment_1_lab/install/rx200_moveit_control'
