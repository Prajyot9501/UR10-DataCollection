import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/praj/Desktop/robot_learning/data_collection/data_col_ws/install/ur10e_data_collector'
