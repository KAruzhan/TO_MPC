import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chri/CHRI_2025/Time_optimal_baseline/install/mppi_controller'
