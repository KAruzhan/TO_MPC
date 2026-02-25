import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chri/CHRI_2025/UR5_pyBullet+acados/src/install/my_package'
