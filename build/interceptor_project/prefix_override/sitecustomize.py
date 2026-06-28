import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/divyansh/Desktop/interceptor_project/install/interceptor_project'
