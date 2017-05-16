from scl import SCL_Reader, scl_get_socket
from gps_msgpack import ALT
from time import sleep
from msgpack import loads
import time

logger = SCL_Reader('log_data_pub','sub')


while True:
    
    print logger.data
    sleep(0.01)
