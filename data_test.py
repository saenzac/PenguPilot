from scl import SCL_Reader, scl_get_socket
from gps_msgpack import ALT
from time import sleep
from msgpack import loads
import time

acc_raw = SCL_Reader('acc_raw', 'sub')
gyro_raw = SCL_Reader('gyro_raw', 'sub')
gyro = SCL_Reader('gyro','sub')
int_en = SCL_Reader('int_en','sub')
mot_state = SCL_Reader('mot_state','sub')
rs_ctrl_sp_p = SCL_Reader('rs_ctrl_sp_p','sub')
rs_ctrl_sp_r = SCL_Reader('rs_ctrl_sp_r','sub')
rs_ctrl_sp_y = SCL_Reader('rs_ctrl_sp_y','sub')
#johnny_err = SCL_Reader('johnny_err', 'sub')


start = int(round(time.time() * 1000))

sleep(1)
while True:
    
    ms = int(round(time.time() * 1000))
    ms = ms - start
    
    print gyro.data, rs_ctrl_sp_p.data, rs_ctrl_sp_r.data, rs_ctrl_sp_y.data, int_en.data, mot_state.data
    sleep(0.01)

