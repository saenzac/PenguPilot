from scl import SCL_Reader, scl_get_socket
from gps_msgpack import ALT
from time import sleep
from msgpack import loads
import time

#acc_raw = SCL_Reader('acc_raw', 'sub')
#gyro_raw = SCL_Reader('gyro_raw', 'sub')
#gyro = SCL_Reader('gyro','sub')
int_en = SCL_Reader('int_en','sub')
mot_state = SCL_Reader('mot_state','sub')
voltage = SCL_Reader('voltage','sub')
pwms = SCL_Reader('pwms','sub')
#rs_ctrl_sp_p = SCL_Reader('rs_ctrl_sp_p','sub')
#rs_ctrl_sp_r = SCL_Reader('rs_ctrl_sp_r','sub')
#rs_ctrl_sp_y = SCL_Reader('rs_ctrl_sp_y','sub')
#thrust_maxp = SCL_Reader('thrust_maxp', 'sub')
#thrust = SCL_Reader('thrust','sub')
forces = SCL_Reader('forces','sub')
#mot_en = SCL_Reader('mot_en','sub')
#johnny_err = SCL_Reader('johnny_err', 'sub')
logger = SCL_Reader('log_data_pub','sub')

start = int(round(time.time() * 1000))

sleep(1)
while True:
    
    #ms = int(round(time.time() * 1000))
    #ms = ms - start
    
    #print gyro.data, thrust_maxp.data, thrust.data, forces.data, mot_state.data
#    print 'mot_state',mot_state.data
#    print 'pwms', pwms.data
#    print logger.data
#    print 'forces', forces.data
    print 'int_en', int_en.data
    sleep(0.4)

