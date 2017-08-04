
from scl import SCL_Reader, scl_get_socket
from gps_msgpack import ALT
from time import sleep
from msgpack import loads
import time

acc = SCL_Reader('acc', 'sub')
acc_raw = SCL_Reader('acc_raw', 'sub')
gyro = SCL_Reader('gyro', 'sub')
gyro_raw = SCL_Reader('gyro_raw', 'sub')

rs_ctrl_sp_p = SCL_Reader('rs_ctrl_sp_p', 'sub')
rs_ctrl_sp_r = SCL_Reader('rs_ctrl_sp_r', 'sub')
rs_ctrl_sp_y = SCL_Reader('rs_ctrl_sp_y', 'sub')

rp_ctrl_sp_p = SCL_Reader('rp_ctrl_sp_p', 'sub')
rp_ctrl_sp_r = SCL_Reader('rp_ctrl_sp_r', 'sub')
rp_ctrl_sp_y = SCL_Reader('rp_ctrl_sp_y', 'sub')

torques = SCL_Reader('torques', 'sub')
torques_p = SCL_Reader('torques_p', 'sub')

voltage = SCL_Reader('voltage', 'sub')
current = SCL_Reader('current', 'sub')

forces = SCL_Reader('forces', 'sub')

pos_est = SCL_Reader('pos_speed_est_neu', 'sub')
orientation = SCL_Reader('orientation', 'sub')
#orientation_socket = scl_get_socket('orientation', 'sub')

rc = SCL_Reader('rc', 'sub')
int_en = SCL_Reader('int_en', 'sub')

start = int(round(time.time() * 1000))

#f = open("simu1.log", "w")
sleep(1)
while True:
    
    ms = int(round(time.time() * 1000))
    ms = ms - start
    
    print ms, acc.data, acc_raw.data, gyro.data, gyro_raw.data, rs_ctrl_sp_p.data, rs_ctrl_sp_r.data, rs_ctrl_sp_y.data, torques.data, torques_p.data, forces.data, voltage.data, current.data, orientation.data, rp_ctrl_sp_p.data, rp_ctrl_sp_r.data, rp_ctrl_sp_y.data, int_en.data, rc.data, pos_est.data
    sleep(0.01)
