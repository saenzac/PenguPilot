from scl import scl_get_socket, SCL_Reader
from time import sleep
from geomath import vec2_rot
from ctrl_api import CtrlAPI
from gps_msgpack import fix
from opcd_interface import OPCD_Subscriber
from pos_speed_est_neu import N_POS, E_POS
from pid_ctrl import PID_Ctrl
import math
import time

sleep(1)
api = CtrlAPI()
try:
   while True:
     print 'send off simu'
     api.off_simu()
except Exception, e:
   print e

