#!/usr/bin/env python
"""
  ___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 Semi Auto Flight Logic

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from sticks import *
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

def mot_en_cb(gesture):
   if gesture[0]: # start
      api.mot_en(True)
      print 'start'
   if gesture[1]: # stop
      api.mot_en(False)
      print 'stop'

orientation = SCL_Reader('orientation', 'sub', [0.0])
pse = SCL_Reader('pos_speed_est_neu', 'sub')
flying = SCL_Reader('flying', 'sub', 0)
rc_socket = scl_get_socket('rc', 'sub')
gps = SCL_Reader('gps', 'sub', [0])
ultra_raw = SCL_Reader('ultra_raw', 'sub')
ap_ctrl = scl_get_socket('ap_ctrl', 'req')
ap_state = SCL_Reader('ap_status', 'sub')
#col_av_hs = SCL_Reader('col_av_hs', 'sub')
col_av_pr = SCL_Reader('col_av_pr', 'sub', [0.0, 0.0])
col_av_lim_sticks = SCL_Reader('col_av_lim_sticks', 'sub', [1.0, 1.0, 1.0, 1.0])
#SCL_Reader('rc_gestures', 'sub', callback = mot_en_cb)

init(OPCD_Subscriber())
sleep(1)
api = CtrlAPI()
api.set_thrust_max(55.0)
try:
   killed = False
   mc = False
   pos_locked = False
   alt_locked = False
   mode_prev = None

   while True:
      rc_data = rc_socket.recv()
      state = ap_state.data      
      if rc_data[0]:
         pitch_stick, roll_stick, yaw_stick, gas_stick, on_switch, mode_switch = rc_data[1:7]
         mot_en_state = on_switch > 0.5
         api.mot_en(on_switch > 0.5)
         pr_sticks = [pitch_stick, roll_stick]
         
	 #set the mode of flight to a desired one
         mode = channel_to_mode(mode_switch)
         #if mode == 'gps' and fix(gps.data) < 2:
         #   mode = 'acc'
         if mode_prev != mode:
            print 'new mode:', mode
            once = 1
            init_vp = False
            task_vp = ''
            if mode == 'acc':
                print 'limit. mode'
            if mode == 'gps':
                print 'only CA mode'
            if mode == 'gyro':
                print 'manual mode'

         if mode == 'gyro':
            #full manual mode
            api.set_thrust(12.0 * (gas_stick + 1.0))
            api.set_ys(0.6 * yaw_stick)
              
            vals = map(pitch_roll_angle_func, pr_sticks)
            api.set_rp([-vals[0], vals[1]])
         
            #if killed == False:
                #kill autopilot
            #    ap_ctrl.send('kill')
            #    killed = True
 
         if mode == 'gpsddss':
            
            api.set_thrust(10.0* (gas_stick + 1.0))
            api.set_ys(0.6 * yaw_stick)
              
            min_val = 0.001

            if col_av_pr.data[0] > min_val or col_av_pr.data[1] > min_val:
                pr_col = [col_av_pr.data[0]+pr_sticks[0], col_av_pr.data[1]+pr_sticks[1]]            
                vals = map(pitch_roll_angle_func, pr_col)
                api.set_rp([-vals[0], vals[1]])
            else:
                 if not mot_en_state or mode_prev != mode:
                     pos_locked = None
                 #set horizontal speed using sticks, otherwise - hold pos
                 if pitch_roll_in_deadzone(pitch_stick, roll_stick):
                    if not pos_locked:
                        pos_locked = pse.data[N_POS], pse.data[E_POS]
                        print 'locking:', pos_locked
                        api.set_hp(pos_locked)
                 else:
                    pos_locked = None
                    scale = 3.0
                    v_local = [scale * pitch_stick, scale * roll_stick]
                    v_global = vec2_rot(v_local, orientation.data[0])
                    api.set_hs(v_global)


         if mode == 'accddss':
            
            api.set_thrust(10.0 * (gas_stick + 1.0))
            api.set_ys(0.6 * yaw_stick)

            if pr_sticks[0] > 0.01:
                pr_sticks[0] = pr_sticks[0] * col_av_lim_sticks.data[0]
            if pr_sticks[0] < -0.01:
                pr_sticks[0] = pr_sticks[0] * col_av_lim_sticks.data[1]

            if pr_sticks[1] > 0.01:
                pr_sticks[1] = pr_sticks[1] * col_av_lim_sticks.data[2]
            if pr_sticks[1] < -0.01:
                pr_sticks[1] = pr_sticks[1] * col_av_lim_sticks.data[3]
            
            pr_col = [col_av_pr.data[0]+pr_sticks[0], col_av_pr.data[1]+pr_sticks[1]]            
            vals = map(pitch_roll_angle_func, pr_col)
            api.set_rp([-vals[0], vals[1]])

         if mode == 'acc': #or mode == 'acc':
             #position hold/control, vertical control and takeoff/land via: autopilot
#             if state == 'standing' and yaw_stick >= 0.7:
#                 try:
#                    ap_ctrl.send('takeoff')
#                    rep = ap_ctrl.recv()
#                    if not rep[0]:
#                      print rep[1]
#                 except Exception, e:
#                    print e
#                    pass
#
#             if state == 'hovering' and yaw_stick <= -0.7:
#                 try:
#                    ap_ctrl.send('land')
#                    rep = ap_ctrl.recv()
#                    if not rep[0]:
#                      print rep[1]
#                 except Exception, e:
#                    print e
#                    pass
             
           # if (abs(gas_stick) > 0.1):
           #     api.set_vs(gas_stick * 2.0)
           #     hold_baro = None
           # else:
           #     if not pse.data:
           #         api.set_vs(gas_stick * 2.0)
           # if not hold_baro:
           #    hold_baro = pse.data[0]
           #    print 'setting', hold_baro
           #    api.set_vp(hold_baro, 'ultra')

            if ((gas_stick) > 0.5):
                api.set_vs(0.5)
                print "Climbing up"
            elif ((gas_stick) < -0.5):
                api.set_vs(-0.5)
                print "Climbing down"
            else:
                api.set_vs(0.0)

            vals = map(pitch_roll_angle_func, pr_sticks)
            api.set_rp([-vals[0], vals[1]])

#             if True:
#                 #set the vertical position
#                 vp = 2.5 * (gas_stick + 1.0)
#                 if not alt_locked:
#                     alt_locked = vp
#                     if vp >= 0.05: #1.0
#                        api.set_vp(vp, 'baro_rel')
#                     else:
#                        vp = 0.0
#                        api.set_vp(vp, 'baro_rel')
#                     print 'Desired altitude set: ' + str(vp)
#                 else:
#                     if abs(vp - alt_locked) >= 0.1:
#                         alt_locked = False
#
#                 api.set_ys(0.6 * yaw_stick)
#              
#                 vals = map(pitch_roll_angle_func, pr_sticks)
#                 api.set_rp([-vals[0], vals[1]])
         

#
#                 #set horizontal speed using sticks, otherwise - hold pos
#                 if pitch_roll_in_deadzone(pitch_stick, roll_stick):
#                    if not pos_locked:
#                        pos_locked = pse.data[N_POS], pse.data[E_POS]
#                        print 'locking:', pos_locked
#                        api.set_hp(pos_locked)
#                 else:
#                    pos_locked = None
#                    scale = 3.0
#                    v_local = [scale * pitch_stick, scale * roll_stick]
#                    v_global = vec2_rot(v_local, orientation.data[0])
#                    api.set_hs(v_global)
#
#                 if (col_av_hs.data[0] > 0 and col_av_hs.data[1] > 0) and mode == 'acc':
#                    api.set_hs(col_av_hs.data)

         #if mode == 'acc':
         
         mode_prev = mode
      else:
         api.mot_en(False)
except Exception, e:
   print e

api.mot_en(False)
sleep(0.5)
