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
  
 Horizontal Speed Control (using GPS)

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from pid_ctrl import PID_Ctrl
from scl import scl_get_socket, SCL_Reader
from opcd_interface import OPCD_Subscriber
from misc import daemonize
from geomath import deg2rad, sym_limit, vec2_rot
from pp_prio import PP_PRIO_3
from scheduler import sched_set_prio
from math import sin, cos
from pos_speed_est_neu import N_SPEED, E_SPEED
from mot_state import STOPPED, RUNNING


def main(name):
   sched_set_prio(PP_PRIO_3)
   opcd = OPCD_Subscriber()
   orientation = SCL_Reader('orientation', 'sub')
   ms = SCL_Reader('mot_state', 'sub', STOPPED)
   sp_n = SCL_Reader(name + '_sp_n', 'sub', 0.0)
   sp_e = SCL_Reader(name + '_sp_e', 'sub', 0.0)
   sp = [sp_n, sp_e]
   err = scl_get_socket(name + '_err', 'pub')
   p_oe = SCL_Reader(name + '_p_oe', 'pull', 1)
   r_oe = SCL_Reader(name + '_r_oe', 'pull', 1)
   pitch_socket = scl_get_socket('rp_ctrl_spp_p', 'push')
   roll_socket = scl_get_socket('rp_ctrl_spp_r', 'push')
   ctrls = [PID_Ctrl(), PID_Ctrl()]
   pos_speed_est = scl_get_socket('pos_speed_est_neu', 'sub')
   while True:
      est = pos_speed_est.recv()
      ne_speed = [est[N_SPEED], est[E_SPEED]]

      ne_ctrl = [0.0, 0.0]
      try:
         for c in range(2):
            ctrls[c].p = opcd[name + '.p']
            ctrls[c].i = opcd[name + '.i']
            ctrls[c].max_sum_err = opcd[name + '.max_sum_err']
            if ms.data != RUNNING:
               ctrls[c].reset()
            ne_ctrl[c] = ctrls[c].control(sp[c].data, ne_speed[c])
         err.send([ctrls[0].err, ctrls[1].err])
      except:
         pass

      pr_ctrl = [0.0, 0.0]
      try:
         pr_ctrl = vec2_rot(ne_ctrl, -orientation.data[0])
      except:
         pass
      
      angles_max = deg2rad(opcd[name + '.angles_max'])
      if p_oe.data:
         pitch_socket.send(sym_limit(pr_ctrl[0], angles_max))
      if r_oe.data:
         roll_socket.send(sym_limit(-pr_ctrl[1], angles_max))

daemonize('hs_ctrl', main)
