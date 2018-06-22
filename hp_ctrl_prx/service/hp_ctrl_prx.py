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
  
 Horizontal Position Control Setpoints Proxy

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. """


from scl import SCL_Proxy
from misc import daemonize
from pp_prio import PP_PRIO_5
from scheduler import sched_set_prio


def main(name):
   sched_set_prio(PP_PRIO_5)
   n_proxy = SCL_Proxy('hp_ctrl_spp_n', 'pull', 'hp_ctrl_sp_n', 'pub')
   e_proxy = SCL_Proxy('hp_ctrl_spp_e', 'pull', 'hp_ctrl_sp_e', 'pub')


daemonize('hp_ctrl_prx', main)
