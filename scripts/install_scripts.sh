#!/bin/bash
#  ___________________________________________________
# |  _____                       _____ _ _       _    |
# | |  __ \                     |  __ (_) |     | |   |
# | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
# | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
# | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
# | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
# |                   __/ |                           |
# |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
# |___________________________________________________|
#
# This Script install Binaries to /usr/local/bin
#
# Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.


cd /usr/local/bin
ln -f -s $PENGUPILOT_PATH/blackbox/tools/filter_txt_col.py pp_filter_txt_col
ln -f -s $PENGUPILOT_PATH/blackbox/tools/msgpack_to_txt.py pp_msgpack_to_txt
ln -f -s $PENGUPILOT_PATH/blackbox/tools/compare_msgpack.py pp_compare_msgpack
ln -f -s $PENGUPILOT_PATH/opcd/tools/opcd_shell.sh pp_opcd_shell
ln -f -s $PENGUPILOT_PATH/autopilot/tools/replay.py pp_replay
ln -f -s $PENGUPILOT_PATH/autopilot/tools/shell/pilot_shell.sh pp_pilot_shell
ln -f -s $PENGUPILOT_PATH/autopilot/tools/logger.py pp_logger
ln -f -s $PENGUPILOT_PATH/scripts/clear_pidfiles.sh pp_clear_pidfiles
ln -f -s $PENGUPILOT_PATH/svctrl/svctrl.py pp_svctrl
ln -f -s $PENGUPILOT_PATH/gps/tools/gps_debug.py pp_gps_debug
ln -f -s $PENGUPILOT_PATH/icarus/tools/icarus_shell.sh pp_icarus_shell
ln -f -s $PENGUPILOT_PATH/autopilot/tools/acc_mag_cal/acc_mag_dump pp_acc_mag_dump
ln -f -s $PENGUPILOT_PATH/autopilot/tools/acc_mag_cal/cal_from_file.sh pp_cal_from_file
ln -f -s $PENGUPILOT_PATH/autopilot/tools/acc_mag_cal/calibrate.py pp_calibrate

