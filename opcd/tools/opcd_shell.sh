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
# OPCD Shell Loader Script
#
# Copyright (C) 2014 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau
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


s=`realpath "$0"`
dir=`dirname $s`
cd $dir
PYTHONSTARTUP=opcd_shell.py python
