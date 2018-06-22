pp_svctrl --start log_proxy
pp_svctrl --start simulation
pp_svctrl --start plantsimpl
pp_svctrl --start rp_ctrl_prx
#pp_svctrl --start rp_ctrl
pp_svctrl --start rs_ctrl_prx
#pp_svctrl --start rs_ctrl
pp_svctrl --start mixer_prx
pp_svctrl --start bcks_ctrl
pp_svctrl --start mixer
pp_svctrl --start motors
echo 'calling semi_auto_logic.py'
python flight_logic/semi_auto_logic.py
./scripts/stopservices.sh
