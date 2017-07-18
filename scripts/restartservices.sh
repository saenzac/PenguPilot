#pp_svctrl --start i2c_sensors
#pp_svctrl --start ahrs
pp_svctrl --restart simulation
pp_svctrl --restart plantsimpl
pp_svctrl --restart rp_ctrl_prx
pp_svctrl --restart rp_ctrl
pp_svctrl --restart rs_ctrl_prx
pp_svctrl --restart rs_ctrl
pp_svctrl --restart mixer_prx
pp_svctrl --restart mixer
pp_svctrl --restart motors
python flight_logic/semi_auto_logic.py
##pp_svctrl --start simulation
#johnnysaenz@johnny-mb-pro:~/workspace/PenguPilot$ pp_svctrl --start rp_ctrl_prx
#dependency resolution order: ['log_proxy', 'opcd', 'gpsp', 'i2c_sensors', 'motors', 'arduino', 'mag_adc_cal', 'geomag', 'gyro_cal', 'acc_cal', 'mixer', 'cmc', 'mixer_prx', 'rs_ctrl', 'ahrs', 'rs_ctrl_prx', 'rp_ctrl', 'rp_ctrl_prx']
#note: log_proxy is already running
#note: opcd is already running
#starting gpsp ... [OK]
#starting i2c_sensors ... [OK]
#starting motors ... [OK]
#starting arduino ... [OK]
#starting mag_adc_cal ... [OK]
#starting geomag ... [OK]
#starting gyro_cal ... [OK]
#starting acc_cal ... [OK]
#starting mixer ... [OK]
#starting cmc ... [OK]
#starting mixer_prx ... [OK]
#starting rs_ctrl ... [OK]
#starting ahrs ... [OK]
#starting rs_ctrl_prx ... [OK]
#starting rp_ctrl ... [OK]
#starting rp_ctrl_prx ... [OK]
