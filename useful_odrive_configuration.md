## handy refence 
https://docs.odriverobotics.com/v/latest/manual/control.html#torque-control


#calibrate with the motor detached!
#also be in passthough mode for raw behaviour!

#to have full torque at low rpm 
odrv0.axis0.controller.enable_torque_mode_vel_limit = False


#do enable external forces discrepany 
odrv0.axis0.controller.config.spinout_electrical_power_treshold = 30
odrv0.axis0.controller.config.spinout_mechanical_power_treshold = -30

odrv0.config.dc_max_positive_current = 15
odrv0.config.dc_max_negative_current = -6
odrv0 = .Config.dc_bus_overvoltage_trip_level = 50
odrv0.Config.dc_bus_undervoltage_trip_level = 30

odrv0.config.max_regen_current = 0.1

# to make the regerative current circuit work i had to disable otp (over temp protection) switch -> off because we don't have a thermistor jet but we can add one in the near future
