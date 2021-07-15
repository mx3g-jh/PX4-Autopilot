#add_definitions("-DYMAVLINK_GIMBAL_ENABLE") //no need ymavlink h920e

add_definitions("-DTAP_HAS_OFDM")
# FIXME: review and remove following obsolete codes if neccessary, by definition
# the idea of this macro is to provide an OFDM binding function similar to what's
# been used in zigbee, however it is now replaced by a physical button on OFDM
# module for binding purpose.

#add_definitions("-DTAP_ENABLE_OFDM_FLIP_BIND")

# YUNEEC_PRODUCT_ID = YUNEEC_PRODUCT_ID + 2
add_definitions("-DYUNEEC_PRODUCT_INCREMENT=2")
px4_add_board(
	PLATFORM nuttx
	VENDOR yuneec
	MODEL tap-v3
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	#IO px4_io-v2_default
	TESTING
	# UAVCAN_INTERFACES 2
	SERIAL_PORTS
		# GPS1:/dev/ttyS0
		#TEL1:/dev/ttyS1
		#TEL2:/dev/ttyS2
		#TEL4:/dev/ttyS3
	DRIVERS
		adc/board_adc
		#barometer # all available barometer drivers
		barometer/ms5611
		#barometer/mpc2520
		# batt_smbus
		# camera_capture
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		# distance_sensor/hc_sr04
		#dshot
		gps
		#heater
		#imu # all available imu drivers
		#imu/mpu6000 # legacy icm20602/icm20689 driver
		# irlock
		#lights/blinkm
		# lights/rgbled
		#lights/rgbled_ncp5623c
		#lights/rgbled_pwm
		#magnetometer # all available magnetometer drivers
		#magnetometer/ist8310
		magnetometer/hmc5883
		#magnetometer/ist8310
		# mkblctrl
		#optical_flow # all available optical flow drivers
		#osd
		# pca9685
		# power_monitor/ina226
		#protocol_splitter
		pwm_input
		pwm_out_sim
		pwm_out
		#px4io
		rc_input
		#roboclaw
		rpm
		#safety_button
		#tap_esc
		telemetry # all available telemetry drivers
		test_ppm
		tone_alarm
		# uavcan
		#yuneec_flow
	MODULES
		#attitude_estimator_q
		battery_status
		# camera_feedback
		commander
		dataman
		ekf2
		events
		land_detector
		# landing_target_estimator
		load_mon
		# local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		navigator
		rc_update
		sensors
		sih
		temperature_compensation
		vmount
	SYSTEMCMDS
		bl_update
		dmesg
		dumpfile
		esc_calib
		gpio
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver
		work_queue
	EXAMPLES
		#fake_magnetometer
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		##matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#uuv_example_app
		#work_item
	)
