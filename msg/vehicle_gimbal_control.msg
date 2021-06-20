uint64 timestamp		# time since system start (microseconds)

int16[4]  quaternion  # drone's quaternion, multiplied by 10000
int16  hvel           # drone's horizontal velocity in cm/s unit
int16  hacc           # drone's horizontal acceleration in cm/s/s unit
int16  yaw_deg_desire # yaw desired angle, drone send to gimbal,precise 0.1 degree, multiply 0.1 to use
uint16 yaw_channel    # yaw channel value,value range 0-4096,follow point ahead mode is none;degree = -((channel_value-2048)*0.03dgree+45dgree); velocity=-(channel_value-2130)/41 or velocity=-(channel_value-1966)/41 there has a lsb deadline
uint16 pitch_channel  # pitch channel value,value range 0-4096,follow point ahead mode is none;degree = -((channel_value-2048)*0.03dgree+45dgree); velocity=-(channel_value-2130)/41 or velocity=-(channel_value-1966)/41 there has a lsb deadline
uint16 roll_channel   # roll channel value,value range 0-4096,follow point ahead mode is none;degree = -((channel_value-2048)*0.03dgree+45dgree); velocity=-(channel_value-2130)/41 or velocity=-(channel_value-1966)/41 there has a lsb deadline
uint16 yaw_mode       # yaw mode value, 410-820 is mode :follow point ahead;820-1229 is mode:follow point angle changeable, 1229-1638 is mode:follow point velocity  changeable,2048-2458 is mode:global angle changeable, 2867-3686 is mode: global velocity  changeable
uint16 pitch_mode     # pitch mode value, 410-820 is mode :follow point  ahead;820-1229 is mode:follow point angle changeable, 1229-1638 is mode:follow point velocity  changeable,2048-2458 is mode:global angle changeable, 2867-3686 is mode: global velocity  changeable
uint16 roll_mode      # roll mode value, 410-820 is mode :follow point  ahead;820-1229 is mode:follow point angle changeable, 1229-1638 is mode:follow point velocity  changeable,2048-2458 is mode:global angle changeable, 2867-3686 is mode: global velocity changeable
