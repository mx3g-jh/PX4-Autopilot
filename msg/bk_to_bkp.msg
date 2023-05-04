# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#

uint64 timestamp		# time since system start (microseconds)

float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)

# Position in local NED frame
float32 x				# North position in NED earth-fixed frame, (metres)
float32 y				# East position in NED earth-fixed frame, (metres)
float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)

uint16 jump_mission_index

uint16 break_current_mission_index

bool back_to_break_point		# Terrain altitude estimate is valid
bool jump_mission_index_reach
bool break_from_mission_point		# True if this position is estimated through dead-reckoning

uint8 msg_type
uint8 feedback_msg_type

uint8 MSG_TYPE_NONE = 0
uint8 MSG_TYPE_JUMP_MODE = 1
uint8 MSG_TYPE_BREAK_MODE = 2
uint8 MSG_TYPE_FEEDBACK_CURRENT_STATE = 3
uint8 MSG_TYPE_FEEDBACK_BACK_BREAK_POINT_STATE = 4
uint8 MSG_TYPE_FEEDBACK_BACK_JUMP_POINT_STATE = 5

uint16 current_mission_index
bool current_mission_index_reach

# TOPICS bk_to_bkp bk_to_bkp_feedback
