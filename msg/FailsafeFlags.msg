# Input flags for the failsafe state machine set by the arming & health checks.
#
# Flags must be named such that false == no failure (e.g. _invalid, _unhealthy, _lost)
# The flag comments are used as label for the failsafe state machine simulation

uint64 timestamp				# time since system start (microseconds)

# Per-mode requirements
uint32 mode_req_angular_velocity
uint32 mode_req_attitude
uint32 mode_req_local_alt
uint32 mode_req_local_position
uint32 mode_req_local_position_relaxed
uint32 mode_req_global_position
uint32 mode_req_mission
uint32 mode_req_offboard_signal
uint32 mode_req_home_position
uint32 mode_req_prevent_arming    # if set, cannot arm while in this mode
uint32 mode_req_other             # other requirements, not covered above (for external modes)


# Mode requirements
bool angular_velocity_invalid         # Angular velocity invalid
bool attitude_invalid                 # Attitude invalid
bool local_altitude_invalid           # Local altitude invalid
bool local_position_invalid           # Local position estimate invalid
bool local_position_invalid_relaxed   # Local position with reduced accuracy requirements invalid (e.g. flying with optical flow)
bool local_velocity_invalid           # Local velocity estimate invalid
bool global_position_invalid          # Global position estimate invalid
bool gps_position_invalid             # GPS position invalid
bool auto_mission_missing             # No mission available
bool offboard_control_signal_lost     # Offboard signal lost
bool home_position_invalid            # No home position available

# Control links
bool manual_control_signal_lost       # Manual control (RC) signal lost
bool gcs_connection_lost              # GCS connection lost

# Battery
uint8 battery_warning                 # Battery warning level
bool battery_low_remaining_time       # Low battery based on remaining flight time
bool battery_unhealthy                # Battery unhealthy

# Other
bool primary_geofence_breached        # Primary Geofence breached
bool mission_failure                  # Mission failure
bool vtol_transition_failure          # VTOL transition failed (quadchute)
bool wind_limit_exceeded              # Wind limit exceeded
bool flight_time_limit_exceeded       # Maximum flight time exceeded

# Failure detector
bool fd_critical_failure              # Critical failure (attitude/altitude limit exceeded, or external ATS)
bool fd_esc_arming_failure            # ESC failed to arm
bool fd_imbalanced_prop               # Imbalanced propeller detected
bool fd_motor_failure                 # Motor failure

