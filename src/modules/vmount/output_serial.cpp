/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file output_serial.cpp
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output_serial.h"


#include <px4_platform_common/defines.h>
#include <matrix/math.hpp>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

namespace vmount
{

#pragma pack(push, 1)

// NOTE: Don't change the order of the elements inside the struct!!! (TODO: fix)
struct packet_header_t {
	uint8_t stx;     	//< protocol magic marker
	uint8_t len;       	//< Length of payload
	uint8_t seq;       	//< Sequence of packet
	uint8_t senderSysID;    //< ID of the message sender system
	uint8_t senderCompID;   //< ID of the message sender component
	uint8_t targetSysID;  	//< ID of the message target system
	uint8_t targetCompID; 	//< ID of the message target component
	uint8_t msg;     	//< ID of message in payload
};

// NOTE: Don't change the order of the elements inside the struct!!! (TODO: fix)
struct gimbal_control_t {
	int16_t quaternion[4]; 	/*< drone's quaternion, multiplied by 10000 */
	int16_t hvel; 		/*< drone's horizontal velocity in cm/s unit */
	int16_t hacc; 		/*< drone's horizontal acceleration in cm/s/s unit */
	int16_t yaw_deg_desire; /*< yaw desired angle, drone send to gimbal. Unit: 0.1 degrees. */

	/* Control setpoint:
	 * 0-4096 in follow point ahead mode is none;
	 * degree = -((channel_value-2048)*0.03dgree+45dgree)
	 * velocity=-(channel_value-2130)/41 or velocity=-(channel_value-1966)/41 there has a lsb deadline.
	 */
	uint16_t yaw_channel;
	uint16_t pitch_channel;
	uint16_t roll_channel;

	/* Control modes:
	 * 410-820:   "follow point ahead" 		- look forward
	 * 820-1229:  "follow point angle changeable" 	- angle setpoint in body-frame
	 * 1229-1638: "follow point velocity changeable"- angular rate setpoint in body-frame
	 * 2048-2458: "global angle changeable 		- angle setpoint" in global frame
	 * 2867-3686: "global velocity changeable" 	- angular rate setpoint in global frame
	 */
	uint16_t yaw_mode;
	uint16_t pitch_mode;
	uint16_t roll_mode;
};

struct heartbeat_t {
	uint16_t firmware_version;
	uint16_t software_version;
	uint8_t protocol_version;
};

#pragma pack(pop)

OutputSerial::OutputSerial(const OutputConfig &output_config)
	: OutputBase(output_config)
{
}

int OutputSerial::initialize()
{
	int ret = OutputBase::initialize();

	if (ret) {
		return ret;
	}

	PX4_INFO("opening serial device %s, baud=%i", _config.device, _config.baudrate);
	_serial_fd = open(_config.device, O_RDWR | O_NOCTTY);

	if (_serial_fd < 0) {
		PX4_ERR("Failed to open serial device (%i)", errno);
		return -errno;
	}

	struct termios uart_config;

	int termios_state;

	/* Fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;


	int speed = B57600;

	switch (_config.baudrate) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;
	}

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("SET BAUD %s\n", _config.device);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR SET CONF %s", _config.device);
		return -1;
	}

	return ret;
}

OutputSerial::~OutputSerial()
{
	if (_serial_fd >= 0) {
		close(_serial_fd);
	}
}

#ifdef YMAVLINK_GIMBAL_ENABLE
void OutputSerial::_handle_send_heartbeat()
{
	if (_last_heartbeat_timestamp == 0 || hrt_elapsed_time(&_last_heartbeat_timestamp) > 1e6) {

		heartbeat_t heartbeat;
		memset(&heartbeat, 0, sizeof(heartbeat));
		heartbeat.protocol_version = 1;
		heartbeat.software_version = 0;

		_send_packet((uint8_t *)&heartbeat, sizeof(heartbeat), 0);

		_last_heartbeat_timestamp = hrt_absolute_time();
	}
}
#endif

#ifdef YMAVLINK_GIMBAL_ENABLE
bool OutputSerial::_send_packet(uint8_t *payload, int payload_len, int msg_id)
{
	packet_header_t packet_header;
	packet_header.stx = 254;
	packet_header.len = payload_len;
	packet_header.seq = _seq++;
	packet_header.senderCompID = 0;
	packet_header.senderSysID = 1;
	packet_header.targetCompID = 0;
	packet_header.targetSysID = 2;
	packet_header.msg = msg_id;

	uint16_t checksum;
	crc_init(&checksum);
	uint8_t *p = (uint8_t *)&packet_header;
	++p;

	for (size_t i = 1; i < sizeof(packet_header); ++i) {
		crc_accumulate(*p++, &checksum);
	}

	p = payload;

	for (int i = 0; i < payload_len; ++i) {
		crc_accumulate(*p++, &checksum);
	}

	crc_accumulate(0, &checksum);

	if (write(_serial_fd, &packet_header, sizeof(packet_header)) != sizeof(packet_header)) {
		return false;
	}

	if (write(_serial_fd, payload, payload_len) != payload_len) {
		return false;
	}

	if (write(_serial_fd, &checksum, 2) != 2) {
		return false;
	}

	return true;
}
#endif

int OutputSerial::update(const ControlData *control_data)
{
	if (control_data) {
		//got new command
		_set_angle_setpoints(control_data);

		//we don't directly use the _angle_speeds
		if (_cur_control_data->type == ControlData::Type::Angle) {
			for (int i = 0; i < 3; ++i) {
				if (_cur_control_data->type_data.angle.is_speed[i]) {
					_angle_setpoints[i] = _angle_speeds[i];
				}
			}
		}
	}

	if (_cur_control_data->type == ControlData::Type::Neutral) { //no input yet... just return
		return 0;
	}

	_handle_position_update();

	hrt_abstime t = hrt_absolute_time();


	//Get yaw speed setpoint.
	static vehicle_attitude_setpoint_s _att_sp = {};
	if (_att_sp_sub.updated()) {
		_att_sp_sub.copy(&_att_sp);
	}

	//Get arming state.
	static actuator_armed_s actuator_armed = {};
	if (_actuator_armed_sub.updated()) {
		_actuator_armed_sub.copy(&actuator_armed);
	}

	vehicle_attitude_s vehicle_attitude;
	vehicle_angular_velocity_s angular_velocity{};
	if(_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&vehicle_attitude);
	}

	if(_angular_velocity_sub.updated()){
		_angular_velocity_sub.copy(&angular_velocity);
	}

	// Strangely, at the beginning, some spurious attitude updates are published,
	// therefore we need to actually check if the norm is sane.
	// The right fix would be if the attitude was only published when it's sane
	// and we would not run this at all if no attitude is published, however, that would
	// require bigger changes to vmount, so this check should be a good enough
	// workaround for now.
	const float quaternion_norm_squared =
		vehicle_attitude.q[0] * vehicle_attitude.q[0] +
		vehicle_attitude.q[1] * vehicle_attitude.q[1] +
		vehicle_attitude.q[2] * vehicle_attitude.q[2] +
		vehicle_attitude.q[3] * vehicle_attitude.q[3];

	if (quaternion_norm_squared < 0.9f * 0.9f) {
		// Don't send gimbal_controls when the attitude is not initialized yet, otherwise
		// the gimbal initializes incorrectly.
		return 0;
	}

#ifndef  YMAVLINK_GIMBAL_ENABLE
	vehicle_gimbal_control_s gimbal_control {};
#else
	gimbal_control_t gimbal_control;
#endif
	memset(&gimbal_control, 0, sizeof(gimbal_control));
	gimbal_control.quaternion[0] = 10000.f * vehicle_attitude.q[0]; //w
	gimbal_control.quaternion[1] = 10000.f * vehicle_attitude.q[1]; //x
	gimbal_control.quaternion[2] = 10000.f * vehicle_attitude.q[2]; //y
	gimbal_control.quaternion[3] = 10000.f * vehicle_attitude.q[3]; //z

	// Send the drone spining rate to gimbal as yaw feedward control, Let it follow better.
	if (actuator_armed.armed == true) {
		gimbal_control.yaw_deg_desire = _att_sp.yaw_sp_move_rate * M_RAD_TO_DEG_F * 6.f;

	} else {
		gimbal_control.yaw_deg_desire = angular_velocity.xyz[2] * M_RAD_TO_DEG_F * 6.f;
	}
	if(_sensor_bias_sub.updated()){
		_sensor_bias_sub.copy(&_sensors_bias);
	}
	if(_vehicle_lpos_sub.updated()){
		_vehicle_lpos_sub.copy(&_vlpos);
	}
	gimbal_control.hvel = 100.f * sqrtf(_vlpos.vx * _vlpos.vx +
					    _vlpos.vy * _vlpos.vy);
	gimbal_control.hacc = 100.f * sqrtf(_sensors_bias.accel_bias[0] * _sensors_bias.accel_bias[0] +
					    _sensors_bias.accel_bias[1] * _sensors_bias.accel_bias[1]);

	// roll
	float roll_angle = _angle_setpoints[0] * M_RAD_TO_DEG_F;

	if (roll_angle < -45.f) {
		roll_angle = -45.f;

	} else if (roll_angle > 45.f) {
		roll_angle = 45.f;
	}

	gimbal_control.roll_channel = 2048.f - roll_angle / 0.035f;
	gimbal_control.roll_mode = 2048;

	// pitch
	float pitch_angle = _angle_setpoints[1] * M_RAD_TO_DEG_F;

	if (_cur_control_data->type == ControlData::Type::Angle
	    && _cur_control_data->type_data.angle.is_speed[1]) { //speed mode
		if (_cur_control_data->stabilize_axis[1]) {
			gimbal_control.pitch_mode = 3000;

		} else {
			gimbal_control.pitch_mode = 1300;
		}

		gimbal_control.pitch_channel = 2048.f + pitch_angle * 10.f;

	} else { //angle mode
		if (_cur_control_data->stabilize_axis[1]) {
			gimbal_control.pitch_mode = 2100;

		} else {
			gimbal_control.pitch_mode = 830;
		}

		gimbal_control.pitch_channel = 2048.f - (pitch_angle + 45.f) / 0.035f;
	}

	// yaw
	float yaw_angle = _angle_setpoints[2] * M_RAD_TO_DEG_F;

	if (_cur_control_data->type == ControlData::Type::Angle
	    && _cur_control_data->type_data.angle.is_speed[2]) { //speed mode
		if (_cur_control_data->stabilize_axis[2]) {
			gimbal_control.yaw_mode = 3000;

		} else {
			gimbal_control.yaw_mode = 1300;
		}

		gimbal_control.yaw_channel = 2048.f + yaw_angle * 10.f;

	} else { //angle mode
		if (_cur_control_data->stabilize_axis[2]) {
			gimbal_control.yaw_mode = 2100;

		} else {
			gimbal_control.yaw_mode = 830;
		}

		gimbal_control.yaw_channel = 2048.f - yaw_angle / 0.09f;
	}

#ifndef  YMAVLINK_GIMBAL_ENABLE


		/* publish it */
		_gimbal_control_pub.publish(gimbal_control);



#else
	_send_packet((uint8_t *)&gimbal_control, sizeof(gimbal_control), 1);
	_handle_send_heartbeat();
#endif

	_last_update = t;

	return 0;
}

void OutputSerial::print_status()
{
	PX4_INFO("Output: Serial (%s)", _config.device);
}

} /* namespace vmount */
