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
 * @file output_serial.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "output.h"

#include <uORB/uORB.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#include <uORB/topics/vehicle_gimbal_control.h>
#include <../../mavlink/include/mavlink//v2.0/checksum.h>

namespace vmount
{


/**
 ** class OutputSerial
 *  Output to a serial device
 */
class OutputSerial : public OutputBase
{
public:
	OutputSerial(const OutputConfig &output_config);
	virtual ~OutputSerial();

	virtual int initialize();

	virtual int update(const ControlData *control_data);

	virtual void print_status();

private:
#ifdef YMAVLINK_GIMBAL_ENABLE
	hrt_abstime _last_heartbeat_timestamp = 0;
	uint8_t _seq = 0;
#endif
	int _serial_fd = -1;
	uORB::Subscription _sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _vehicle_lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	estimator_sensor_bias_s	_sensors_bias;
	vehicle_local_position_s _vlpos;


#ifndef YMAVLINK_GIMBAL_ENABLE
	uORB::Publication <vehicle_gimbal_control_s>	_gimbal_control_pub{ORB_ID(vehicle_gimbal_control)};

#else
	void _handle_send_heartbeat();
	bool _send_packet(uint8_t *payload, int payload_len, int msg_id);
#endif

};


} /* namespace vmount */
