/****************************************************************************
*
*   Copyright (c) 2016-2017 PX4 Development Team. All rights reserved.
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
 * @file input_rc_yuneec.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "input_rc.h"

namespace vmount
{


/**
 ** class InputRCYuneec
 * RC input class using manual_control_setpoint topic, specific for Yuneec.
 * It expects:
 * - pitch control on aux2
 * - yaw control on aux1
 * - pitch (tilt) mode on aux3
 * - yaw (pan) mode on aux4
 */
class InputRCYuneec : public InputRC
{
public:

	InputRCYuneec(float stick_deadzone);
	virtual ~InputRCYuneec();


protected:
	virtual bool _read_control_data_from_subscription(ControlData &control_data, bool already_active);

private:
	bool _first_time = true;
	float _last_set_aux_values[2] = {};
	float _stick_deadzone = 0.0f;
	uint8_t _last_gimbal_yaw_mode = 0;
	uint8_t _last_gimbal_pitch_mode = 0;
};


} /* namespace vmount */
