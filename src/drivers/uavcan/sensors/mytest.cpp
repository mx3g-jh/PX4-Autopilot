/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "mytest.hpp"
#include <lib/ecl/geo/geo.h>
#include <px4_defines.h>

const char *const UavcanMyTestBridge::NAME = "mytest";

UavcanMyTestBridge::UavcanMyTestBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_mytest", ORB_ID(mytest)),
	ModuleParams(nullptr),
	_sub_mytest(node),
	_last_timestamp(0)
{
}

int UavcanMyTestBridge::init()
{
	int res = _sub_mytest.start(MyTestCbBinder(this, &UavcanMyTestBridge::mytest_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void
UavcanMyTestBridge::mytest_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::MyTest> &msg)
{
	mytest_s mytest{};

	mytest.timestamp = hrt_absolute_time();
	mytest.mytest_id = msg.getSrcNodeID().get();
	mytest.x = msg.x;

	publish(msg.getSrcNodeID().get(), &mytest);
}
