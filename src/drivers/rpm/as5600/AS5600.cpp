/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "AS5600.hpp"

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

int AS5600::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("addr: %" PRId8 ", pool: %" PRId32 ", reset: %" PRId32 ", magenet: %" PRId32, get_device_address(),
		  _param_as5600_pool.get(),
		  _param_as5600_reset.get(),
		  _param_as5600_magnet.get());

	// initCounter();
	ScheduleOnInterval(_param_as5600_pool.get());
	_rpm_pub.advertise();

	return PX4_OK;
}

int AS5600::probe()
{
	// setRegister(0x00, 0b00100000);
	setZPosition(0xFFF);
	setMPosition(0xFFF);
	setMaxAngle(0xFFF);
	setOutputMode(AS5600_OUTMODE_PWM);
	setPowerMode(AS5600_POWERMODE_NOMINAL);
	setPWMFrequency(AS5600_PWM_920);
	setHysteresis(AS5600_HYST_OFF);
	setSlowFilter(AS5600_SLOW_FILT_16X);
	setFastFilter(AS5600_FAST_FILT_LSB24);
	setWatchDog(AS5600_WATCHDOG_OFF);

	uint16_t ss = getMaxAngle();
	PX4_WARN("----------------- %d", ss);
	// uint8_t s = readRegister(0x00);
	// PX4_DEBUG("status register: %" PRId8 " fail_count: %" PRId8, s, _tranfer_fail_count);

	// // AS5600 contains free RAM registers
	// // This checks if I2C devices contains this RAM memory registers
	// // Some values are stored into this registers
	// // then it is vertified that the entered values fit.
	// setRegister(0x04, 10);
	// setRegister(0x05, 10);
	// setRegister(0x06, 10);
	// setRegister(0x0c, 5);
	// setRegister(0x0d, 5);
	// setRegister(0x0e, 5);
	// uint32_t tmp{0};

	// // check values stored in free RAM parts
	// tmp += readRegister(0x04);
	// tmp += readRegister(0x05);
	// tmp += readRegister(0x06);
	// tmp += readRegister(0x0c);
	// tmp += readRegister(0x0d);
	// tmp += readRegister(0x0e);

	// if (tmp != 45) {
	// 	return PX4_ERROR;
	// }

	return PX4_OK;
}

void AS5600::initCounter()
{
	// set counter mode
	_tranfer_fail_count = 0;
	// setRegister(0x00, 0b00100000);
	resetCounter();

}

uint32_t AS5600::getCounter()
{
	// Counter value is stored in 9 words
	// in 3 register as BCD value
	uint8_t a = readRegister(0x01);
	uint8_t b = readRegister(0x02);
	uint8_t c = readRegister(0x03);

	return uint32_t(
		       hiWord(a) * 1u + loWord(a) * 10u
		       + hiWord(b) * 100u + loWord(b) * 1000u
		       + hiWord(c) * 10000u + loWord(c) * 1000000u);
}

void AS5600::resetCounter()
{
	_last_measurement_time = hrt_absolute_time();
	// setRegister(0x01, 0x00);
	// setRegister(0x02, 0x00);
	// setRegister(0x03, 0x00);
	_count = 0;
	_last_reset_time = _last_measurement_time;
	_reset_count ++;
}

// Configure AS5600 driver into counting mode
void AS5600::setRegister(uint8_t reg, uint8_t value)
{
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (reg == 0x00) {
		_last_config_register_content = value;
	}

	if (PX4_OK != ret) {
		PX4_DEBUG("setRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
	}
}

uint8_t AS5600::readRegister(uint8_t reg)
{
	uint8_t rcv{};
	int ret = transfer(&reg, 1, &rcv, 1);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
	}

	return rcv;
}

void AS5600::RunImpl()
{
	// read sensor and compute frequency
	// int32_t oldcount = _count;

	// int32_t diffTime = hrt_elapsed_time(&_last_measurement_time);

	// // check if delay is enought
	// if (diffTime < _param_as5600_pool.get() / 2) {
	// 	PX4_ERR("as5600 loop called too early");
	// 	return;
	// }

	// _count = getCounter();
	// _last_measurement_time = hrt_absolute_time();

	// int32_t diffCount = _count - oldcount;

	// // check if there is enought space in counter
	// // Otherwise, reset counter
	// if (diffCount > (999999 - oldcount)) {
	// 	PX4_ERR("as5600 RPM register overflow");
	// 	resetCounter();
	// 	return;
	// }

	// //check if device failed or reset
	// uint8_t s = readRegister(0x00);

	// if (_tranfer_fail_count > 0 || s != 0b00100000 || diffCount < 0) {
	// 	PX4_ERR("as5600 RPM sensor restart: fail count %d, status: %d, diffCount: %ld",
	// 		_tranfer_fail_count, s, diffCount);
	// 	initCounter();
	// 	return;
	// }

	// // Calculate RPM and accuracy estimation
	// float indicated_rpm = (((float)diffCount / _param_as5600_magnet.get()) / ((float)diffTime / 1000000.f)) * 60.f;
	// float estimated_accurancy = 1 / (float)_param_as5600_magnet.get() / ((float)diffTime / 1000000) * 60.f;

	// // publish data to uorb
	// rpm_s msg{};
	// msg.indicated_frequency_rpm = indicated_rpm;
	// msg.estimated_accurancy_rpm = estimated_accurancy;
	// msg.timestamp = hrt_absolute_time();
	// _rpm_pub.publish(msg);

	// //check counter range
	// if (_param_as5600_reset.get() < diffCount + (int)_count) {
	// 	resetCounter();
	// }
	uint16_t pos = readReg2(AS5600_ANGLE);
	float speed = getAngularSpeed(AS5600_MODE_DEGREES, true);
	PX4_WARN("pos: %d speed %3.2f", pos, (double)speed);
	sleep(1);
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %" PRId32 " us", _param_as5600_pool.get());
	PX4_INFO("Last reset %.3fs ago, Count of resets: %d", (double)(hrt_absolute_time() - _last_reset_time) / 1000000.0,
		 _reset_count);
	PX4_INFO("Last count %ld", _count);
}

uint8_t AS5600::getAddress()
{
	return _address;
}

uint8_t AS5600::getZMCO()
{
	uint8_t value = readReg(AS5600_ZMCO);
	return value;
}


bool AS5600::setZPosition(uint16_t value)
{
	if (value > 0x0FFF) { return false; }

	writeReg2(AS5600_ZPOS, value);
	return true;
}


uint16_t AS5600::getZPosition()
{
	uint16_t value = readReg2(AS5600_ZPOS) & 0x0FFF;
	return value;
}


bool AS5600::setMPosition(uint16_t value)
{
	if (value > 0x0FFF) { return false; }

	writeReg2(AS5600_MPOS, value);
	return true;
}


uint16_t AS5600::getMPosition()
{
	uint16_t value = readReg2(AS5600_MPOS) & 0x0FFF;
	return value;
}


bool AS5600::setMaxAngle(uint16_t value)
{
	if (value > 0x0FFF) { return false; }

	writeReg2(AS5600_MANG, value);
	return true;
}


uint16_t AS5600::getMaxAngle()
{
	uint16_t value = readReg2(AS5600_MANG) & 0x0FFF;
	return value;
}


/////////////////////////////////////////////////////////
//
//  CONFIGURATION
//
bool AS5600::setConfigure(uint16_t value)
{
	if (value > 0x3FFF) { return false; }

	writeReg2(AS5600_CONF, value);
	return true;
}


uint16_t AS5600::getConfigure()
{
	uint16_t value = readReg2(AS5600_CONF) & 0x3FFF;
	return value;
}


//  details configure
bool AS5600::setPowerMode(uint8_t powerMode)
{
	if (powerMode > 3) { return false; }

	uint8_t value = readReg(AS5600_CONF + 1);
	value &= ~AS5600_CONF_POWER_MODE;
	value |= powerMode;
	writeReg(AS5600_CONF + 1, value);
	return true;
}


uint8_t AS5600::getPowerMode()
{
	return readReg(AS5600_CONF + 1) & 0x03;
}


bool AS5600::setHysteresis(uint8_t hysteresis)
{
	if (hysteresis > 3) { return false; }

	uint8_t value = readReg(AS5600_CONF + 1);
	value &= ~AS5600_CONF_HYSTERESIS;
	value |= (hysteresis << 2);
	writeReg(AS5600_CONF + 1, value);
	return true;
}


uint8_t AS5600::getHysteresis()
{
	return (readReg(AS5600_CONF + 1) >> 2) & 0x03;
}


bool AS5600::setOutputMode(uint8_t outputMode)
{
	if (outputMode > 2) { return false; }

	uint8_t value = readReg(AS5600_CONF + 1);
	value &= ~AS5600_CONF_OUTPUT_MODE;
	value |= (outputMode << 4);
	writeReg(AS5600_CONF + 1, value);
	return true;
}


uint8_t AS5600::getOutputMode()
{
	return (readReg(AS5600_CONF + 1) >> 4) & 0x03;
}


bool AS5600::setPWMFrequency(uint8_t pwmFreq)
{
	if (pwmFreq > 3) { return false; }

	uint8_t value = readReg(AS5600_CONF + 1);
	value &= ~AS5600_CONF_PWM_FREQUENCY;
	value |= (pwmFreq << 6);
	writeReg(AS5600_CONF + 1, value);
	return true;
}


uint8_t AS5600::getPWMFrequency()
{
	return (readReg(AS5600_CONF + 1) >> 6) & 0x03;
}


bool AS5600::setSlowFilter(uint8_t mask)
{
	if (mask > 3) { return false; }

	uint8_t value = readReg(AS5600_CONF);
	value &= ~AS5600_CONF_SLOW_FILTER;
	value |= mask;
	writeReg(AS5600_CONF, value);
	return true;
}


uint8_t AS5600::getSlowFilter()
{
	return readReg(AS5600_CONF) & 0x03;
}


bool AS5600::setFastFilter(uint8_t mask)
{
	if (mask > 7) { return false; }

	uint8_t value = readReg(AS5600_CONF);
	value &= ~AS5600_CONF_FAST_FILTER;
	value |= (mask << 2);
	writeReg(AS5600_CONF, value);
	return true;
}


uint8_t AS5600::getFastFilter()
{
	return (readReg(AS5600_CONF) >> 2) & 0x07;
}


bool AS5600::setWatchDog(uint8_t mask)
{
	if (mask > 1) { return false; }

	uint8_t value = readReg(AS5600_CONF);
	value &= ~AS5600_CONF_WATCH_DOG;
	value |= (mask << 5);
	writeReg(AS5600_CONF, value);
	return true;
}


uint8_t AS5600::getWatchDog()
{
	return (readReg(AS5600_CONF) >> 5) & 0x01;
}


/////////////////////////////////////////////////////////
//
//  OUTPUT REGISTERS
//
uint16_t AS5600::rawAngle()
{
	int16_t value = readReg2(AS5600_RAW_ANGLE);

	if (_offset > 0) { value += _offset; }

	value &= 0x0FFF;

	if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
	    (_direction == AS5600_COUNTERCLOCK_WISE)) {
		value = (4096 - value) & 0x0FFF;
	}

	return value;
}


uint16_t AS5600::readAngle()
{
	uint16_t value = readReg2(AS5600_ANGLE);

	if (_error != AS5600_OK) {
		return _lastReadAngle;
	}

	if (_offset > 0) { value += _offset; }

	value &= 0x0FFF;

	if ((_directionPin == AS5600_SW_DIRECTION_PIN) &&
	    (_direction == AS5600_COUNTERCLOCK_WISE)) {
		//  mask needed for value == 0.
		value = (4096 - value) & 0x0FFF;
	}

	_lastReadAngle = value;
	return value;
}


bool AS5600::setOffset(float degrees)
{
	//  expect loss of precision.
	if (abs(degrees) > 36000) { return false; }

	bool neg = (degrees < 0);

	if (neg) { degrees = -degrees; }

	uint16_t offset = round(degrees * AS5600_DEGREES_TO_RAW);
	offset &= 0x0FFF;

	if (neg) { offset = (4096 - offset) & 0x0FFF; }

	_offset = offset;
	return true;
}


float AS5600::getOffset()
{
	return _offset * AS5600_RAW_TO_DEGREES;
}


bool AS5600::increaseOffset(float degrees)
{
	//  add offset to existing offset in degrees.
	return setOffset((_offset * AS5600_RAW_TO_DEGREES) + degrees);
}


/////////////////////////////////////////////////////////
//
//  STATUS REGISTERS
//
uint8_t AS5600::readStatus()
{
	uint8_t value = readReg(AS5600_STATUS);
	return value;
}


uint8_t AS5600::readAGC()
{
	uint8_t value = readReg(AS5600_AGC);
	return value;
}


uint16_t AS5600::readMagnitude()
{
	uint16_t value = readReg2(AS5600_MAGNITUDE) & 0x0FFF;
	return value;
}


bool AS5600::detectMagnet()
{
	return (readStatus() & AS5600_MAGNET_DETECT) > 1;
}


bool AS5600::magnetTooStrong()
{
	return (readStatus() & AS5600_MAGNET_HIGH) > 1;
}


bool AS5600::magnetTooWeak()
{
	return (readStatus() & AS5600_MAGNET_LOW) > 1;
}


/////////////////////////////////////////////////////////
//
//  BURN COMMANDS
//
//  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
//
//  void AS5600::burnAngle()
//  {
//    writeReg(AS5600_BURN, x0x80);
//  }
//
//
//  See https://github.com/RobTillaart/AS5600/issues/38
//  void AS5600::burnSetting()
//  {
//    writeReg(AS5600_BURN, 0x40);
//    delay(5);
//    writeReg(AS5600_BURN, 0x01);
//    writeReg(AS5600_BURN, 0x11);
//    writeReg(AS5600_BURN, 0x10);
//    delay(5);
//  }


float AS5600::getAngularSpeed(uint8_t mode, bool update)
{
	if (update) {
		_lastReadAngle = readAngle();

		if (_error != AS5600_OK) {
			return NAN;
		}
	}

	//  default behaviour
	uint32_t now     = hrt_absolute_time();
	int      angle   = _lastReadAngle;
	uint32_t deltaT  = now - _lastMeasurement;
	int      deltaA  = angle - _lastAngle;

	//  assumption is that there is no more than 180° rotation
	//  between two consecutive measurements.
	//  => at least two measurements per rotation (preferred 4).
	if (deltaA >  2048) { deltaA -= 4096; }

	else if (deltaA < -2048) { deltaA += 4096; }

	float speed = (deltaA * 1e6) / deltaT;

	//  remember last time & angle
	_lastMeasurement = now;
	_lastAngle       = angle;

	//  return radians, RPM or degrees.
	if (mode == AS5600_MODE_RADIANS) {
		return speed * AS5600_RAW_TO_RADIANS;
	}

	if (mode == AS5600_MODE_RPM) {
		return speed * AS5600_RAW_TO_RPM;
	}

	//  default return degrees
	return speed * AS5600_RAW_TO_DEGREES;
}


/////////////////////////////////////////////////////////
//
//  POSITION cumulative
//
int32_t AS5600::getCumulativePosition(bool update)
{
	if (update) {
		_lastReadAngle = readAngle();

		if (_error != AS5600_OK) {
			return _position;  //  last known position.
		}
	}

	int16_t value = _lastReadAngle;

	//  whole rotation CW?
	//  less than half a circle
	if ((_lastPosition > 2048) && (value < (_lastPosition - 2048))) {
		_position = _position + 4096 - _lastPosition + value;
	}

	//  whole rotation CCW?
	//  less than half a circle
	else if ((value > 2048) && (_lastPosition < (value - 2048))) {
		_position = _position - 4096 - _lastPosition + value;

	} else {
		_position = _position - _lastPosition + value;
	}

	_lastPosition = value;

	return _position;
}


int32_t AS5600::getRevolutions()
{
	int32_t p = _position >> 12;  //  divide by 4096

	if (p < 0) { p++; }  //  correct negative values, See #65

	return p;
}


int32_t AS5600::resetPosition(int32_t position)
{
	int32_t old = _position;
	_position = position;
	return old;
}


int32_t AS5600::resetCumulativePosition(int32_t position)
{
	_lastPosition = readAngle();
	int32_t old = _position;
	_position = position;
	return old;
}


int AS5600::lastError()
{
	int value = _error;
	_error = AS5600_OK;
	return value;
}


/////////////////////////////////////////////////////////
//
//  PROTECTED AS5600
//
uint8_t AS5600::readReg(uint8_t reg)
{
	uint8_t rcv{};
	_error = AS5600_OK;
	int ret = transfer(&reg, 1, &rcv, 1);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
		_error = AS5600_ERROR_I2C_READ_1;
		return 0;
		_tranfer_fail_count++;
	}

	return rcv;
}


uint16_t AS5600::readReg2(uint8_t reg)
{
	uint8_t rcv[2];
	uint16_t data{};
	_error = AS5600_OK;
	int ret = transfer(&reg, 1, &rcv[0], 2);

	if (PX4_OK != ret) {
		PX4_DEBUG("readRegister : i2c::transfer returned %d", ret);
		_error = AS5600_ERROR_I2C_READ_3;
		return 0;
		_tranfer_fail_count++;
	}

	data = rcv[0] << 8 | rcv[1];
	return data;
}


uint8_t AS5600::writeReg(uint8_t reg, uint8_t value)
{
	_error = AS5600_OK;
	uint8_t buff[2];
	buff[0] = reg;
	buff[1] = value;
	int ret = transfer(buff, 2, nullptr, 0);

	if (reg == 0x00) {
		_last_config_register_content = value;
	}

	if (PX4_OK != ret) {
		PX4_DEBUG("setRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
		_error = AS5600_ERROR_I2C_WRITE_0;
	}

	return _error;
}


uint8_t AS5600::writeReg2(uint8_t reg, uint16_t value)
{
	_error = AS5600_OK;
	uint8_t buff[3];
	buff[0] = reg;
	buff[1] = value >> 8;
	buff[2] = value & 0xFF;
	int ret = transfer(buff, 3, nullptr, 0);

	if (reg == 0x00) {
		_last_config_register_content = value;
	}

	if (PX4_OK != ret) {
		PX4_DEBUG("setRegister : i2c::transfer returned %d", ret);
		_tranfer_fail_count++;
		_error = AS5600_ERROR_I2C_WRITE_0;
	}

	return _error;

}
