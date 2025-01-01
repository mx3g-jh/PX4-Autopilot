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

/**
 * @file AS5600.hpp
 *
 * @author ThunderFly s.r.o., Vít Hanousek <hanousekvit@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace/TFRPM01
 *
 * Driver for Main Rotor frequency sensor using AS5600 I2C counter.
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/rpm.h>
#include <drivers/drv_hrt.h>

/* Configuration Constants */
#define AS5600_BASEADDR_DEFAULT             0x36

//  default addresses
const uint8_t AS5600_DEFAULT_ADDRESS    = 0x36;
const uint8_t AS5600L_DEFAULT_ADDRESS   = 0x40;
const uint8_t AS5600_SW_DIRECTION_PIN   = 255;

//  setDirection
const uint8_t AS5600_CLOCK_WISE         = 0;  //  LOW
const uint8_t AS5600_COUNTERCLOCK_WISE  = 1;  //  HIGH

//  0.087890625;
const float   AS5600_RAW_TO_DEGREES     = 360.0 / 4096;
const float   AS5600_DEGREES_TO_RAW     = 4096 / 360.0;
//  0.00153398078788564122971808758949;
const float   AS5600_RAW_TO_RADIANS     = M_PI * 2.0 / 4096;
//  4.06901041666666e-6
const float   AS5600_RAW_TO_RPM         = 60.0 / 4096;

//  getAngularSpeed
const uint8_t AS5600_MODE_DEGREES       = 0;
const uint8_t AS5600_MODE_RADIANS       = 1;
const uint8_t AS5600_MODE_RPM           = 2;


//  ERROR CODES
const int     AS5600_OK                 = 0;
const int     AS5600_ERROR_I2C_READ_0   = -100;
const int     AS5600_ERROR_I2C_READ_1   = -101;
const int     AS5600_ERROR_I2C_READ_2   = -102;
const int     AS5600_ERROR_I2C_READ_3   = -103;
const int     AS5600_ERROR_I2C_WRITE_0  = -200;
const int     AS5600_ERROR_I2C_WRITE_1  = -201;


//  CONFIGURE CONSTANTS
//  check datasheet for details

//  setOutputMode
const uint8_t AS5600_OUTMODE_ANALOG_100 = 0;
const uint8_t AS5600_OUTMODE_ANALOG_90  = 1;
const uint8_t AS5600_OUTMODE_PWM        = 2;

//  setPowerMode
const uint8_t AS5600_POWERMODE_NOMINAL  = 0;
const uint8_t AS5600_POWERMODE_LOW1     = 1;
const uint8_t AS5600_POWERMODE_LOW2     = 2;
const uint8_t AS5600_POWERMODE_LOW3     = 3;

//  setPWMFrequency
const uint8_t AS5600_PWM_115            = 0;
const uint8_t AS5600_PWM_230            = 1;
const uint8_t AS5600_PWM_460            = 2;
const uint8_t AS5600_PWM_920            = 3;

//  setHysteresis
const uint8_t AS5600_HYST_OFF           = 0;
const uint8_t AS5600_HYST_LSB1          = 1;
const uint8_t AS5600_HYST_LSB2          = 2;
const uint8_t AS5600_HYST_LSB3          = 3;

//  setSlowFilter
const uint8_t AS5600_SLOW_FILT_16X      = 0;
const uint8_t AS5600_SLOW_FILT_8X       = 1;
const uint8_t AS5600_SLOW_FILT_4X       = 2;
const uint8_t AS5600_SLOW_FILT_2X       = 3;

//  setFastFilter
const uint8_t AS5600_FAST_FILT_NONE     = 0;
const uint8_t AS5600_FAST_FILT_LSB6     = 1;
const uint8_t AS5600_FAST_FILT_LSB7     = 2;
const uint8_t AS5600_FAST_FILT_LSB9     = 3;
const uint8_t AS5600_FAST_FILT_LSB18    = 4;
const uint8_t AS5600_FAST_FILT_LSB21    = 5;
const uint8_t AS5600_FAST_FILT_LSB24    = 6;
const uint8_t AS5600_FAST_FILT_LSB10    = 7;

//  setWatchDog
const uint8_t AS5600_WATCHDOG_OFF       = 0;
const uint8_t AS5600_WATCHDOG_ON        = 1;

//  CONFIGURATION REGISTERS
const uint8_t AS5600_ZMCO = 0x00;
const uint8_t AS5600_ZPOS = 0x01;   //  + 0x02
const uint8_t AS5600_MPOS = 0x03;   //  + 0x04
const uint8_t AS5600_MANG = 0x05;   //  + 0x06
const uint8_t AS5600_CONF = 0x07;   //  + 0x08

//  CONFIGURATION BIT MASKS - byte level
const uint8_t AS5600_CONF_POWER_MODE    = 0x03;
const uint8_t AS5600_CONF_HYSTERESIS    = 0x0C;
const uint8_t AS5600_CONF_OUTPUT_MODE   = 0x30;
const uint8_t AS5600_CONF_PWM_FREQUENCY = 0xC0;
const uint8_t AS5600_CONF_SLOW_FILTER   = 0x03;
const uint8_t AS5600_CONF_FAST_FILTER   = 0x1C;
const uint8_t AS5600_CONF_WATCH_DOG     = 0x20;


//  UNKNOWN REGISTERS 0x09-0x0A

//  OUTPUT REGISTERS
const uint8_t AS5600_RAW_ANGLE = 0x0C;   //  + 0x0D
const uint8_t AS5600_ANGLE     = 0x0E;   //  + 0x0F

// I2C_ADDRESS REGISTERS (AS5600L)
const uint8_t AS5600_I2CADDR   = 0x20;
const uint8_t AS5600_I2CUPDT   = 0x21;

//  STATUS REGISTERS
const uint8_t AS5600_STATUS    = 0x0B;
const uint8_t AS5600_AGC       = 0x1A;
const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C
const uint8_t AS5600_BURN      = 0xFF;

//  STATUS BITS
const uint8_t AS5600_MAGNET_HIGH   = 0x08;
const uint8_t AS5600_MAGNET_LOW    = 0x10;
const uint8_t AS5600_MAGNET_DETECT = 0x20;

class AS5600 : public device::I2C, public ModuleParams, public I2CSPIDriver<AS5600>
{
public:
	AS5600(const I2CSPIDriverConfig &config);
	~AS5600() override = default;

	static void print_usage();

	void		RunImpl();

	int    init() override;
	void   print_status() override;

private:

	uint8_t  _address         = AS5600_DEFAULT_ADDRESS;
	uint8_t  _directionPin    = 255;
	uint8_t  _direction       = AS5600_CLOCK_WISE;
	int      _error           = AS5600_OK;

	//  for getAngularSpeed()
	uint32_t _lastMeasurement = 0;
	int16_t  _lastAngle       = 0;
	int16_t  _lastReadAngle   = 0;
	//  for readAngle() and rawAngle()
	uint16_t _offset          = 0;


	//  EXPERIMENTAL
	//  cumulative position counter
	//  works only if the sensor is read often enough.
	int32_t  _position        = 0;
	int16_t  _lastPosition    = 0;

	int  probe() override;

	void           initCounter();
	uint32_t       getCounter();
	void           resetCounter();

	uint8_t        readRegister(uint8_t reg);
	void           setRegister(uint8_t reg, uint8_t value);

	//  made virtual, see #66
	uint8_t  readReg(uint8_t reg);
	uint16_t readReg2(uint8_t reg);
	uint8_t  writeReg(uint8_t reg, uint8_t value);
	uint8_t  writeReg2(uint8_t reg, uint16_t value);

	uint8_t        hiWord(uint8_t in) { return (in & 0x0fu); }
	uint8_t        loWord(uint8_t in) { return ((in & 0xf0u) >> 4); }

	uint32_t       _count{0};
	uint16_t       _reset_count{0};
	hrt_abstime    _last_measurement_time{0};
	hrt_abstime    _last_reset_time{0};
	int            _tranfer_fail_count{0};
	uint8_t        _last_config_register_content{0x00};

	//  address = fixed   0x36 for AS5600,
	//          = default 0x40 for AS5600L
	uint8_t  getAddress();


	//  SET CONFIGURE REGISTERS
	//  read datasheet first

	//  0         = AS5600_CLOCK_WISE
	//  1         = AS5600_COUNTERCLOCK_WISE
	//  all other = AS5600_COUNTERCLOCK_WISE
	void     setDirection(uint8_t direction = AS5600_CLOCK_WISE);
	uint8_t  getDirection();

	uint8_t  getZMCO();

	//  0 .. 4095
	//  returns false if parameter out of range
	bool     setZPosition(uint16_t value);
	uint16_t getZPosition();

	//  0 .. 4095
	//  returns false if parameter out of range
	bool     setMPosition(uint16_t value);
	uint16_t getMPosition();

	//  0 .. 4095
	//  returns false if parameter out of range
	bool     setMaxAngle(uint16_t value);
	uint16_t getMaxAngle();

	//  access the whole configuration register
	//  check datasheet for bit fields
	//  returns false if parameter out of range
	bool     setConfigure(uint16_t value);
	uint16_t getConfigure();

	//  access details of the configuration register
	//  0 = Normal
	//  1,2,3 are low power mode - check datasheet
	//  returns false if parameter out of range
	bool     setPowerMode(uint8_t powerMode);
	uint8_t  getPowerMode();

	//  0 = off    1 = lsb1    2 = lsb2    3 = lsb3
	//  returns false if parameter out of range
	//  suppresses noise when the magnet is not moving.
	bool     setHysteresis(uint8_t hysteresis);
	uint8_t  getHysteresis();

	//  0 = analog 0-100%
	//  1 = analog 10-90%
	//  2 = PWM
	//  returns false if parameter out of range
	bool     setOutputMode(uint8_t outputMode);
	uint8_t  getOutputMode();

	//  0 = 115    1 = 230    2 = 460    3 = 920 (Hz)
	//  returns false if parameter out of range
	bool     setPWMFrequency(uint8_t pwmFreq);
	uint8_t  getPWMFrequency();

	//  0 = 16x    1 = 8x     2 = 4x     3 = 2x
	//  returns false if parameter out of range
	bool     setSlowFilter(uint8_t mask);
	uint8_t  getSlowFilter();

	//  0 = none   1 = LSB6   2 = LSB7   3 = LSB9
	//  4 = LSB18  5 = LSB21  6 = LSB24  7 = LSB10
	//  returns false if parameter out of range
	bool     setFastFilter(uint8_t mask);
	uint8_t  getFastFilter();

	//  0 = OFF
	//  1 = ON   (auto low power mode)
	//  returns false if parameter out of range
	bool     setWatchDog(uint8_t mask);
	uint8_t  getWatchDog();


	//  READ OUTPUT REGISTERS
	uint16_t rawAngle();
	uint16_t readAngle();

	//  software based offset.
	//  degrees = -359.99 .. 359.99 (preferred)
	//  returns false if abs(parameter) > 36000
	//          => expect loss of precision
	bool     setOffset(float degrees);       //  sets an absolute offset
	float    getOffset();
	bool     increaseOffset(float degrees);  //  adds to existing offset.


	//  READ STATUS REGISTERS
	uint8_t  readStatus();
	uint8_t  readAGC();
	uint16_t readMagnitude();

	//  access detail status register
	bool     detectMagnet();
	bool     magnetTooStrong();
	bool     magnetTooWeak();


	//  BURN COMMANDS
	//  DO NOT UNCOMMENT - USE AT OWN RISK - READ DATASHEET
	//  void burnAngle();
	//  void burnSetting();


	//  EXPERIMENTAL 0.1.2 - to be tested.
	//  approximation of the angular speed in rotations per second.
	//  mode == 1: radians /second
	//  mode == 0: degrees /second  (default)
	float    getAngularSpeed(uint8_t mode = AS5600_MODE_DEGREES,
				 bool update = true);

	//  EXPERIMENTAL CUMULATIVE POSITION
	//  reads sensor and updates cumulative position
	int32_t  getCumulativePosition(bool update = true);
	//  converts last position to whole revolutions.
	int32_t  getRevolutions();
	//  resets position only (not the i)
	//  returns last position but not internal lastPosition.
	int32_t  resetPosition(int32_t position = 0);
	//  resets position and internal lastPosition
	//  returns last position.
	int32_t  resetCumulativePosition(int32_t position = 0);

	//  EXPERIMENTAL 0.5.2
	int      lastError();

	uORB::PublicationMulti<rpm_s> _rpm_pub{ORB_ID(rpm)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AS5600_POOL>) _param_as5600_pool,
		(ParamInt<px4::params::AS5600_RESET>) _param_as5600_reset,
		(ParamInt<px4::params::AS5600_MAGNET>) _param_as5600_magnet
	)
};
