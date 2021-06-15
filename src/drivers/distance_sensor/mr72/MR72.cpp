/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

#include "MR72.hpp"

#include <fcntl.h>

MR72::MR72(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0 /* TODO: device ids */, ORB_PRIO_DEFAULT, rotation)
{
	PX4_INFO("gouzao");
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';

	// Use conservative distance bounds, to make sure we don't fuse garbage data
	_px4_rangefinder.set_min_distance(0.2f);	// Datasheet: 0.17m
	_px4_rangefinder.set_max_distance(7.9f);	// Datasheet: 8.0m
	_px4_rangefinder.set_fov(0.0488692f);


	_last_value = 0;
	_valid_orign_distance = 0;
	_keep_valid_time = 0;
	_mf_cycle_counter = 0;
	_var_cycle_couter = 0;
}

MR72::~MR72()
{
	// Ensure we are truly inactive.
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int
MR72::collect()
{
	perf_begin(_sample_perf);

	read_uart_data(_file_descriptor, &_uartbuf);
	uint16_t packet_message_id;
	packet_message_id = _packet.message_id2 * 256 + _packet.message_id1;

	if (packet_message_id== TARGET_INFO) {
				float range = 0;
				uint16_t tmp;

				TargetInfo &feed_back_data = _packet.payload.runinfo;
				tmp  = (uint16_t)(feed_back_data.RangeH << 8);
				tmp |= (uint16_t)(feed_back_data.RangeL);
				range = tmp * 0.01;
				float variance_after_filter;
				float range_value_after_filter;
				variance_after_filter = _calc_variance(range);
				range_value_after_filter = _median_filter(_valid_orign_distance);
				range_value_after_filter = range_value_after_filter > ULANDING_MAX_DISTANCE ? ULANDING_MAX_DISTANCE :
							  range_value_after_filter;
				range_value_after_filter = range_value_after_filter < ULANDING_MIN_DISTANCE ? ULANDING_MIN_DISTANCE :
							  range_value_after_filter;
							  _px4_rangefinder.set_covariance(variance_after_filter);
				const hrt_abstime timestamp_sample = hrt_absolute_time();
				_px4_rangefinder.update(timestamp_sample, range_value_after_filter);


	}
	perf_end(_sample_perf);

	return PX4_OK;
}


int
MR72::init()
{
	start();

	return PX4_OK;
}

int
MR72::open_serial_port(const speed_t speed)
{
	PX4_INFO("driver Run");
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}









int
MR72::read_uart_data(int uart_fd, UART_BUF *const uart_buf)
{
	int bytes_read = ::read(uart_fd, &_linebuf[0], sizeof(_linebuf));
	int i = 0;
	int bytes_processed = 0;

	if (bytes_read > 0 ) {
		i = bytes_read - 14;

		while(i > 0){
			if (_linebuf[i] == MR72_HEAD1 && _linebuf[i+1] == MR72_HEAD2) {
				bytes_processed = i;


			while (bytes_processed < bytes_read) {
				mr72_parser(_linebuf[bytes_processed],&_packet,_parse_state);
				bytes_processed++;
			}
			_parse_state = HEAD1;
			}
			i--;
		}

	} else if (bytes_read == -1 && errno == EAGAIN) {
		return -EAGAIN;

	} else {

		PX4_ERR("read error: %i, errno: %i", bytes_read, errno);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	return PX4_OK;
}


int
MR72::mr72_parser(uint8_t c, Packet *const packetdata,PARSR_mr72_STATE &state)
{
	int ret = -1;
	switch (state) {
	case HEAD1:
	if (c == MR72_HEAD1) {
			state = HEAD2;
			packetdata->start_sequence1 = MR72_HEAD1;
		}
	break;

	case HEAD2:
	if (c == MR72_HEAD2) {
			state = ID1;
			packetdata->start_sequence2 = MR72_HEAD2;
		}else {
			state = HEAD1;
		}
	break;

	case ID1:
	if (c == SENSOR_STATUS_L_8_BIT || c == TARGET_STATUS_L_8_BIT|| c == TARGET_INFO_L_8_BIT) {
		packetdata->message_id1 = c;
		state = ID2;
		} else {
		state = HEAD1;
		}
	break;

	case ID2:
	packetdata->message_id2 = c;
	state = DATA;
	data_index = 0;
	break;

	case DATA:
	packetdata->payload.bytes[data_index++] = c;

	if (data_index >= DATA_PAYLOAD_NUM) {
		state = END1;
	}
	break;

	case END1:
	if (c == MR72_END1) {
		packetdata->stop_sequence1 = MR72_END1; //just_keep the format
		state = END2;
		}
	break;

	case END2:
	if (c == MR72_END2) {
		packetdata->stop_sequence2 = MR72_END2; //just_keep the format
		state = HEAD1;
				}
	break;

	default:

		break;

	}


	return ret;
}



float
MR72::_calc_variance(float value)
{
	float accum = 0.f;
	float variance_sum = 0.f;

	/* this is an unusual special case handling,
	 * because ultrasound does not receive echoes in many cases, so we ignore the data
	 */
	if ((((value - _last_value) > 0.5f) ||  value >= ULANDING_MAX_DISTANCE || value <= ULANDING_MIN_DISTANCE)
	    && (hrt_elapsed_time(&_keep_valid_time) < RANDAR_DATA_ABNORMAL_TIMEOUT)) { // add abnormal timeout handle
		value = _last_value;
		_valid_orign_distance = _last_value;

	} else {
		_valid_orign_distance = value;
		_keep_valid_time = hrt_absolute_time();
	}

	_last_value = value;

	_variance_window[(_var_cycle_couter + 1) % _VAR_WINDOW_SIZE] = value;

	for (int i = 0; i < _VAR_WINDOW_SIZE; i++) {
		variance_sum += _variance_window[i];
	}

	_var_cycle_couter++;

	float average = variance_sum / _VAR_WINDOW_SIZE;

	for (int i = 0; i < _VAR_WINDOW_SIZE; i++) {
		accum  += (_variance_window[i] - average) * (_variance_window[i] - average);
	}

	return (accum / (_VAR_WINDOW_SIZE - 1));
}

int static cmp(const void *a, const void *b)
{
	return (*(const float *)a > *(const float *)b);
}


float
MR72::_median_filter(float value)
{
	/* TODO: replace with ring buffer */
	_mf_window[(_mf_cycle_counter + 1) % _MF_WINDOW_SIZE] = value;

	for (int i = 0; i < _MF_WINDOW_SIZE; ++i) {
		_mf_window_sorted[i] = _mf_window[i];
	}

	qsort(_mf_window_sorted, _MF_WINDOW_SIZE, sizeof(float), cmp);

	_mf_cycle_counter++;

	return _mf_window_sorted[_MF_WINDOW_SIZE / 2];
}


void
MR72::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
MR72::Run()
{

	// Ensure the serial port is open.
	open_serial_port();

	// Perform collection.
	collect();
}

void
MR72::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(MR72_MEASURE_INTERVAL);

	PX4_INFO("driver started");
}

void
MR72::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();

	// Ensure the serial port is closed.
	::close(_file_descriptor);
}
