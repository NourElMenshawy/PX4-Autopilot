/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef LATCH_SENSOR_STATUS_HPP
#define LATCH_SENSOR_STATUS_HPP

#include <uORB/topics/latch_sensor_status.h>


class MavlinkStreamLatchSensorStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamLatchSensorStatus(mavlink); }

	static constexpr const char *get_name_static() { return "LATCH_SENSOR_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_LATCHSENSOR_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _latch_sensor_status_sub.advertised() ? MAVLINK_MSG_ID_LATCHSENSOR_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamLatchSensorStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _latch_sensor_status_sub{ORB_ID(latch_sensor_status)};


	bool send() override
	{
		latch_sensor_status_s _latch_sensor_status;

		if (_latch_sensor_status_sub.update(&_latch_sensor_status)) {
			mavlink_latchsensor_status_t msg{};
			msg.latchsensor_1 = _latch_sensor_status.latchsensor_1;
			msg.latchsensor_2 = _latch_sensor_status.latchsensor_2;
			msg.latchsensor_3 = _latch_sensor_status.latchsensor_3;
			msg.latchsensor_4 = _latch_sensor_status.latchsensor_4;
			mavlink_msg_latchsensor_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // LATCH_SENSOR_STATUS
