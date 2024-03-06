/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/latch_sensor_status.h>
#include <px4_platform_common/events.h>
#include <uORB/topics/vehicle_command.h>

using namespace time_literals;


enum class LatchState {
	LOCKED = 0,
	UNLOCKED,
	IDLE
};

enum class LatchPosition {
	LEFTTOP = 0,
	LEFTBOTTOM,
	RIGHTTOP,
	RIGHTBOTTOM
};

class Latch
{
public:
	Latch() = default;

	// Initialize latch stauts
	void init();

	// check for latch status
	void check();

	// rest latch status
	bool is_updated(){return _status_changed;};

	// Update latch status
	void update();

	// Return latch's state in string format
	const char *get_state_str() const;

private:

	LatchState    _latchLastStatus[4];
	LatchPosition _latchPositions[4]= {
		LatchPosition::LEFTTOP,
		LatchPosition::LEFTBOTTOM,
		LatchPosition::RIGHTTOP,
		LatchPosition::RIGHTBOTTOM
		};
	uORB::Subscription                        _latch_sensor_status_sub{ORB_ID(latch_sensor_status)};
	//uORB::Publication<latch_sensor_status_s>  _latch_sensor_status_pub{ORB_ID(latch_sensor_status)};
	latch_sensor_status_s _latch_sensor_status;
	bool _status_changed{false};
	bool _report{false};
};
