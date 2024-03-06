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

#include "latch.h"

void Latch::init()
{
	// Reset internal states
	_latch_sensor_status.latchsensor_1 = static_cast<uint8_t>(LatchState::IDLE);
	_latch_sensor_status.latchsensor_2 = static_cast<uint8_t>(LatchState::IDLE);
	_latch_sensor_status.latchsensor_3 = static_cast<uint8_t>(LatchState::IDLE);
	_latch_sensor_status.latchsensor_4 = static_cast<uint8_t>(LatchState::IDLE);
	for(int i = 0; i < 4; i++)
	{
		_latchLastStatus[i] = LatchState::IDLE;
	}

}



void Latch::update()
{
	latch_sensor_status_s latch_sensor_status;
	_status_changed = false;
	if (_latch_sensor_status_sub.update(&latch_sensor_status)) {
		_latch_sensor_status.latchsensor_1 = latch_sensor_status.latchsensor_1;
		_latch_sensor_status.latchsensor_2 = latch_sensor_status.latchsensor_2;
		_latch_sensor_status.latchsensor_3 = latch_sensor_status.latchsensor_3;
		_latch_sensor_status.latchsensor_4 = latch_sensor_status.latchsensor_4;
		_status_changed = true;
	}

}

void Latch::check()
{
	_report = false;
	// Check if the status has changed
	if(_latchLastStatus[static_cast<uint8_t>(LatchPosition::LEFTTOP)] == LatchState::UNLOCKED && _latch_sensor_status.latchsensor_1)
	{
		PX4_WARN(" Left Top Latch is not unlocked");
		_report = true;
	}
	_latchLastStatus[static_cast<uint8_t>(LatchPosition::LEFTTOP)] = static_cast<LatchState>(_latch_sensor_status.latchsensor_1);

	if(_latchLastStatus[static_cast<uint8_t>(LatchPosition::LEFTBOTTOM)] == LatchState::UNLOCKED && _latch_sensor_status.latchsensor_2)
	{
		PX4_WARN(" Left Bottom Latch is not unlocked");
		_report = true;

	}
	_latchLastStatus[static_cast<uint8_t>(LatchPosition::LEFTBOTTOM)] = static_cast<LatchState>(_latch_sensor_status.latchsensor_2);

	if(_latchLastStatus[static_cast<uint8_t>(LatchPosition::RIGHTTOP)] == LatchState::UNLOCKED && _latch_sensor_status.latchsensor_3)
	{
		PX4_WARN(" Right Top Latch is not unlocked");
		_report = true;
	}
	_latchLastStatus[static_cast<uint8_t>(LatchPosition::RIGHTTOP)] = static_cast<LatchState>(_latch_sensor_status.latchsensor_3);


	if(_latchLastStatus[static_cast<uint8_t>(LatchPosition::RIGHTBOTTOM)] == LatchState::UNLOCKED && _latch_sensor_status.latchsensor_4)
	{
		PX4_WARN(" Right Bottom Latch is not unlocked");
		_report = true;
	}
	_latchLastStatus[static_cast<uint8_t>(LatchPosition::RIGHTBOTTOM)] = static_cast<LatchState>(_latch_sensor_status.latchsensor_4);

	if(_report)
	{
		events::send(events::ID("payload"), events::Log::Error,
				     "Latches are not locked");
		PX4_WARN("Latches are not unlocked");
	}



}


const char *Latch::get_state_str() const
{
    static char result[100]; // Assuming a maximum length for the result string

    // Loop through latch statuses and build the result string
    for (size_t i = 0; i < 4; ++i) {
        const char* latchStr = "";
        switch (static_cast<int>(_latchLastStatus[i])) {
            case static_cast<int>(LatchState::IDLE):
                latchStr = "IDLE";
                break;
            case static_cast<int>(LatchState::LOCKED):
                latchStr = "LOCKED";
                break;
            case static_cast<int>(LatchState::UNLOCKED):
                latchStr = "UNLOCKED";
                break;
            default:
                latchStr = "UNKNOWN";
                break;
        }

        const char* positionStr = "";
        switch (i) {
            case 0:
                positionStr = "LEFTTOP";
                break;
            case 1:
                positionStr = "LEFTBOTTOM";
                break;
            case 2:
                positionStr = "RIGHTTOP";
                break;
            case 3:
                positionStr = "RIGHTBOTTOM";
                break;
            default:
                positionStr = "UNKNOWN";
                break;
        }

        // Append position and state to the result string
        snprintf(result + strlen(result), sizeof(result) - strlen(result), "%s: %s", positionStr, latchStr);

        // If not the last latch status, add a comma separator
        if (i != 3) {
            strcat(result, ", ");
        }
    }

    return result;

}
