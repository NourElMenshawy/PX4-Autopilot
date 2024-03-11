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

#include "payloadCheck.hpp"

using namespace time_literals;

void PayloadCheck::checkAndReport(const Context &context, Report &reporter)
{
	bool payload_secure_failure = false;

	latch_sensor_status_s latch_sensor_status;

	if (_latch_sensor_status_sub.copy(&latch_sensor_status)) {
		//_latch_sensor_status_sub.copy(&latch_sensor_status);
		PX4_INFO("Payload check \n");
		if(latch_sensor_status.latchsensor_1 || latch_sensor_status.latchsensor_2 ||
		latch_sensor_status.latchsensor_3 || latch_sensor_status.latchsensor_4)
		{
			payload_secure_failure = true;
			if (reporter.mavlink_log_pub()) {
				PX4_INFO("Payload Reporer is not secured");
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Payload is not secured");
				}

		}

	}

	reporter.failsafeFlags().payload_secure_failure = payload_secure_failure;
}
