/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>


using namespace time_literals;

class Stewart : public ModuleBase<Stewart>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Stewart();
	~Stewart() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void vehicle_cmd_publish(float pulse_servo_1, float pulse_servo_2, float pulse_servo_3, uint8_t sys_id);

	void parameters_update();

	inline int8_t deg_to_pulse_normed(float deg)
	{
		int16_t deg_tmp = deg_limit(deg);
		return (PULSE_NOMRED_MAX - PULSE_NOMRED_MIN) * (deg_tmp - _deg_min) / (_deg_max - _deg_min) + PULSE_NOMRED_MIN;
	}

	int16_t deg_limit(int16_t deg)
	{
		if(deg > _deg_max)	return _deg_max;
		else if(deg < _deg_min) return _deg_min;
		else			return deg;
	}

	// Publications
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription 		   _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PWM_MAIN_MAX1>) _param_pwm_main_max1,
		(ParamInt<px4::params::PWM_MAIN_MAX2>) _param_pwm_main_max2,
		(ParamInt<px4::params::PWM_MAIN_MAX3>) _param_pwm_main_max3,
		(ParamInt<px4::params::PWM_MAIN_MIN1>) _param_pwm_main_min1,
		(ParamInt<px4::params::PWM_MAIN_MIN2>) _param_pwm_main_min2,
		(ParamInt<px4::params::PWM_MAIN_MIN3>) _param_pwm_main_min3,
		(ParamFloat<px4::params::STEWART_DEG_MIN>) _param_stewart_deg_min,
		(ParamFloat<px4::params::STEWART_DEG_MAX>) _param_stewart_deg_max
	)


	float _pulse_servo_1{0.0f};
	float _pulse_servo_2{0.0f};
	float _pulse_servo_3{0.0f};
	float _param_pulse[6]= {0.0f, };
	float _deg_min{0.0f};
	float _deg_max{0.0f};
	static constexpr int8_t PULSE_NOMRED_MAX = 1.0f;
	static constexpr int8_t PULSE_NOMRED_MIN = -1.0f;
};
