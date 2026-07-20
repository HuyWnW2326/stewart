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

#include "Stewart.hpp"

Stewart::Stewart() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

Stewart::~Stewart()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool Stewart::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void Stewart::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();

		_param_pulse[0] = _param_pwm_main_min1.get();
		_param_pulse[1] = _param_pwm_main_max1.get();
		_param_pulse[2] = _param_pwm_main_min2.get();
		_param_pulse[3] = _param_pwm_main_max2.get();
		_param_pulse[4] = _param_pwm_main_min3.get();
		_param_pulse[5] = _param_pwm_main_max3.get();
	}
}

void Stewart::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&vehicle_status);
	}

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		// _pulse_servo_1 = deg_to_pulse_normed(60.0f);
		// _pulse_servo_2 = deg_to_pulse_normed(45.0f);
		// _pulse_servo_3 = deg_to_pulse_normed(90.0f);
	}

	// vehicle_cmd_publish(_pulse_servo_1, _pulse_servo_2, _pulse_servo_3, vehicle_status.system_id);

	perf_end(_loop_perf);
}

void Stewart::vehicle_cmd_publish(float pulse_servo_1, float pulse_servo_2, float pulse_servo_3, uint8_t sys_id)
{
	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.command   = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;

	vehicle_command.param1 = pulse_servo_1;
	vehicle_command.param2 = pulse_servo_2;
	vehicle_command.param3 = pulse_servo_3;
	vehicle_command.param4 = NAN;
	vehicle_command.param5 = NAN;
	vehicle_command.param6 = NAN;
	vehicle_command.param7 = 0;

	vehicle_command.target_system    = sys_id;
	vehicle_command.target_component = 1;
	vehicle_command.source_system    = vehicle_command.target_system;
	vehicle_command.source_component = 0;
	vehicle_command.confirmation     = 0;
	vehicle_command.from_external    = false;

	_vehicle_cmd_pub.publish(vehicle_command);
}

int Stewart::task_spawn(int argc, char *argv[])
{
	Stewart *instance = new Stewart();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Stewart::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int Stewart::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Stewart::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("Stewart", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int stewart_main(int argc, char *argv[])
{
	return Stewart::main(argc, argv);
}
