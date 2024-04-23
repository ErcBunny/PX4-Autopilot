/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "WrenchAdjuster.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <mathlib/math/Functions.hpp>
// #include "PositionControl/ControlMath.hpp"

using namespace matrix;

WrenchAdjuster::WrenchAdjuster() :
	// SuperBlock(nullptr, "AdaptiveLayer"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
	// _vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	// _vel_x_deriv(this, "VELD"),
	// _vel_y_deriv(this, "VELD"),
	// _vel_z_deriv(this, "VELD")
{
	_wrench_adjustion = new WrenchAdjustionNone();
	parameters_updated();
	// _failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
	// _tilt_limit_slew_rate.setSlewRate(.2f);
	// reset_setpoint_to_nan(_setpoint);
}

WrenchAdjuster::~WrenchAdjuster()
{
	delete _wrench_adjustion;
	perf_free(_cycle_perf);
}

bool WrenchAdjuster::init()
{
	// if (!_local_pos_sub.registerCallback()) {
	// 	PX4_ERR("vehicle_local_position callback registration failed!");
	// 	return false;
	// }

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("vehicle_torque_setpoint callback registration failed!");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("vehicle_thrust_setpoint callback registration failed!");
		return false;
	}

	if (!_vehicle_local_pos_sub.registerCallback()) {
		PX4_ERR("local_pos_sub callback registration failed!");
		return false;
	}

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

WrenchAdjustionStates WrenchAdjuster::set_wrench_ad_states(
	const vehicle_local_position_s &local_pos,
	const vehicle_attitude_s &attitude,
	const vehicle_angular_velocity_s &angular_vel,
	const Vector3d &thrust,
	const Vector3d &torque
)
{
	WrenchAdjustionStates states;

	// only set position states if valid and finite
	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && PX4_ISFINITE(local_pos.z)
	    && local_pos.z_valid && local_pos.xy_valid) {
		states.pos(0) = local_pos.x;
		states.pos(1) = local_pos.y;
		states.pos(2) = local_pos.z;

	} else {
		states.pos(0) = NAN;
		states.pos(1) = NAN;
		states.pos(2) = NAN;
		PX4_WARN("states.pos NAN!");
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && PX4_ISFINITE(local_pos.vz)
	    && local_pos.v_z_valid && local_pos.v_xy_valid) {
		states.v(0) = local_pos.vx;
		states.v(1) = local_pos.vy;
		states.v(2) = local_pos.vz;

	} else {
		states.v(0) = NAN;
		states.v(1) = NAN;
		states.v(2) = NAN;
		PX4_WARN("states.v NAN!");
	}

	if (PX4_ISFINITE(attitude.q[0]) && PX4_ISFINITE(attitude.q[1]) && PX4_ISFINITE(attitude.q[2])
	    && PX4_ISFINITE(attitude.q[3])) {
		states.q(0) = attitude.q[0];
		states.q(1) = attitude.q[1];
		states.q(2) = attitude.q[2];
		states.q(3) = attitude.q[3];

	} else {
		states.q(0) = NAN;
		states.q(1) = NAN;
		states.q(2) = NAN;
		states.q(2) = NAN;
		PX4_WARN("states.q NAN!");
	}

	if (PX4_ISFINITE(angular_vel.xyz[0]) && PX4_ISFINITE(angular_vel.xyz[1]) && PX4_ISFINITE(angular_vel.xyz[2])) {
		states.w(0) = angular_vel.xyz[0];
		states.w(1) = angular_vel.xyz[1];
		states.w(2) = angular_vel.xyz[2];

	} else {
		states.w(0) = NAN;
		states.w(1) = NAN;
		states.w(2) = NAN;
		PX4_WARN("states.w NAN!");
	}

	if (PX4_ISFINITE(_thrust_sp(0)) && PX4_ISFINITE(_thrust_sp(1)) && PX4_ISFINITE(_thrust_sp(2))) {
		states.thrust = _thrust_sp;

	} else {
		states.thrust(0) = NAN;
		states.thrust(1) = NAN;
		states.thrust(2) = NAN;
		PX4_WARN("states.thrust NAN!");
	}

	if (PX4_ISFINITE(_torque_sp(0)) && PX4_ISFINITE(_torque_sp(1)) && PX4_ISFINITE(_torque_sp(2))) {
		states.torque = _torque_sp;

	} else {
		states.torque(0) = NAN;
		states.torque(1) = NAN;
		states.torque(2) = NAN;
		PX4_WARN("states.torque NAN!");
	}

	return states;
}

void WrenchAdjuster::parameters_updated()
{
	// check for parameter updates
	// if (_parameter_update_sub.updated() || force) {
	// 	// clear update
	// 	parameter_update_s pupdate;
	// 	_parameter_update_sub.copy(&pupdate);

	// 	// update parameters from storage
	// 	ModuleParams::updateParams();
	// 	// SuperBlock::updateParams();

	// }

	// _wrench_adjustion = new WrenchAdjustionNone();


	if (_wrench_adjustion == nullptr) {
		PX4_ERR("_wrench_adjustion == nullptr!");
		return;
	}

	update_wrench_adjustion_method();
	// Specific parameters are obtained in their respective methods
	return;
}

void WrenchAdjuster::update_wrench_adjustion_method()
{
	WrenchAdjustionMethod method = (WrenchAdjustionMethod)_param_wa_method.get();

	if (_wrench_adjustion_method_id != method) {
		WrenchAdjustion *tmp = nullptr;

		// PX4_INFO("in switch");

		switch (method) {
		case WrenchAdjustionMethod::NONE:
			tmp = new WrenchAdjustionNone();
			PX4_INFO("method == NONE!");
			break;

		case WrenchAdjustionMethod::L1:
			tmp = new WrenchAdjustionL1();
			PX4_INFO("method == L1!");
			break;

		// case WrenchAdjustionMethod::EKF:
		// 	tmp = new WrenchAdjustionEKF();
		// 	break;

		default:
			PX4_ERR("Unknown wrench adjustion method");
			break;
		}

		// Replace previous method with new one
		if (tmp == nullptr) {
			PX4_ERR("Wrench adjustion init failed");
			_param_wa_method.set((int)_wrench_adjustion_method_id);

		} else {
			delete _wrench_adjustion;
			_wrench_adjustion = tmp;

			_wrench_adjustion_method_id = method;
			PX4_INFO("_wrench_adjustion_method_id:\t%8.4d",
				 (int)method);
			// TODO: use the class function to configure new method
			_wrench_adjustion->updateParameters();
		}
	}
}

void WrenchAdjuster::clear_wrench_adjustion_method()
{
	WrenchAdjustion *tmp = nullptr;

	tmp = new WrenchAdjustionNone();
	PX4_INFO("RESET! method == NONE!");

	delete _wrench_adjustion;
	_wrench_adjustion = tmp;

	_wrench_adjustion->updateParameters();
}

void WrenchAdjuster::Run()
{
	// PX4_INFO("debug 1");

	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		_vehicle_wrench_setpoint_sub.unregisterCallback();
		_vehicle_local_pos_sub.unregisterCallback();
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	ScheduleDelayed(100_ms);
	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_wrench_adjustion) {
			_wrench_adjustion->updateParameters();
		}

		ModuleParams::updateParams();
		parameters_updated();
	}

	// reschedule backup

	// parameters_update(false);// TODO need to think again


	// Check if parameters have changed


	// reschedule backup
	// ScheduleDelayed(100_ms);
	const hrt_abstime now = hrt_absolute_time();
	// const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
	const float dt_w = (now - _last_wrench_update) / 1e6f;

	// PX4_INFO("dt_w = %.4f", (double)dt_w);
	// bool do_update = false;

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		// Time for L1 calculate
		const hrt_abstime time_stamp_now = angular_velocity.timestamp;
		const double Ts = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		// PX4_INFO("Ts = %.4f", (double)Ts);
		_time_stamp_last_loop = time_stamp_now;

		_timestamp_sample = angular_velocity.timestamp_sample;

		// Sub
		_vehicle_attitude_sub.update(&_vehicle_attitude);
		_vehicle_local_pos_sub.update(&_vehicle_local_pos);
		_vehicle_thrust_setpoint_sub.update(&_vehicle_thrust_setpoint);
		_vehicle_torque_setpoint_sub.update(&_vehicle_torque_setpoint);

		// from mpc
		if (_vehicle_wrench_setpoint_sub.update(&_vehicle_wrench_setpoint)) {

			_last_wrench_update = now;

			_use_wrench_sp = _vehicle_wrench_setpoint.wrench_sp_valid;

			if (_use_wrench_sp) {
				_thrust_sp = matrix::Vector3d(
						     _vehicle_wrench_setpoint.wrench[0],
						     _vehicle_wrench_setpoint.wrench[1],
						     _vehicle_wrench_setpoint.wrench[2]
					     );
				_torque_sp = matrix::Vector3d(
						     _vehicle_wrench_setpoint.wrench[3],
						     _vehicle_wrench_setpoint.wrench[4],
						     _vehicle_wrench_setpoint.wrench[5]
					     );

				// do_update = true;
				// _timestamp_sample = vehicle_wrench_setpoint.timestamp_sample;
			}
		}

		// if wrench_setpoint hasn't been updated for 20ms
		// and _use_wrench_sp is true (often in the case that MPC node is shut down)
		// switch back to PX4 MC control pipeline
		if (dt_w > 0.03f && _use_wrench_sp) {
			_use_wrench_sp = false;
			PX4_INFO("dt_w = %.4f, switch back to PX4 ctrl", (double)dt_w);
			clear_wrench_adjustion_method();
		}

		// from pid
		if (!_use_wrench_sp) {
			_thrust_sp = matrix::Vector3d(_vehicle_thrust_setpoint.xyz[0],
						      _vehicle_thrust_setpoint.xyz[1],
						      _vehicle_thrust_setpoint.xyz[2]
						     );
			_torque_sp = matrix::Vector3d(_vehicle_torque_setpoint.xyz[0],
						      _vehicle_torque_setpoint.xyz[1],
						      _vehicle_torque_setpoint.xyz[2]
						     );
		}

		// vehicle states update
		WrenchAdjustionStates states{set_wrench_ad_states(_vehicle_local_pos, _vehicle_attitude, angular_velocity, _thrust_sp, _torque_sp)};


		// use different method to calculate
		if (abs(Ts - 0.0040) > 0.0004) {
			PX4_WARN("Ts != 0.0040! please check the LPF params!");
			PX4_WARN("Ts = %.4f", (double)Ts);

		} else {
			//the params of L1 is decided by Ts and is pre-setted,
			//if Ts is abnormal, the result of L1 can not be used,
			//so just use the previous result.
			_wrench_adjustion->adjustCalculate(states, Ts);
		}

		// publish the result of adjuster
		vehicle_thrust_setpoint_adjusted_s vehicle_thrust_setpoint_adjusted{};
		vehicle_torque_setpoint_adjusted_s vehicle_torque_setpoint_adjusted{};

		vehicle_thrust_setpoint_adjusted.timestamp = hrt_absolute_time();
		vehicle_thrust_setpoint_adjusted.timestamp_sample = _timestamp_sample;
		vehicle_thrust_setpoint_adjusted.xyz[0] = _wrench_adjustion->_thrust_adjusted(0);
		vehicle_thrust_setpoint_adjusted.xyz[1] = _wrench_adjustion->_thrust_adjusted(1);
		vehicle_thrust_setpoint_adjusted.xyz[2] = _wrench_adjustion->_thrust_adjusted(2);

		vehicle_torque_setpoint_adjusted.timestamp = hrt_absolute_time();
		vehicle_torque_setpoint_adjusted.timestamp_sample = _timestamp_sample;
		vehicle_torque_setpoint_adjusted.xyz[0] = _wrench_adjustion->_torque_adjusted(0);
		vehicle_torque_setpoint_adjusted.xyz[1] = _wrench_adjustion->_torque_adjusted(1);
		vehicle_torque_setpoint_adjusted.xyz[2] = _wrench_adjustion->_torque_adjusted(2);

		_vehicle_thrust_setpoint_adjusted_pub.publish(vehicle_thrust_setpoint_adjusted);
		_vehicle_torque_setpoint_adjusted_pub.publish(vehicle_torque_setpoint_adjusted);




	}



	// for debug
	// PX4_INFO("WrenchAdjuster Run");
	// PX4_INFO("Torque:\t%8.4f\t%8.4f\t%8.4f",
	// 	 (double)_torque_sp(0),
	// 	 (double)_torque_sp(1),
	// 	 (double)_torque_sp(2));

	// if wrench or thrust or torque was updated, then update the states
	// and calculate the outputs.
	// if (do_update) {
	// _last_run = now;

	// }

	perf_end(_cycle_perf);
}


int WrenchAdjuster::task_spawn(int argc, char *argv[])
{
	// bool vtol = false;

	// if (argc > 1) {
	// 	if (strcmp(argv[1], "vtol") == 0) {
	// 		// vtol = true;
	// 	}
	// }

	WrenchAdjuster *instance = new WrenchAdjuster();

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

int WrenchAdjuster::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WrenchAdjuster::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int WrenchAdjuster::print_status(){
	return 0;
}

extern "C" __EXPORT int wrench_adjuster_main(int argc, char *argv[])
{
	return WrenchAdjuster::main(argc, argv);
}
