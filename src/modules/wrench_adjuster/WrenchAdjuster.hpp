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

/**
 * Multicopter position controller.
 */

#pragma once

#include "WrenchAdjustion/WrenchAdjustionNone.hpp"
#include "WrenchAdjustion/WrenchAdjustionL1.hpp"

#include <drivers/drv_hrt.h>
// #include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
// #include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
// #include <uORB/topics/vehicle_attitude.h>
// #include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
// #include <uORB/topics/vehicle_control_mode.h>
// #include <uORB/topics/vehicle_land_detected.h>
// #include <uORB/topics/vehicle_local_position.h>
// #include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint_adjusted.h>
#include <uORB/topics/vehicle_thrust_setpoint_adjusted.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_actuator_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_wrench_setpoint.h>

#include <uORB/topics/vehicle_angular_velocity.h> // get w
#include <uORB/topics/vehicle_local_position.h> // get p&v
#include <uORB/topics/vehicle_attitude.h> // get q
#include <matrix/Dcm.hpp>  // transform q to R

using namespace time_literals;

class WrenchAdjuster : public ModuleBase<WrenchAdjuster>,
// public control::SuperBlock,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WrenchAdjuster();
	~WrenchAdjuster();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

	virtual WrenchAdjustionStates set_wrench_ad_states(
		const vehicle_local_position_s &local_pos,
		const vehicle_attitude_s &attitude,
		const vehicle_angular_velocity_s &angular_vel,
		const matrix::Vector3d &thrust,
		const matrix::Vector3d &torque
	);

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void update_wrench_adjustion_method();
	void clear_wrench_adjustion_method();

	enum class WrenchAdjustionMethod {
		NONE = -1,
		L1 = 0,
		EKF = 1,
	};

	WrenchAdjustionMethod _wrench_adjustion_method_id{WrenchAdjustionMethod::NONE};
	WrenchAdjustion *_wrench_adjustion{nullptr};
	orb_advert_t _mavlink_log_pub{nullptr};

	//Outputs
	uORB::Publication<vehicle_thrust_setpoint_adjusted_s> _vehicle_thrust_setpoint_adjusted_pub{ORB_ID(vehicle_thrust_setpoint_adjusted)};
	uORB::Publication<vehicle_torque_setpoint_adjusted_s> _vehicle_torque_setpoint_adjusted_pub{ORB_ID(vehicle_torque_setpoint_adjusted)};
	// uORB::Publication<vehicle_attitude_setpoint_s>	_vehicle_attitude_setpoint_pub;
	// uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */

	//Inputs
	uORB::SubscriptionCallbackWorkItem _vehicle_wrench_setpoint_sub{this, ORB_ID(vehicle_wrench_setpoint)};  /**< wrench sp from adaptive mpc */
	uORB::SubscriptionCallbackWorkItem _vehicle_thrust_setpoint_sub{this, ORB_ID(vehicle_thrust_setpoint)};  /**< wrench sp from adaptive mpc */
	uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint)};  /**< wrench sp from adaptive mpc */
	uORB::SubscriptionCallbackWorkItem _vehicle_local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub {ORB_ID(vehicle_attitude)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	// uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	// uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	// uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	// uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	// uORB::Subscription _vehicle_attitude_sub {ORB_ID(vehicle_attitude)};

	matrix::Vector3d _torque_sp;
	matrix::Vector3d _thrust_sp;
	bool _use_wrench_sp;

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	hrt_abstime _last_warn{0}; /**< timer when the last warn message was sent out */
	hrt_abstime	_time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _last_run{0}, _last_wrench_update{0};
	hrt_abstime _timestamp_sample{0};

	bool _in_failsafe{false}; /**< true if failsafe was entered within current cycle */


	int _task_failure_count{0};         /**< counter for task failures */

	// vehicle_control_mode_s _control_mode{};
	// vehicle_local_position_setpoint_s _setpoint{};
	vehicle_attitude_s _vehicle_attitude {};
	vehicle_local_position_s _vehicle_local_pos {};
	vehicle_thrust_setpoint_s _vehicle_thrust_setpoint {};
	vehicle_torque_setpoint_s _vehicle_torque_setpoint {};
	vehicle_wrench_setpoint_s _vehicle_wrench_setpoint {};

	// vehicle_constraints_s _vehicle_constraints{
	// 	.timestamp = 0,
	// 	.speed_xy = NAN,
	// 	.speed_up = NAN,
	// 	.speed_down = NAN,
	// 	.want_takeoff = false,
	// };

	// vehicle_land_detected_s _vehicle_land_detected {
	// 	.timestamp = 0,
	// 	.alt_max = -1.0f,
	// 	.freefall = false,
	// 	.ground_contact = true,
	// 	.maybe_landed = true,
	// 	.landed = true,
	// };

	DEFINE_PARAMETERS(
		// Wrench Adjuster
		// (ParamFloat<px4::params::WA_LPF_A11>) _param_wa_a11,
		// (ParamFloat<px4::params::WA_A22>) _param_wa_a22,
		// (ParamFloat<px4::params::WA_A33>) _param_wa_a33,
		// (ParamFloat<px4::params::WA_A44>) _param_wa_a44,
		// (ParamFloat<px4::params::WA_A55>) _param_wa_a55,
		// (ParamFloat<px4::params::WA_A66>) _param_wa_a66,
		// (ParamFloat<px4::params::WA_LPF_OMEGA11>) _param_wa_lpf_omega11,
		// (ParamFloat<px4::params::WA_LPF_OMEGA22>) _param_wa_lpf_omega22,
		// (ParamFloat<px4::params::WA_LPF_OMEGA33>) _param_wa_lpf_omega33,
		// (ParamFloat<px4::params::WA_LPF_OMEGA44>) _param_wa_lpf_omega44,
		// (ParamFloat<px4::params::WA_LPF_OMEGA55>) _param_wa_lpf_omega55,
		// (ParamFloat<px4::params::WA_LPF_TEST>) _param_wa_test,
		(ParamInt<px4::params::WA_METHOD>) _param_wa_method
	);

	// control::BlockDerivative _vel_x_deriv; /**< velocity derivative in x */
	// control::BlockDerivative _vel_y_deriv; /**< velocity derivative in y */
	// control::BlockDerivative _vel_z_deriv; /**< velocity derivative in z */

	// PositionControl _control; /**< class for core PID position control */

	// bool _hover_thrust_initialized{false};

	/** Timeout in us for trajectory data to get considered invalid */
	// static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
	/** If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	// static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200_ms;
	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off and tilt is limited */
	// static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	// static constexpr float MAX_SAFE_TILT_DEG = 89.f; // Numerical issues above this value due to tanf

	// systemlib::Hysteresis _failsafe_land_hysteresis{false}; /**< becomes true if task did not update correctly for LOITER_TIME_BEFORE_DESCEND */
	// SlewRate<float> _tilt_limit_slew_rate;

	// uint8_t _vxy_reset_counter{0};
	// uint8_t _vz_reset_counter{0};
	// uint8_t _xy_reset_counter{0};
	// uint8_t _z_reset_counter{0};
	// uint8_t _heading_reset_counter{0};


	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	// int parameters_update(bool force);

	/**
	 * Check for validity of positon/velocity states.
	 */
	// PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, const vehicle_attitude_s &attitude);

	/**
	 * Adjust the setpoint during landing.
	 * Thrust is adjusted to support the land-detector during detection.
	 * @param setpoint gets adjusted based on land-detector state
	 */
	// void limit_thrust_during_landing(vehicle_attitude_setpoint_s &setpoint);

	/**
	 * Failsafe.
	 * If flighttask fails for whatever reason, then do failsafe. This could
	 * occur if the commander fails to switch to a mode in case of invalid states or
	 * setpoints. The failsafe will occur after LOITER_TIME_BEFORE_DESCEND. If force is set
	 * to true, the failsafe will be initiated immediately.
	 */
	// void failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states,
	//   bool warn);

	/**
	 * Reset setpoints to NAN
	 */
	// void reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint);
};
