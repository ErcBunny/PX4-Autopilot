/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationOmniHex.cpp
 *
 * Control allocation algorithm exclusively built for omni hex
 *
 * @author Yueqian Liu <yueqianliu@outlook.com>
 */

#include "ControlAllocationOmniHex.hpp"

void ControlAllocationOmniHex::updateParameters()
{
	_servo_delta_lim = _param_ca_oh_arm_d_max.get();
    _debug_switch_flag = _param_ca_oh_debug_en.get();
    _force_norm_thresh = _param_ca_oh_force_thr.get();
	_damp_thresh = _param_ca_oh_damp_thr.get();
	_align_thresh = _param_ca_oh_algn_thr.get();
	_rotate_direction = _param_ca_oh_rotate_dir.get();
}

void ControlAllocationOmniHex::publish_servo_setpoint()
{
	arm_rotation_s arm_rotation_data;
	arm_rotation_data.timestamp = hrt_absolute_time();

	if(_rotate_direction == 0)
	{
		arm_rotation_data.pos[0] = _servo_sp(0);
		arm_rotation_data.pos[1] = _servo_sp(1);
		arm_rotation_data.pos[2] = _servo_sp(2);
		arm_rotation_data.pos[3] = _servo_sp(3);
		arm_rotation_data.pos[4] = _servo_sp(4);
		arm_rotation_data.pos[5] = _servo_sp(5);
	}
	else
	{
		arm_rotation_data.pos[0] = -_servo_sp(0);
		arm_rotation_data.pos[1] = -_servo_sp(1);
		arm_rotation_data.pos[2] = -_servo_sp(2);
		arm_rotation_data.pos[3] = -_servo_sp(3);
		arm_rotation_data.pos[4] = -_servo_sp(4);
		arm_rotation_data.pos[5] = -_servo_sp(5);
	}
	arm_rotation_data.limit_reached = _limit_reached;

	_arm_rotation_pub.publish(arm_rotation_data);
}

float ControlAllocationOmniHex::angle_between(matrix::Vector<float, 3> a, matrix::Vector<float, 3> b)
{
	matrix::Vector<float, 1> dot_product = a.T() * b;
	float theta = acosf(dot_product(0) / (a.norm() * b.norm()));
	return theta;
}

void ControlAllocationOmniHex::limit_servo_delta()
{
	for(int i = 0; i < 6; i++)
	{
		if(_servo_delta(i) > _servo_delta_lim)
			_servo_delta(i) = _servo_delta_lim;
		if(_servo_delta(i) < -_servo_delta_lim)
			_servo_delta(i) = -_servo_delta_lim;
		if(isnan(_servo_delta(i)))
			_servo_delta(i) = 0.f;
	}
}

void ControlAllocationOmniHex::handle_singularity()
{

	matrix::Matrix<float, 3, 6> arm_benchmark;
	arm_benchmark(0, 0) = 0;
	arm_benchmark(1, 0) = 1;
	arm_benchmark(2, 0) = 0;

	arm_benchmark(0, 1) = 0;
	arm_benchmark(1, 1) = -1;
	arm_benchmark(2, 1) = 0;
	
	arm_benchmark(0, 2) = sqrtf(3);
	arm_benchmark(1, 2) = -1;
	arm_benchmark(2, 2) = 0;
	
	arm_benchmark(0, 3) = -sqrtf(3);
	arm_benchmark(1, 3) = 1;
	arm_benchmark(2, 3) = 0;
	
	arm_benchmark(0, 4) = sqrtf(3);
	arm_benchmark(1, 4) = 1;
	arm_benchmark(2, 4) = 0;
	
	arm_benchmark(0, 5) = -sqrtf(3);
	arm_benchmark(1, 5) = -1;
	arm_benchmark(2, 5) = 0;

	matrix::Vector<float, 6> arm_theta;
	for(int i = 0; i < 6; i++)
	{
		arm_theta(i) = angle_between(arm_benchmark.col(i), _force_sp);
	}

	// handle kinematic singularity
	for(int i = 0; i < 6; i++)
	{
		if(arm_theta(i) < _damp_thresh)
		{
			float a;
			if(arm_theta(i) < _align_thresh)
				a = 1;
			else
				a = powf(1 - (arm_theta(i) - _align_thresh) / (_damp_thresh - _align_thresh), 2);

			_servo_delta(i) = _servo_delta(i) * (1 - a);
			if(i % 2 == 0)
			{
				_servo_delta(i + 1) = _servo_delta(i + 1) * (1 - a);
			}
			else
			{
				_servo_delta(i - 1) = _servo_delta(i - 1) * (1 - a);
			}

			break;
		}
	}


}

void ControlAllocationOmniHex::allocate()
{
	updatePseudoInverse();

	// Allocate
	_allocation_raw = _actuator_trim + _mix * (_control_sp - _control_trim);

	// Extract rotor setpoint and servo setpoint
	_rotor_sp.setZero();
	_servo_sp.setZero();
	for (int i = 0; i < 6; i++)
	{
		// _rotor_sp(i) = powf(powf(_allocation_raw(i * 2), 2) + powf(_allocation_raw(i * 2 + 1), 2), 0.5f); // rotor_sp = omega^2
		_rotor_sp(i) = powf(powf(_allocation_raw(i * 2), 2) + powf(_allocation_raw(i * 2 + 1), 2), 0.25f); // rotor_sp = omega
		_servo_sp(i) = atan2f(_allocation_raw(i * 2 + 1), _allocation_raw(i * 2));
	}

	// Post process
	_limit_reached = false;
	for(int i = 0; i < 6; i++)
	{
		while(_servo_sp(i) - _last_servo_sp(i) > M_PI_F)
			_servo_sp(i) -= 2 * M_PI_F;
		while(_servo_sp(i) - _last_servo_sp(i) < -M_PI_F)
			_servo_sp(i) += 2 * M_PI_F;
		if((_last_servo_sp(i) > 6.5f && _servo_sp(i) > _last_servo_sp(i)) || (_last_servo_sp(i) < -6.5f && _servo_sp(i) < _last_servo_sp(i)))
		{
			_limit_reached = true;
		}
		if(_servo_sp(i) > 7.0f)
		{
			_servo_sp(i) = 7.0f;
		}
		if(_servo_sp(i) < -7.0f)
		{
			_servo_sp(i) = -7.0f;
		}
	}
	_servo_delta = _servo_sp - _last_servo_sp;
	handle_singularity();
	limit_servo_delta();

	// Publish servo setpoint (now goes to _actuator_sp too)
	vehicle_status_s status;
	_vehicle_status_sub.copy(&status);

	if (_force_sp.norm() < _force_norm_thresh || status.arming_state != vehicle_status_s::ARMING_STATE_ARMED)
	{
		_servo_delta.setZero();	// otherwise the condition number is too big
	}
	_servo_sp = _last_servo_sp + _servo_delta;
	if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED)
	{
		_servo_sp.setZero();
	}
	
	_force_sp(0) = _control_sp(3);
	_force_sp(1) = _control_sp(4);
	_force_sp(2) = _control_sp(5);
	_last_servo_sp = _servo_sp;
	publish_servo_setpoint();

	// Rotor setpoint is assigned to _acturator_sp and published in ControlAllocator.cpp
	_actuator_sp.setZero();
	for(int i = 0; i < 6; i++)
	{
		_actuator_sp(i) = _rotor_sp(i);
		_actuator_sp(i + 8) = _servo_sp(i);
	}

	// Compute achieved control
	_allocation_sp.setZero();
	for (int i = 0; i < 6; i++)
	{
		_allocation_sp(2 * i) = _rotor_sp(i) * _rotor_sp(i) * cosf(_servo_sp(i));
		_allocation_sp(2 * i + 1) = _rotor_sp(i) * _rotor_sp(i) * sinf(_servo_sp(i));
	}
	_control_allocated = _effectiveness * _allocation_sp;

	// Print debug
	if (_debug_switch_flag)
		print_debug_info();
}

void ControlAllocationOmniHex::print_debug_info()
{
	printf("--------------------------------------------------------------------------------------");
	printf("--------------------------------------------------------------------------------------");
	printf("--------------------------------------------------------------------------------------\n");
	printf("wrench=\n");
	matrix::Vector<float, NUM_AXES> wrench;
	wrench(0) = _control_sp(3);
	wrench(1) = _control_sp(4);
	wrench(2) = _control_sp(5);
	wrench(3) = _control_sp(0);
	wrench(4) = _control_sp(1);
	wrench(5) = _control_sp(2);
	wrench.T().print();
	printf("allocation_raw=\n");
	_allocation_raw.T().print();
	printf("rotor_sp=\n");
	_rotor_sp.T().print();
	printf("servo_sp=\n");
	_servo_sp.T().print();
	printf("allocation_sp=\n");
	_allocation_sp.T().print();
	printf("allocated_wrench=\n");
	matrix::Vector<float, NUM_AXES> awrench;
	awrench(0) = _control_allocated(3);
	awrench(1) = _control_allocated(4);
	awrench(2) = _control_allocated(5);
	awrench(3) = _control_allocated(0);
	awrench(4) = _control_allocated(1);
	awrench(5) = _control_allocated(2);
	awrench.T().print();
}