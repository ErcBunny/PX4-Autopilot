/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
// #include <uORB/topics/vehicle_attitude.h>
// #include <uORB/topics/vehicle_attitude_setpoint.h>
// #include <uORB/topics/vehicle_local_position_setpoint.h>

struct WrenchAdjustionStates {
	matrix::Vector3d thrust;
	matrix::Vector3d torque;
	matrix::Vector3f pos;
	matrix::Vector3d v;
	matrix::Quaternion<double> q;
	matrix::Vector3d w;
};





class WrenchAdjustion
{
public:
	WrenchAdjustion() = default;
	virtual ~WrenchAdjustion() = default;
	virtual void updateParameters() = 0;
	virtual void adjustCalculate(const WrenchAdjustionStates &states, const double &Ts) = 0;

	// outputs after calculate
	matrix::Vector3d _thrust_adjusted;
	matrix::Vector3d _torque_adjusted;

// private:
// A G H C D E
	// matrix::Matrix<float, 6, 6> A;
	// matrix::Matrix<float, 6, 6> G;
	// matrix::Matrix<float, 6, 6> H;
	// matrix::Matrix<float, 6, 6> C;
	// matrix::Matrix<float, 6, 6> D;
	// matrix::Matrix<float, 6, 6> E;

};
