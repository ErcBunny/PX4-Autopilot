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
 * @file WrenchAdjustionNone.cpp
 *
 * no adjust
 *
 *
 * @author SYZ
 */

#include "WrenchAdjustionL1.hpp"

void matrix_block_set(const matrix::SquareMatrix3d &input, matrix::Matrix<double, 6, 6> &output, int i_index,
		      int j_index)
{
	if (i_index > 3 || j_index > 3) {
		printf("error! overindex!");
		return;
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			output(i_index + i, j_index + j) = input(i, j);
		}
	}
}
void WrenchAdjustionL1::updateParameters()
{
	updateParams();
	_vehicle_mass = _param_vm_mass.get();
	_vehicle_inertia(0, 0) = _param_vm_inertia_xx.get();
	_vehicle_inertia(0, 1) = _param_vm_inertia_xy.get();
	_vehicle_inertia(0, 2) = _param_vm_inertia_xz.get();
	_vehicle_inertia(1, 0) = _param_vm_inertia_xy.get();
	_vehicle_inertia(1, 1) = _param_vm_inertia_yy.get();
	_vehicle_inertia(1, 2) = _param_vm_inertia_yz.get();
	_vehicle_inertia(2, 0) = _param_vm_inertia_xz.get();
	_vehicle_inertia(2, 1) = _param_vm_inertia_yz.get();
	_vehicle_inertia(2, 2) = _param_vm_inertia_zz.get();
	_center_of_mass(0) = _param_vm_com_x.get();
	_center_of_mass(1) = _param_vm_com_x.get();
	_center_of_mass(2) = _param_vm_com_x.get();
	_A.setZero();
	_A(0, 0) = _param_wa_lpf_a11.get();
	_A(1, 1) = _param_wa_lpf_a22.get();
	_A(2, 2) = _param_wa_lpf_a33.get();
	_A(3, 3) = _param_wa_lpf_a44.get();
	_A(4, 4) = _param_wa_lpf_a55.get();
	_A(5, 5) = _param_wa_lpf_a66.get();
	_C.setZero();
	_C(0, 0) = _param_wa_lpf_c11.get();
	_C(1, 1) = _param_wa_lpf_c22.get();
	_C(2, 2) = _param_wa_lpf_c33.get();
	_C(3, 3) = _param_wa_lpf_c44.get();
	_C(4, 4) = _param_wa_lpf_c55.get();
	_C(5, 5) = _param_wa_lpf_c66.get();
	_E.setZero();
	_E(0, 0) = _param_wa_lpf_e11.get();
	_E(1, 1) = _param_wa_lpf_e22.get();
	_E(2, 2) = _param_wa_lpf_e33.get();
	_E(3, 3) = _param_wa_lpf_e44.get();
	_E(4, 4) = _param_wa_lpf_e55.get();
	_E(5, 5) = _param_wa_lpf_e66.get();
	_G.setZero();
	_G(0, 0) = _param_wa_lpf_g11.get();
	_G(1, 1) = _param_wa_lpf_g22.get();
	_G(2, 2) = _param_wa_lpf_g33.get();
	_G(3, 3) = _param_wa_lpf_g44.get();
	_G(4, 4) = _param_wa_lpf_g55.get();
	_G(5, 5) = _param_wa_lpf_g66.get();
	_H.setZero();
	_H(0, 0) = _param_wa_lpf_h11.get();
	_H(1, 1) = _param_wa_lpf_h22.get();
	_H(2, 2) = _param_wa_lpf_h33.get();
	_H(3, 3) = _param_wa_lpf_h44.get();
	_H(4, 4) = _param_wa_lpf_h55.get();
	_H(5, 5) = _param_wa_lpf_h66.get();
	PX4_INFO("L1 parameters updated!");
	calculateOtherParams();
}

void WrenchAdjustionL1::calculateOtherParams()
{
	_D.setZero();
	_d_hat(0, 0) = _d_hat(1, 1) = _d_hat(2, 2) = 0;
	_d_hat(0, 1) = -1 * _center_of_mass(2);
	_d_hat(0, 2) =  _center_of_mass(1);
	_d_hat(1, 0) =  _center_of_mass(2);
	_d_hat(1, 2) = -1 * _center_of_mass(0);
	_d_hat(2, 0) = -1 * _center_of_mass(1);
	_d_hat(2, 1) =  _center_of_mass(0);

	_B.setZero();
	matrix::inv<double>(_vehicle_inertia, _J_inv);
	matrix::Matrix3d temp{};
	temp = -_J_inv * _d_hat;
	// B.block<3, 3>(3, 0) = -params.J_inv * params.d_hat;
	matrix_block_set(temp, _B, 3, 0);
	// B.block<3, 3>(3, 3) = params.J_inv;
	matrix_block_set(_J_inv, _B, 3, 3);

	_Binv.setZero();
	// B_inv.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
	matrix_block_set(_vehicle_inertia, _B, 3, 3);
	// B_inv.block<3, 3>(3, 3) = params.J;

	_lpf.x.setZero();
	_lpf.y.setZero();
}

void WrenchAdjustionL1::adjustCalculate(const WrenchAdjustionStates &states, const double &Ts)
{
	matrix::Dcm<double> R(states.q);// 这里的R就是matrix<double, 3> 类型的
	update_input_gain(_B, _Binv, R);
	update_reduced_states(_z, states.v, states.w);
	_sigma_hat = -_Binv * _E * (_z_hat - _z);
	// PX4_INFO("L1 adjust calculated _sigma_hat: \t%f \t %f", _sigma_hat(2), _sigma_hat(5));
	update_output_lpf(_thrust_adjusted_output, _torque_adjusted_output, _sigma_hat);

	update_reduced_states_estimation(_z_hat, states.thrust, states.torque, R, states.w, _B, _thrust_adjusted_output,
					 _torque_adjusted_output, _sigma_hat, _z, Ts);

	// PX4_INFO("L1 adjust calculated result: \t%f \t %f", _thrust_adjusted_output(0), _torque_adjusted_output(1));

	_thrust_adjusted = states.thrust + _thrust_adjusted_output;
	_torque_adjusted = states.torque + _torque_adjusted_output;
}


void WrenchAdjustionL1::update_input_gain(matrix::SquareMatrix<double, 6> &B, matrix::SquareMatrix<double, 6> Binv,
		const matrix::SquareMatrix3d &R)
{
	matrix_block_set(R / _vehicle_mass, _B, 0, 0);
	matrix_block_set(R.transpose() * _vehicle_mass, _Binv, 0, 0);
	matrix_block_set(_vehicle_mass * _d_hat * R.transpose(), _Binv, 3, 0);
}

void WrenchAdjustionL1::update_reduced_states(matrix::Vector<double, 6> &z, const matrix::Vector3d &v,
		const matrix::Vector3d &w)
{
	z(0) = v(0);
	z(1) = v(1);
	z(2) = v(2);
	z(3) = w(0);
	z(4) = w(1);
	z(5) = w(2);
}

void WrenchAdjustionL1::update_output_lpf(matrix::Vector3d &f, matrix::Vector3d &t, const matrix::Vector<double, 6> &in)
{
	_lpf.y = _C * _lpf.x + _D * in;
	_lpf.x = _G * _lpf.x + _H * in;
	f(0) = -_lpf.y(0);
	f(1) = -_lpf.y(1);
	f(2) = -_lpf.y(2);
	t(0) = -_lpf.y(3);
	t(1) = -_lpf.y(4);
	t(2) = -_lpf.y(5);
}

void WrenchAdjustionL1::update_reduced_states_estimation(matrix::Vector<double, 6> &z_hat,
		const matrix::Vector3d &f_raw, const matrix::Vector3d &t_raw,
		const matrix::SquareMatrix3d &R, const matrix::Vector3d &w,
		const matrix::SquareMatrix<double, 6> &B,
		const matrix::Vector3d &f_l1, const matrix::Vector3d &t_l1,
		const matrix::Vector<double, 6> &sigma_hat,
		const matrix::Vector<double, 6> &z,
		const double &Ts)
{
	matrix::Vector<double, 6> nominal_dynamics{};
	matrix::Vector3d temp{};
	matrix::Vector3d g(0, 0, 9.8);
	temp = R * f_raw / _vehicle_mass + g;
	nominal_dynamics(0) = temp(0);
	nominal_dynamics(1) = temp(1);
	nominal_dynamics(2) = temp(2);
	temp = _J_inv * (t_raw - _d_hat * f_raw - w.cross(_vehicle_inertia * w));
	nominal_dynamics(3) = temp(0);
	nominal_dynamics(4) = temp(1);
	nominal_dynamics(5) = temp(2);

	matrix::Vector<double, 6> z_hat_dot{};
	matrix::Vector<double, 6> u_l1{};
	u_l1(0) = f_l1(0);
	u_l1(1) = f_l1(1);
	u_l1(2) = f_l1(2);
	u_l1(3) = t_l1(0);
	u_l1(4) = t_l1(1);
	u_l1(5) = t_l1(2);
	z_hat_dot = nominal_dynamics + B * (u_l1 + sigma_hat) + _A * (z_hat - z);
	z_hat += Ts * z_hat_dot;
}

