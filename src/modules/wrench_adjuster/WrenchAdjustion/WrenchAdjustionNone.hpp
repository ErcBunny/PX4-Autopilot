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
 * @file WrenchAdjustionNone.hpp
 *
 * no adjust
 *
 *
 * @author SYZ
 */

#pragma once

#include "WrenchAdjustion.hpp"
#include <px4_platform_common/module_params.h>

class WrenchAdjustionNone: public WrenchAdjustion, private ModuleParams
{
public:
	WrenchAdjustionNone(): ModuleParams(nullptr) {}
	virtual ~WrenchAdjustionNone() = default;

	virtual void updateParameters() override;
	virtual void adjustCalculate(const WrenchAdjustionStates &states, const double &Ts) override;

private:
	DEFINE_PARAMETERS(
		// Vehicle physic params
		(ParamFloat<px4::params::VM_MASS>) _param_vm_mass,
		(ParamFloat<px4::params::VM_INERTIA_XX>) _param_vm_inertia_xx,
		(ParamFloat<px4::params::VM_INERTIA_YY>) _param_vm_inertia_yy,
		(ParamFloat<px4::params::VM_INERTIA_ZZ>) _param_vm_inertia_zz,
		(ParamFloat<px4::params::VM_INERTIA_XY>) _param_vm_inertia_xy,
		(ParamFloat<px4::params::VM_INERTIA_XZ>) _param_vm_inertia_xz,
		(ParamFloat<px4::params::VM_INERTIA_YZ>) _param_vm_inertia_yz,
		(ParamFloat<px4::params::VM_COM_X>) _param_vm_com_x,
		(ParamFloat<px4::params::VM_COM_Y>) _param_vm_com_y,
		(ParamFloat<px4::params::VM_COM_Z>) _param_vm_com_z,
		// L1 adaptive params
		(ParamFloat<px4::params::WA_LPF_A11>) _param_wa_lpf_a11,
		(ParamFloat<px4::params::WA_LPF_A22>) _param_wa_lpf_a22,
		(ParamFloat<px4::params::WA_LPF_A33>) _param_wa_lpf_a33,
		(ParamFloat<px4::params::WA_LPF_A44>) _param_wa_lpf_a44,
		(ParamFloat<px4::params::WA_LPF_A55>) _param_wa_lpf_a55,
		(ParamFloat<px4::params::WA_LPF_A66>) _param_wa_lpf_a66,
		(ParamFloat<px4::params::WA_LPF_C11>) _param_wa_lpf_c11,
		(ParamFloat<px4::params::WA_LPF_C22>) _param_wa_lpf_c22,
		(ParamFloat<px4::params::WA_LPF_C33>) _param_wa_lpf_c33,
		(ParamFloat<px4::params::WA_LPF_C44>) _param_wa_lpf_c44,
		(ParamFloat<px4::params::WA_LPF_C55>) _param_wa_lpf_c55,
		(ParamFloat<px4::params::WA_LPF_C66>) _param_wa_lpf_c66,
		(ParamFloat<px4::params::WA_LPF_E11>) _param_wa_lpf_e11,
		(ParamFloat<px4::params::WA_LPF_E22>) _param_wa_lpf_e22,
		(ParamFloat<px4::params::WA_LPF_E33>) _param_wa_lpf_e33,
		(ParamFloat<px4::params::WA_LPF_E44>) _param_wa_lpf_e44,
		(ParamFloat<px4::params::WA_LPF_E55>) _param_wa_lpf_e55,
		(ParamFloat<px4::params::WA_LPF_E66>) _param_wa_lpf_e66,
		(ParamFloat<px4::params::WA_LPF_G11>) _param_wa_lpf_g11,
		(ParamFloat<px4::params::WA_LPF_G22>) _param_wa_lpf_g22,
		(ParamFloat<px4::params::WA_LPF_G33>) _param_wa_lpf_g33,
		(ParamFloat<px4::params::WA_LPF_G44>) _param_wa_lpf_g44,
		(ParamFloat<px4::params::WA_LPF_G55>) _param_wa_lpf_g55,
		(ParamFloat<px4::params::WA_LPF_G66>) _param_wa_lpf_g66,
		(ParamFloat<px4::params::WA_LPF_H11>) _param_wa_lpf_h11,
		(ParamFloat<px4::params::WA_LPF_H22>) _param_wa_lpf_h22,
		(ParamFloat<px4::params::WA_LPF_H33>) _param_wa_lpf_h33,
		(ParamFloat<px4::params::WA_LPF_H44>) _param_wa_lpf_h44,
		(ParamFloat<px4::params::WA_LPF_H55>) _param_wa_lpf_h55,
		(ParamFloat<px4::params::WA_LPF_H66>) _param_wa_lpf_h66
	);
};

