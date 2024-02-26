/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file WrenchAdjustionParams.c
 * Parameters of L1 adaptive module for MPC controller of Omnihex.
 *
 * @author SYZ
 */


/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it < 0.
 *
 * @min -20.0
 * @max 0.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_A11, -6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it < 0.
 *
 * @min -20.0
 * @max 0.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_A22, -6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it < 0.
 *
 * @min -20.0
 * @max 0.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_A33, -6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it < 0.
 *
 * @min -20.0
 * @max 0.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_A44, -6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it < 0.
 *
 * @min -20.0
 * @max 0.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_A55, -6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it < 0.
 *
 * @min -20.0
 * @max 0.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_A66, -6.0f);

/**
 * C segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_C11, 3.0f);

/**
 * C segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_C22, 3.0f);

/**
 * C segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_C33, 3.0f);

/**
 * C segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_C44, 6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_C55, 6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_C66, 6.0f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_E11, 247.01199988f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_E22, 247.01199988f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_E33, 247.01199988f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_E44, 247.01199988f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_E55, 247.01199988f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1000.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_E66, 247.01199988f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_G11, 0.98807171f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_G22, 0.98807171f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_G33, 0.98807171f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_G44, 0.97628571f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_G55, 0.97628571f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_G66, 0.97628571f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_H11, 0.0039761f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_H22, 0.0039761f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_H33, 0.0039761f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_H44, 0.00395238f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_H55, 0.00395238f);

/**
 * A segment in Wrench Adjuster L1 LPF
 *
 * It's recommended to set it > 0.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 8
 * @group Wrench Adjuster
 */
PARAM_DEFINE_FLOAT(WA_LPF_H66, 0.00395238f);


