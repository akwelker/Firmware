/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file TiltrotorParams.c
 *
 * Tiltrotor actuator parameters
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

/**
 * Maximum tiltrotor servo angle in degrees
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_TLT_SRVO_MIN, 0.0);

/**
 * Maximum tiltrotor servo angle in degrees
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_TLT_SRVO_MAX, 115.0);

/**
 * Maximum force in the positive X direction
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MAX_F_POS_X, 18.);

/**
 * Maximum force in the negative X direction
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MAX_F_NEG_X, -4.34);

/**
 * Maximum force in the positive Z direction
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MAX_F_POS_Z, 100.);

/**
 * Maximum force in the negative Z direction
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_MAX_F_NEG_Z, -14.);

/**
 * Nonlinear optimization maximum number of iterations
 *
 * @group Control Allocation
 */
PARAM_DEFINE_INT32(CA_NL_ITER_MAX, 5);

///////////////// Rotor parameters //////////////////

/**
 * CQ0 rotor 0 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_CQ0, 0.0088);

/**
 * CQ1 rotor 0 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_CQ1, 0.0129);

/**
 * CQ2 rotor 0 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_CQ2, -0.0216);

/**
 * CT0 rotor 0 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_CT0, 0.1167);

/**
 * CT1 rotor 0 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_CT1, 0.0144);

/**
 * CT2 rotor 0 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_CT2, -0.1480);

/**
 * Rotor 0 prop diameter (m)
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_PROP_D, .1778);

/**
 * Rotor 0 KV
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_KV, 1450.0);

/**
 * Rotor 0 motor resistance
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_RES, 0.3);

/**
 * Rotor 0 no load current
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_I0, 0.83);

/**
 * Maximum voltage for rotor 0
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT0_VMAX, 11.1);

/**
 * CQ0 rotor 1 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_CQ0, 0.0088);

/**
 * CQ1 rotor 1 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_CQ1, 0.0129);

/**
 * CQ2 rotor 1 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_CQ2, -0.0216);

/**
 * CT0 rotor 1 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_CT0, 0.1167);

/**
 * CT1 rotor 1 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_CT1, 0.0144);

/**
 * CT2 rotor 1 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_CT2, -0.1480);

/**
 * Rotor 1 prop diameter (m)
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_PROP_D, 0.1778);

/**
 * Rotor 1 KV
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_KV, 1450.0);

/**
 * Rotor 1 motor resistance
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_RES, 0.3);

/**
 * Rotor 1 no load current
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_I0, 0.83);

/**
 * Maximum voltage for rotor 1
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT1_VMAX, 11.1);

/**
 * CQ0 rotor 2 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_CQ0, 0.0216);

/**
 * CQ1 rotor 2 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_CQ1, 0.0292);

/**
 * CQ2 rotor 2 moment coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_CQ2, -0.0368);

/**
 * CT0 rotor 2 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_CT0, 0.2097);

/**
 * CT1 rotor 2 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_CT1, 0.0505);

/**
 * CT2 rotor 2 thrust coefficient
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_CT2, -0.1921);

/**
 * Rotor 2 prop diameter (m)
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_PROP_D, 0.1397);

/**
 * Rotor 2 KV
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_KV, 1550.0);

/**
 * Rotor 2 motor resistance
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_RES, 0.4);

/**
 * Rotor 2 no load current
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_I0, 0.6);

/**
 * Maximum voltage for rotor 2
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_ROT2_VMAX, 11.1);

/**
 * Control Surface 0 max angle in degrees
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_SURF0_MAX_ANG, 45.0);

/**
 * Control Surface 1 max angle in degrees
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_SURF1_MAX_ANG, 45.0);

/**
 * Control Surface 2 max angle in degrees
 *
 * @group Control Allocation
 */
PARAM_DEFINE_FLOAT(CA_SURF2_MAX_ANG, 45.0);
