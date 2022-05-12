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
 * @file NonlinearEffectivenessVTOLQuadplane.hpp
 *
 * Nonlinear functions for calculating optimal actuator setpoint for a quadplane
 * being flown in continuous mode (no separate hover/fixed-wing flight)
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>
#include "NonlinearEffectiveness.hpp"

class NonlinearEffectivenessVTOLQuadplane: public NonlinearEffectiveness, ModuleParams
{
public:
	NonlinearEffectivenessVTOLQuadplane() : ModuleParams(nullptr){
		for (uint8_t i = 0; i < 4; ++i) {
			_rotor_params[i].CQ0 = _param_ca_rot0_CQ0.get();
			_rotor_params[i].CQ1 = _param_ca_rot0_CQ1.get();
			_rotor_params[i].CQ2 = _param_ca_rot0_CQ2.get();
			_rotor_params[i].CT0 = _param_ca_rot0_CT0.get();
			_rotor_params[i].CT1 = _param_ca_rot0_CT1.get();
			_rotor_params[i].CT2 = _param_ca_rot0_CT2.get();
			_rotor_params[i].prop_diam = _param_ca_rot0_prop_diam.get();
			_rotor_params[i].KV = _param_ca_rot0_KV.get();
			_rotor_params[i].KQ = (1.f / _rotor_params[i].KV) * 60.f / (2.f * (float) M_PI);
			_rotor_params[i].resistance = _param_ca_rot0_res.get();
			_rotor_params[i].i0 = _param_ca_rot0_i0.get();
			_rotor_params[i].V_max = _param_ca_rot0_vmax.get();
		}

		_rotor_params[0].position_x = _param_ca_mc_r0_px.get();
		_rotor_params[0].position_y = _param_ca_mc_r0_py.get();
		_rotor_params[0].position_z = _param_ca_mc_r0_pz.get();
		_rotor_params[0].axis_x = _param_ca_mc_r0_ax.get();
		_rotor_params[0].axis_y = _param_ca_mc_r0_ay.get();
		_rotor_params[0].axis_z = _param_ca_mc_r0_az.get();
		_rotor_params[1].position_x = _param_ca_mc_r1_px.get();
		_rotor_params[1].position_y = _param_ca_mc_r1_py.get();
		_rotor_params[1].position_z = _param_ca_mc_r1_pz.get();
		_rotor_params[1].axis_x = _param_ca_mc_r1_ax.get();
		_rotor_params[1].axis_y = _param_ca_mc_r1_ay.get();
		_rotor_params[1].axis_z = _param_ca_mc_r1_az.get();
		_rotor_params[2].position_x = _param_ca_mc_r2_px.get();
		_rotor_params[2].position_y = _param_ca_mc_r2_py.get();
		_rotor_params[2].position_z = _param_ca_mc_r2_pz.get();
		_rotor_params[2].axis_x = _param_ca_mc_r2_ax.get();
		_rotor_params[2].axis_y = _param_ca_mc_r2_ay.get();
		_rotor_params[2].axis_z = _param_ca_mc_r2_az.get();
		_rotor_params[3].position_x = _param_ca_mc_r3_px.get();
		_rotor_params[3].position_y = _param_ca_mc_r3_py.get();
		_rotor_params[3].position_z = _param_ca_mc_r3_pz.get();
		_rotor_params[3].axis_x = _param_ca_mc_r3_ax.get();
		_rotor_params[3].axis_y = _param_ca_mc_r3_ay.get();
		_rotor_params[3].axis_z = _param_ca_mc_r3_az.get();

		_rotor_params[4].position_x = _param_ca_mc_r4_px.get();
		_rotor_params[4].position_y = _param_ca_mc_r4_py.get();
		_rotor_params[4].position_z = _param_ca_mc_r4_pz.get();
		_rotor_params[4].axis_x = _param_ca_mc_r4_ax.get();
		_rotor_params[4].axis_y = _param_ca_mc_r4_ay.get();
		_rotor_params[4].axis_z = _param_ca_mc_r4_az.get();
		_rotor_params[4].CQ0 = _param_ca_rot1_CQ0.get();
		_rotor_params[4].CQ1 = _param_ca_rot1_CQ1.get();
		_rotor_params[4].CQ2 = _param_ca_rot1_CQ2.get();
		_rotor_params[4].CT0 = _param_ca_rot1_CT0.get();
		_rotor_params[4].CT1 = _param_ca_rot1_CT1.get();
		_rotor_params[4].CT2 = _param_ca_rot1_CT2.get();
		_rotor_params[4].prop_diam = _param_ca_rot1_prop_diam.get();
		_rotor_params[4].KV = _param_ca_rot1_KV.get();
		_rotor_params[4].KQ = (1.f / _rotor_params[4].KV) * 60.f / (2.f * (float) M_PI);
		_rotor_params[4].resistance = _param_ca_rot1_res.get();
		_rotor_params[4].i0 = _param_ca_rot1_i0.get();
		_rotor_params[4].V_max = _param_ca_rot1_vmax.get();
	}
	virtual ~NonlinearEffectivenessVTOLQuadplane() = default;


	matrix::Vector<float, VTOL_NUM_ACTUATORS>
	getSolutionFromActuatorSp(matrix::Vector<float, 16> &actuator_sp);

	matrix::Vector<float, 16>
	getActuatorSpFromSolution(matrix::Vector<float, VTOL_NUM_ACTUATORS> &solution);

	void calcThrustTorqueAchieved(
	    	matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
	    	matrix::Vector<float, VTOL_NUM_ACTUATORS> &x,
		matrix::Vector3f vBody, float airspeed, float airDensity);

	float
	nonlinearCtrlOptFun(
		const matrix::Vector<float, VTOL_NUM_ACTUATORS>& vals_inp,
		matrix::Vector<float, VTOL_NUM_ACTUATORS> *grad_out,
		void *opt_data);

	matrix::Vector2f
	calcControlSurfaceForce(struct NonlinearEffectiveness_ControlAllocationData *data);

protected:


	void calcThrustTorqueAchieved(matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    		float *thrust, float *torque, matrix::Vector2f elevonForceCoefs,
    		matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma);

	void calcThrustTorqueAchievedDer(
    		matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> *thrustTorqueAchievedDer,
    		float *thrust, float *torque, float *thrustDer, float *torqueDer,
    		matrix::Vector2f elevonForceCoefs, matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma);

	NonlinearEffectiveness_RotorParams _rotor_params[5];

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_MC_R0_PX>) _param_ca_mc_r0_px,
		(ParamFloat<px4::params::CA_MC_R0_PY>) _param_ca_mc_r0_py,
		(ParamFloat<px4::params::CA_MC_R0_PZ>) _param_ca_mc_r0_pz,
		(ParamFloat<px4::params::CA_MC_R0_AX>) _param_ca_mc_r0_ax,
		(ParamFloat<px4::params::CA_MC_R0_AY>) _param_ca_mc_r0_ay,
		(ParamFloat<px4::params::CA_MC_R0_AZ>) _param_ca_mc_r0_az,

		(ParamFloat<px4::params::CA_MC_R1_PX>) _param_ca_mc_r1_px,
		(ParamFloat<px4::params::CA_MC_R1_PY>) _param_ca_mc_r1_py,
		(ParamFloat<px4::params::CA_MC_R1_PZ>) _param_ca_mc_r1_pz,
		(ParamFloat<px4::params::CA_MC_R1_AX>) _param_ca_mc_r1_ax,
		(ParamFloat<px4::params::CA_MC_R1_AY>) _param_ca_mc_r1_ay,
		(ParamFloat<px4::params::CA_MC_R1_AZ>) _param_ca_mc_r1_az,

		(ParamFloat<px4::params::CA_MC_R2_PX>) _param_ca_mc_r2_px,
		(ParamFloat<px4::params::CA_MC_R2_PY>) _param_ca_mc_r2_py,
		(ParamFloat<px4::params::CA_MC_R2_PZ>) _param_ca_mc_r2_pz,
		(ParamFloat<px4::params::CA_MC_R2_AX>) _param_ca_mc_r2_ax,
		(ParamFloat<px4::params::CA_MC_R2_AY>) _param_ca_mc_r2_ay,
		(ParamFloat<px4::params::CA_MC_R2_AZ>) _param_ca_mc_r2_az,

		(ParamFloat<px4::params::CA_MC_R3_PX>) _param_ca_mc_r3_px,
		(ParamFloat<px4::params::CA_MC_R3_PY>) _param_ca_mc_r3_py,
		(ParamFloat<px4::params::CA_MC_R3_PZ>) _param_ca_mc_r3_pz,
		(ParamFloat<px4::params::CA_MC_R3_AX>) _param_ca_mc_r3_ax,
		(ParamFloat<px4::params::CA_MC_R3_AY>) _param_ca_mc_r3_ay,
		(ParamFloat<px4::params::CA_MC_R3_AZ>) _param_ca_mc_r3_az,

		(ParamFloat<px4::params::CA_MC_R4_PX>) _param_ca_mc_r4_px,
		(ParamFloat<px4::params::CA_MC_R4_PY>) _param_ca_mc_r4_py,
		(ParamFloat<px4::params::CA_MC_R4_PZ>) _param_ca_mc_r4_pz,
		(ParamFloat<px4::params::CA_MC_R4_AX>) _param_ca_mc_r4_ax,
		(ParamFloat<px4::params::CA_MC_R4_AY>) _param_ca_mc_r4_ay,
		(ParamFloat<px4::params::CA_MC_R4_AZ>) _param_ca_mc_r4_az,

		(ParamFloat<px4::params::CA_WING_AREA>) _param_ca_wing_area,
		(ParamFloat<px4::params::CA_CHORD_LEN>) _param_ca_chord_len,
		(ParamFloat<px4::params::CA_SPAN>) _param_ca_span,
		(ParamFloat<px4::params::CA_C_ELL_DELTA_A>) _param_ca_C_ell_delta_a,
		(ParamFloat<px4::params::CA_C_M_DELTA_E>) _param_ca_C_m_delta_e,
		(ParamFloat<px4::params::CA_C_L_DELTA_E>) _param_ca_C_L_delta_e,
		(ParamFloat<px4::params::CA_C_D_DELTA_E>) _param_ca_C_D_delta_e,

		(ParamFloat<px4::params::CA_TLT_SRVO_MIN>) _param_ca_tlt_servo_min,
		(ParamFloat<px4::params::CA_TLT_SRVO_MAX>) _param_ca_tlt_servo_max,
		(ParamFloat<px4::params::CA_MAX_F_POS_X>) _param_ca_f_pos_x,
		(ParamFloat<px4::params::CA_MAX_F_NEG_X>) _param_ca_f_neg_x,
		(ParamFloat<px4::params::CA_MAX_F_POS_Z>) _param_ca_f_pos_z,
		(ParamFloat<px4::params::CA_MAX_F_NEG_Z>) _param_ca_f_neg_z,
		(ParamInt<px4::params::CA_NL_ITER_MAX>) _param_ca_nonlin_iter_max,

		(ParamFloat<px4::params::CA_ROT0_CQ0>) _param_ca_rot0_CQ0,
		(ParamFloat<px4::params::CA_ROT0_CQ1>) _param_ca_rot0_CQ1,
		(ParamFloat<px4::params::CA_ROT0_CQ2>) _param_ca_rot0_CQ2,
		(ParamFloat<px4::params::CA_ROT0_CT0>) _param_ca_rot0_CT0,
		(ParamFloat<px4::params::CA_ROT0_CT1>) _param_ca_rot0_CT1,
		(ParamFloat<px4::params::CA_ROT0_CT2>) _param_ca_rot0_CT2,
		(ParamFloat<px4::params::CA_ROT0_PROP_D>) _param_ca_rot0_prop_diam,
		(ParamFloat<px4::params::CA_ROT0_KV>) _param_ca_rot0_KV,
		(ParamFloat<px4::params::CA_ROT0_RES>) _param_ca_rot0_res,
		(ParamFloat<px4::params::CA_ROT0_I0>) _param_ca_rot0_i0,
		(ParamFloat<px4::params::CA_ROT0_VMAX>) _param_ca_rot0_vmax,

		(ParamFloat<px4::params::CA_ROT1_CQ0>) _param_ca_rot1_CQ0,
		(ParamFloat<px4::params::CA_ROT1_CQ1>) _param_ca_rot1_CQ1,
		(ParamFloat<px4::params::CA_ROT1_CQ2>) _param_ca_rot1_CQ2,
		(ParamFloat<px4::params::CA_ROT1_CT0>) _param_ca_rot1_CT0,
		(ParamFloat<px4::params::CA_ROT1_CT1>) _param_ca_rot1_CT1,
		(ParamFloat<px4::params::CA_ROT1_CT2>) _param_ca_rot1_CT2,
		(ParamFloat<px4::params::CA_ROT1_PROP_D>) _param_ca_rot1_prop_diam,
		(ParamFloat<px4::params::CA_ROT1_KV>) _param_ca_rot1_KV,
		(ParamFloat<px4::params::CA_ROT1_RES>) _param_ca_rot1_res,
		(ParamFloat<px4::params::CA_ROT1_I0>) _param_ca_rot1_i0,
		(ParamFloat<px4::params::CA_ROT1_VMAX>) _param_ca_rot1_vmax,

		(ParamFloat<px4::params::CA_SURF0_MAX_ANG>) _param_ca_surf0_max_ang,
		(ParamFloat<px4::params::CA_SURF1_MAX_ANG>) _param_ca_surf1_max_ang,
		(ParamFloat<px4::params::CA_SURF2_MAX_ANG>) _param_ca_surf2_max_ang
	)

};
