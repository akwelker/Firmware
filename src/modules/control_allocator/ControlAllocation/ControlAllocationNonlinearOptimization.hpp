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
 * @file ControlAllocationNonlinearOptimization.hpp
 *
 * Nonlinear optimizer based off of a quadratic thrust/torque rotor model
 *
 * The optimizer uses the BFGS algorithm to iteratively check potential actuator
 * setpoints for thrust/torque achieved until a iterations run out or a good-enough
 * setpoint is found.
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#pragma once

#include "ControlAllocation.hpp"
#include "ActuatorEffectiveness/NonlinearEffectiveness.hpp"
#include "ActuatorEffectiveness/NonlinearEffectivenessVTOLTiltrotor.hpp"
#include "ActuatorEffectiveness/NonlinearEffectivenessVTOLQuadplane.hpp"
#include <px4_platform_common/module_params.h>
#include <px4_log.h>

#define VTOL_NUM_AXES 5
#define VTOL_NUM_ACTUATORS 8

class ControlAllocationNonlinearOptimization: public ControlAllocation, public ModuleParams
{
public:
	ControlAllocationNonlinearOptimization();
	virtual ~ControlAllocationNonlinearOptimization() = default;

	virtual void allocate() override;
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const matrix::Vector<float, NUM_ACTUATORS> &actuator_trim, int num_actuators) override;

	void setNonlinearEffectiveness(size_t effectivenessID);

	/**
	 * Set the airspeed
	 *
	 * @param airspeed Vehicle's airspeed
	 */
	virtual void setAirspeed(const float airspeed) override
	{
		if (!std::isnan(airspeed))
			_airspeed = airspeed;
		else
			_airspeed = 0.f;
		_nonlin_effectiveness->setAirspeed(_airspeed);
	}

	/**
	 * Set the attitude
	 *
	 * @param q Vehicle's attitude
	 */
	void setOrientation(const matrix::Quatf q)
	{
		_q = q;
	}

	/**
	 * Set the inertial velocity
	 *
	 * @param inertial_velocity Vehicle's inertial velocity
	 */
	void setInertialVelocity(const matrix::Vector3f inertial_velocity)
	{
		_interial_velocity = inertial_velocity;
	}

	/**
	 * Set the air denisty
	 *
	 * @param air_density Air density
	 */
	void setAirDensity(const float air_density)
	{
		_air_density = air_density;
		_nonlin_effectiveness->setAirDensity(air_density);
	}

protected:

	typedef struct {
		float position_x;
		float position_y;
		float position_z;
		float axis_x;
		float axis_y;
		float axis_z;
		float CQ0;
		float CQ1;
		float CQ2;
		float CT0;
		float CT1;
		float CT2;
		float prop_diam;
		float KV;
		float KQ;
		float resistance;
		float i0;
	} RotorParams;

	void
	getAchievableThrust(
		matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueDesired,
		matrix::Vector<float, VTOL_NUM_ACTUATORS> actuators,
		NonlinearEffectiveness_ControlAllocationData *controlAllocationData);

	/**
	 * Guards against actuators getting stuck at their limits. A little hacky.
	 */
	void initialGuessGuard();

	matrix::Vector<float, VTOL_NUM_ACTUATORS>
	computeNonlinearOpt(
		matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired,
		matrix::Vector<float, VTOL_NUM_ACTUATORS> x0,
		NonlinearEffectiveness_ControlAllocationData *controlAllocationData, size_t iterMax);

	float
	nonlinearCtrlOptFun(
	const matrix::Vector<float, VTOL_NUM_ACTUATORS>& vals_inp,
	matrix::Vector<float, VTOL_NUM_ACTUATORS> *grad_out, void *opt_data) {
		return _nonlin_effectiveness->nonlinearCtrlOptFun(vals_inp, grad_out, opt_data);
	}

	NonlinearEffectiveness *_nonlin_effectiveness;
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;
	matrix::Vector<float, 7> _initGuessGuardCnt;

	bool _mix_update_needed{false};
	matrix::Quatf _q{1.f, 0.f, 0.f, 0.f};
	matrix::Vector3f _interial_velocity{0.f, 0.f, 0.f};
	float _air_density{1.225f};
	RotorParams _rotor_params[3];

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

		(ParamFloat<px4::params::CA_WING_AREA>) _param_ca_wing_area,
		(ParamFloat<px4::params::CA_CHORD_LEN>) _param_ca_chord_len,
		(ParamFloat<px4::params::CA_SPAN>) _param_ca_span,
		(ParamFloat<px4::params::CA_C_ELL_DELTA_A>) _param_ca_C_ell_delta_a,
		(ParamFloat<px4::params::CA_C_M_DELTA_E>) _param_ca_C_m_delta_e,
		(ParamFloat<px4::params::CA_C_L_DELTA_E>) _param_ca_C_L_delta_e,
		(ParamFloat<px4::params::CA_C_D_DELTA_E>) _param_ca_C_D_delta_e,

		(ParamFloat<px4::params::CA_TLT_SRVO_MIN>) _param_ca_tlt_servo_min,
		(ParamFloat<px4::params::CA_TLT_SRVO_MAX>) _param_ca_tlt_servo_max,
		(ParamFloat<px4::params::CA_ROT0_VMAX>) _param_ca_vmax,
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

		(ParamFloat<px4::params::CA_ROT0_CQ0>) _param_ca_rot2_CQ0,
		(ParamFloat<px4::params::CA_ROT0_CQ1>) _param_ca_rot2_CQ1,
		(ParamFloat<px4::params::CA_ROT0_CQ2>) _param_ca_rot2_CQ2,
		(ParamFloat<px4::params::CA_ROT0_CT0>) _param_ca_rot2_CT0,
		(ParamFloat<px4::params::CA_ROT0_CT1>) _param_ca_rot2_CT1,
		(ParamFloat<px4::params::CA_ROT0_CT2>) _param_ca_rot2_CT2,
		(ParamFloat<px4::params::CA_ROT0_PROP_D>) _param_ca_rot2_prop_diam,
		(ParamFloat<px4::params::CA_ROT0_KV>) _param_ca_rot2_KV,
		(ParamFloat<px4::params::CA_ROT0_RES>) _param_ca_rot2_res,
		(ParamFloat<px4::params::CA_ROT0_I0>) _param_ca_rot2_i0
	)

};
