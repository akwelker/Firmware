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
 * @file ActuatorEffectivenessQuadplaneVTOLCombinedModes.hpp
 *
 * Actuator effectiveness for quadplane VTOL flown in trajectory-tracker mode.
 * This mode implements Jacob Willis's combined hover/fixed-wing algorithm.
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#pragma once

#include "ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

class ActuatorEffectivenessQuadplaneVTOLCombinedModes: public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessQuadplaneVTOLCombinedModes();
	virtual ~ActuatorEffectivenessQuadplaneVTOLCombinedModes() = default;

	bool getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix) override;
	void setAirspeed(const float true_airspeed) override;

	/**
	 * Set the current flight phase
	 *
	 * @param Flight phase
	 */
	void setFlightPhase(const FlightPhase &flight_phase) override;

	int numActuators() const override { return 8; }
protected:
	bool _updated{true};
	RotorGeometry rotors[5];
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_MC_R0_PX>) _param_ca_mc_r0_px,
		(ParamFloat<px4::params::CA_MC_R0_PY>) _param_ca_mc_r0_py,
		(ParamFloat<px4::params::CA_MC_R0_PZ>) _param_ca_mc_r0_pz,
		(ParamFloat<px4::params::CA_MC_R0_AX>) _param_ca_mc_r0_ax,
		(ParamFloat<px4::params::CA_MC_R0_AY>) _param_ca_mc_r0_ay,
		(ParamFloat<px4::params::CA_MC_R0_AZ>) _param_ca_mc_r0_az,
		(ParamFloat<px4::params::CA_MC_R0_CT>) _param_ca_mc_r0_ct,
		(ParamFloat<px4::params::CA_MC_R0_KM>) _param_ca_mc_r0_km,

		(ParamFloat<px4::params::CA_MC_R1_PX>) _param_ca_mc_r1_px,
		(ParamFloat<px4::params::CA_MC_R1_PY>) _param_ca_mc_r1_py,
		(ParamFloat<px4::params::CA_MC_R1_PZ>) _param_ca_mc_r1_pz,
		(ParamFloat<px4::params::CA_MC_R1_AX>) _param_ca_mc_r1_ax,
		(ParamFloat<px4::params::CA_MC_R1_AY>) _param_ca_mc_r1_ay,
		(ParamFloat<px4::params::CA_MC_R1_AZ>) _param_ca_mc_r1_az,
		(ParamFloat<px4::params::CA_MC_R1_CT>) _param_ca_mc_r1_ct,
		(ParamFloat<px4::params::CA_MC_R1_KM>) _param_ca_mc_r1_km,

		(ParamFloat<px4::params::CA_MC_R2_PX>) _param_ca_mc_r2_px,
		(ParamFloat<px4::params::CA_MC_R2_PY>) _param_ca_mc_r2_py,
		(ParamFloat<px4::params::CA_MC_R2_PZ>) _param_ca_mc_r2_pz,
		(ParamFloat<px4::params::CA_MC_R2_AX>) _param_ca_mc_r2_ax,
		(ParamFloat<px4::params::CA_MC_R2_AY>) _param_ca_mc_r2_ay,
		(ParamFloat<px4::params::CA_MC_R2_AZ>) _param_ca_mc_r2_az,
		(ParamFloat<px4::params::CA_MC_R2_CT>) _param_ca_mc_r2_ct,
		(ParamFloat<px4::params::CA_MC_R2_KM>) _param_ca_mc_r2_km,

		(ParamFloat<px4::params::CA_MC_R3_PX>) _param_ca_mc_r3_px,
		(ParamFloat<px4::params::CA_MC_R3_PY>) _param_ca_mc_r3_py,
		(ParamFloat<px4::params::CA_MC_R3_PZ>) _param_ca_mc_r3_pz,
		(ParamFloat<px4::params::CA_MC_R3_AX>) _param_ca_mc_r3_ax,
		(ParamFloat<px4::params::CA_MC_R3_AY>) _param_ca_mc_r3_ay,
		(ParamFloat<px4::params::CA_MC_R3_AZ>) _param_ca_mc_r3_az,
		(ParamFloat<px4::params::CA_MC_R3_CT>) _param_ca_mc_r3_ct,
		(ParamFloat<px4::params::CA_MC_R3_KM>) _param_ca_mc_r3_km,

		(ParamFloat<px4::params::CA_MC_R4_PX>) _param_ca_mc_r4_px,
		(ParamFloat<px4::params::CA_MC_R4_PY>) _param_ca_mc_r4_py,
		(ParamFloat<px4::params::CA_MC_R4_PZ>) _param_ca_mc_r4_pz,
		(ParamFloat<px4::params::CA_MC_R4_AX>) _param_ca_mc_r4_ax,
		(ParamFloat<px4::params::CA_MC_R4_AY>) _param_ca_mc_r4_ay,
		(ParamFloat<px4::params::CA_MC_R4_AZ>) _param_ca_mc_r4_az,
		(ParamFloat<px4::params::CA_MC_R4_CT>) _param_ca_mc_r4_ct,
		(ParamFloat<px4::params::CA_MC_R4_KM>) _param_ca_mc_r4_km,

		(ParamFloat<px4::params::CA_WING_AREA>) _param_ca_wing_area,
		(ParamFloat<px4::params::CA_CHORD_LEN>) _param_ca_chord_len,
		(ParamFloat<px4::params::CA_SPAN>) _param_ca_span,
		(ParamFloat<px4::params::CA_C_ELL_DELTA_A>) _param_ca_c_l_delta_a,
		(ParamFloat<px4::params::CA_C_M_DELTA_E>) _param_ca_c_m_delta_e
	)
};
