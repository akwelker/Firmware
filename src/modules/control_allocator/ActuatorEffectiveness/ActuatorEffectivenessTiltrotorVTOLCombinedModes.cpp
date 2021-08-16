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
 * @file ActuatorEffectivenessTiltrotorVTOLCombinedModes.hpp
 *
 * Actuator effectiveness for tiltrotor VTOL
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#include "ActuatorEffectivenessTiltrotorVTOLCombinedModes.hpp"

ActuatorEffectivenessTiltrotorVTOLCombinedModes::ActuatorEffectivenessTiltrotorVTOLCombinedModes():
	ModuleParams(nullptr)
{
	rotors[0].position_x 	= _param_ca_mc_r0_px.get();
	rotors[0].position_y 	= _param_ca_mc_r0_py.get();
	rotors[0].position_z 	= _param_ca_mc_r0_pz.get();
	rotors[0].axis_x 	= _param_ca_mc_r0_ax.get();
	rotors[0].axis_y 	= _param_ca_mc_r0_ay.get();
	rotors[0].axis_z 	= _param_ca_mc_r0_az.get();
	rotors[0].thrust_coef   = _param_ca_mc_r0_ct.get();
	rotors[0].moment_ratio  = _param_ca_mc_r0_km.get();

	rotors[1].position_x 	= _param_ca_mc_r1_px.get();
	rotors[1].position_y 	= _param_ca_mc_r1_py.get();
	rotors[1].position_z 	= _param_ca_mc_r1_pz.get();
	rotors[1].axis_x 	= _param_ca_mc_r1_ax.get();
	rotors[1].axis_y 	= _param_ca_mc_r1_ay.get();
	rotors[1].axis_z 	= _param_ca_mc_r1_az.get();
	rotors[1].thrust_coef   = _param_ca_mc_r1_ct.get();
	rotors[1].moment_ratio  = _param_ca_mc_r1_km.get();

	rotors[2].position_x 	= _param_ca_mc_r2_px.get();
	rotors[2].position_y 	= _param_ca_mc_r2_py.get();
	rotors[2].position_z 	= _param_ca_mc_r2_pz.get();
	rotors[2].axis_x 	= _param_ca_mc_r2_ax.get();
	rotors[2].axis_y 	= _param_ca_mc_r2_ay.get();
	rotors[2].axis_z 	= _param_ca_mc_r2_az.get();
	rotors[2].thrust_coef   = _param_ca_mc_r2_ct.get();
	rotors[2].moment_ratio  = _param_ca_mc_r2_km.get();
}

bool
ActuatorEffectivenessTiltrotorVTOLCombinedModes::getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix)
{
	if (!_updated) {
		return false;
	}

	float elevon_factor = 0.5f * _air_density * _true_airspeed * _true_airspeed * _param_ca_wing_area.get();

	const float tiltrotor_vtol_combined_modes[NUM_AXES][NUM_ACTUATORS] = {
		{-rotors[0].moment_ratio,				(-rotors[0].thrust_coef * rotors[0].position_y),	-rotors[1].moment_ratio,				(-rotors[1].thrust_coef * rotors[1].position_y),	(-rotors[2].thrust_coef * rotors[2].position_y),	(elevon_factor * _param_ca_aero_moment_l.get() * _param_ca_span.get()),		(elevon_factor * _param_ca_aero_moment_l.get() * _param_ca_span.get()), 	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{(rotors[0].thrust_coef * rotors[0].position_z),	(rotors[0].thrust_coef * rotors[0].position_x),		(rotors[1].thrust_coef * rotors[1].position_z),		(rotors[1].thrust_coef * rotors[1].position_x),		(rotors[2].thrust_coef * rotors[2].position_x),		(-elevon_factor * _param_ca_aero_moment_m.get() * _param_ca_chord_len.get()),	(elevon_factor * _param_ca_aero_moment_m.get() * _param_ca_chord_len.get()), 	0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{(-rotors[0].thrust_coef * rotors[0].position_y),	rotors[0].moment_ratio,					(-rotors[1].thrust_coef * rotors[1].position_y),	rotors[1].moment_ratio, 				rotors[2].moment_ratio, 				0.f,										0.f, 										0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{rotors[0].thrust_coef, 				0.f, 							rotors[1].thrust_coef, 					0.f, 							0.f, 							0.f,										0.f, 										0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 							0.f,							0.f, 							0.f, 							0.f, 							0.f,										0.f, 										0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{ 0.f, 							-rotors[0].thrust_coef, 				0.f,							-rotors[1].thrust_coef, 				-rotors[2].thrust_coef, 				0.f,										0.f, 										0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
	};
	matrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>(tiltrotor_vtol_combined_modes);

	_updated = false;
	return true;
}

void
ActuatorEffectivenessTiltrotorVTOLCombinedModes::setFlightPhase(const FlightPhase &flight_phase)
{
	ActuatorEffectiveness::setFlightPhase(flight_phase);

	_updated = true;
}

void
ActuatorEffectivenessTiltrotorVTOLCombinedModes::setAirspeed(const float true_airspeed)
{
	ActuatorEffectiveness::setAirspeed(true_airspeed);

	_updated = true;
}
