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
 * @file ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes.hpp
 *
 * Adds addition computation to the PseudoInverse class in order to determine
 * tiltrotor positions based off of the effectiveness matrix.
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#pragma once

#include "ControlAllocationPseudoInverse.hpp"
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>

class ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes: public ModuleParams, public ControlAllocationPseudoInverse
{
public:
	ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes(): ModuleParams(nullptr) {}
	virtual ~ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes() = default;

	virtual void allocate() override {
        float deg2rad = (float) M_PI / 180.f;
        float tilt_servo_max = _param_ca_tlt_srvo_max.get() * deg2rad;
        // float elevon_max = _param_ca_elevon_max.get() * deg2rad;

        //Compute new gains if needed
        updatePseudoInverse();

        // for(uint8_t i = 0; i < 16; ++i)
        //     PX4_INFO("mix[%d]: %.5f %.5f %.5f %.5f %.5f %.5f", i,
        //     (double) _mix(i, 0), (double) _mix(i, 1), (double) _mix(i, 2), (double) _mix(i, 3), (double) _mix(i, 4), (double) _mix(i, 5));
        // for(uint8_t i = 0; i < 6; ++i)
        //     PX4_INFO("effectiveness_matrix[%d]: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f", i,
        //     (double) _effectiveness(i, 0),
        //     (double) _effectiveness(i, 1),
        //     (double) _effectiveness(i, 2),
        //     (double) _effectiveness(i, 3),
        //     (double) _effectiveness(i, 4),
        //     (double) _effectiveness(i, 5),
        //     (double) _effectiveness(i, 6),
        //     (double) _effectiveness(i, 7),
        //     (double) _effectiveness(i, 8),
        //     (double) _effectiveness(i, 9),
        //     (double) _effectiveness(i, 10),
        //     (double) _effectiveness(i, 11),
        //     (double) _effectiveness(i, 12),
        //     (double) _effectiveness(i, 13),
        //     (double) _effectiveness(i, 14),
        //     (double) _effectiveness(i, 15)
        //     );

        // Allocate
        matrix::Vector<float, NUM_ACTUATORS> mix_solution =
            _actuator_trim + _mix * (_control_sp - _control_trim);
        // PX4_INFO("mix_solution: %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f",
        //     (double) mix_solution(0),
        //     (double) mix_solution(1),
        //     (double) mix_solution(2),
        //     (double) mix_solution(3),
        //     (double) mix_solution(4),
        //     (double) mix_solution(5),
        //     (double) mix_solution(6),
        //     (double) mix_solution(7));
        // PX4_INFO("trig_stuff: %.5f %.5f", (double) (atan2f(mix_solution(1), mix_solution(0))), (double) (atan2f(mix_solution(3), mix_solution(2))));

        _actuator_sp(0) = sqrt(mix_solution(0) * mix_solution(0) + mix_solution(1) * mix_solution(1)); //1.0f;
        _actuator_sp(1) = sqrt(mix_solution(2) * mix_solution(2) + mix_solution(3) * mix_solution(3)); //1.0f;
        _actuator_sp(2) = mix_solution(4); //1.0f;
        _actuator_sp(3) = 0.f; //0.f;
        _actuator_sp(4) = ((atan2f(mix_solution(1), mix_solution(0)) / (tilt_servo_max))); //.7826f;
        _actuator_sp(5) = 1.f - ((atan2f(mix_solution(3), mix_solution(2)) / (tilt_servo_max))); //.2174f;
        _actuator_sp(6) = mix_solution(5); //mix_solution(5) / (elevon_max) + .5f; //0.f;
        _actuator_sp(7) = mix_solution(6); //mix_solution(6) / (elevon_max) + .5f; //0.f;


        // PX4_INFO("actuator_setpoint: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f", (double) _actuator_sp(0), (double) _actuator_sp(1), (double) _actuator_sp(2),
		//     (double) _actuator_sp(3), (double) _actuator_sp(4), (double) _actuator_sp(5), (double) _actuator_sp(6), (double) _actuator_sp(7));

        // Clip
        clipActuatorSetpoint(_actuator_sp);

        matrix::Vector<float, NUM_ACTUATORS> clipped_mix_solution;
        clipped_mix_solution(0) = _actuator_sp(0) / (float) sqrt(1 + pow(2, tanf(tilt_servo_max * _actuator_sp(4))));
        clipped_mix_solution(1) = _actuator_sp(0) / (float) sqrt(1 + 1 / pow(2, tanf(tilt_servo_max * _actuator_sp(4))));
        clipped_mix_solution(2) = _actuator_sp(1) / (float) sqrt(1 + pow(2, tanf(tilt_servo_max * (1.f - _actuator_sp(5)))));
        clipped_mix_solution(3) = _actuator_sp(1) / (float) sqrt(1 + 1 / pow(2, tanf(tilt_servo_max * (1.f - _actuator_sp(5)))));
        clipped_mix_solution(4) = _actuator_sp(2);
        clipped_mix_solution(5) = _actuator_sp(6); //(_actuator_sp(6) - .5f) * elevon_max;
        clipped_mix_solution(6) = _actuator_sp(6); //(_actuator_sp(7) - .5f) * elevon_max;

        // Compute achieved control
        _control_allocated = _effectiveness * clipped_mix_solution;
    }

protected:

    DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_ELEVON_MAX>) _param_ca_elevon_max,
		(ParamFloat<px4::params::CA_TLT_SRVO_MAX>) _param_ca_tlt_srvo_max
	)
};
