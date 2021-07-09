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

class ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes: public ControlAllocationPseudoInverse
{
public:
	ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes() = default;
	virtual ~ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes() = default;

	virtual void allocate() override {
        //Compute new gains if needed
        updatePseudoInverse();

        // Allocate
        matrix::Vector<float, NUM_ACTUATORS> mix_solution = 
            _actuator_trim + _mix * (_control_sp - _control_trim);

        _actuator_sp(0) = sqrt(pow(2, mix_solution(0)) + pow(2, mix_solution(1)));
        _actuator_sp(1) = sqrt(pow(2, mix_solution(2)) + pow(2, mix_solution(3)));
        _actuator_sp(2) = mix_solution(5);
        _actuator_sp(3) = 0.f;
        _actuator_sp(4) = atan2f(mix_solution(1), mix_solution(0));
        _actuator_sp(5) = atan2f(mix_solution(3), mix_solution(2));
        _actuator_sp(6) = mix_solution(6);
        _actuator_sp(7) = mix_solution(7);

        // Clip
        clipActuatorSetpoint(_actuator_sp);

        // Compute achieved control
        _control_allocated = _effectiveness * mix_solution;
    }
};
