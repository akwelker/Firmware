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

class ControlAllocationNonlinearOptimization: public ControlAllocation
{
public:
	ControlAllocationNonlinearOptimization() = default;
	virtual ~ControlAllocationNonlinearOptimization() = default;

	virtual void allocate() override;
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const matrix::Vector<float, NUM_ACTUATORS> &actuator_trim, int num_actuators) override;

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
	}

protected:

	/**
	 * Guards against actuators getting stuck at their limits. A little hacky.
	 */
	void initialGuessGuard();

	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> _mix;
	matrix::Vector<float, 7> _initGuessGuardCnt;

	bool _mix_update_needed{false};
	bool _set_init_actuators{true};
	matrix::Quatf _q{1.f, 0.f, 0.f, 0.f};
	matrix::Vector3f _interial_velocity{0.f, 0.f, 0.f};
	float _air_density{1.225f};

};
