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
 * @file NonlinearEffectiveness.hpp
 *
 * Used in conjunction with the ControlAllocationNonlinearOptimization to
 * find optimal actuator setpoints in order to achieve a desired thrust/torque using
 * a nonlinear thrust/torque rotor model.
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include "ActuatorEffectiveness.hpp"

#define VTOL_NUM_AXES 5
#define VTOL_NUM_ACTUATORS 8

/**
 * ControlAllocationData contains information to be passed into nonlinear minimization function
 */
struct NonlinearEffectiveness_ControlAllocationData {
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired;
    matrix::Vector3f vBody;
    float Gamma;
    float VaRear;
    float servoMax;
    float airspeed;
    float airDensity;
};

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
	float V_max;
	float KV;
	float KQ;
	float resistance;
	float i0;
	bool CW; // clockwise
} NonlinearEffectiveness_RotorParams;

class NonlinearEffectiveness : public ActuatorEffectiveness
{
public:
	NonlinearEffectiveness() = default;
	virtual ~NonlinearEffectiveness() = default;

	/**
	 * Set the airspeed
	 *
	 * @param airspeed Vehicle's airspeed
	 */
	void setAirspeed(const float airspeed)
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

	virtual matrix::Vector<float, VTOL_NUM_ACTUATORS>
	getSolutionFromActuatorSp(matrix::Vector<float, 16> &actuator_sp);

	virtual matrix::Vector<float, 16>
	getActuatorSpFromSolution(matrix::Vector<float, VTOL_NUM_ACTUATORS> &solution);

	virtual void calcThrustTorqueAchieved(
	    	matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
	    	matrix::Vector<float, VTOL_NUM_ACTUATORS> &x,
		matrix::Vector3f vBody, float airspeed, float airDensity);

	virtual float
	nonlinearCtrlOptFun(
		const matrix::Vector<float, VTOL_NUM_ACTUATORS>& vals_inp,
		matrix::Vector<float, VTOL_NUM_ACTUATORS> *grad_out,
		void *opt_data);

	virtual matrix::Vector2f calcControlSurfaceForce(
		struct NonlinearEffectiveness_ControlAllocationData* data);

	/**
	 * Get the number of actuators
	 */
	int numActuators() const override {
		return VTOL_NUM_ACTUATORS;
	}

	/**
	 * Get the control effectiveness matrix if updated
	 *
	 * @return true if updated and matrix is set
	 */
	bool getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix) override {
		return true;
	}

protected:


	void rotorThrustTorque(float *thrustTorqueDers, float delta, float Va, float airDensity,
		NonlinearEffectiveness_RotorParams &rotor);
	virtual void calcThrustTorqueAchieved(matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    		float *thrust, float *torque, matrix::Vector2f elevonForceCoefs,
    		matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma);

	virtual void calcThrustTorqueAchievedDer(
    		matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> *thrustTorqueAchievedDer,
    		float *thrust, float *torque, float *thrustDer, float *torqueDer,
    		matrix::Vector2f elevonForceCoefs, matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma);

	matrix::Quatf _q{1.f, 0.f, 0.f, 0.f};
	matrix::Vector3f _interial_velocity{0.f, 0.f, 0.f};
	float _air_density{1.225f};
	float _airspeed{0.f};

};
