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
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#include "ControlAllocationNonlinearOptimization.hpp"
#include "bfgs_optimization.hpp"
#include <uORB/Subscription.hpp>
#include <functional>

using std::pow;

#define CA_ROTOR_RIGHT 0
#define CA_ROTOR_LEFT 1
#define CA_ROTOR_REAR 2
#define CA_SERVO_RIGHT 3
#define CA_SERVO_LEFT 4
#define CA_ELEVON_RIGHT 5
#define CA_ELEVON_LEFT 6

#define TILTROTOR_VTOL_COMBINED_MODES 3
#define QUADPLANE_COMBINED_MODES 4

/**
 * If thrust is outside of the range achievable by the servos, projects the thrust onto an
 * achievable axis.
 * @param thrustTorqueDesired will be updated with new thrustTorqueDesired
 * @param actuators current actuator setpoints used to calculate lift and drag forces
 */
void
ControlAllocationNonlinearOptimization::getAchievableThrust(
    matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueDesired,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> actuators,
    NonlinearEffectiveness_ControlAllocationData *controlAllocationData) {
    float tilt_servo_max = _param_ca_tlt_servo_max.get() * (float) M_PI / 180.f;
    float tilt_servo_min = _param_ca_tlt_servo_min.get() * (float) M_PI / 180.f;

    // Get the thrust needed to be produced by rotors
    matrix::Vector2f thrustDesired((*thrustTorqueDesired)(0), (*thrustTorqueDesired)(1));
    matrix::Vector2f elevonForceCoefs = _nonlin_effectiveness->calcControlSurfaceForce(controlAllocationData);
    matrix::Vector2f forcesLiftDrag;
    forcesLiftDrag(0) = elevonForceCoefs(0) * (actuators(CA_ELEVON_RIGHT) + actuators(CA_ELEVON_LEFT));
    forcesLiftDrag(1) = elevonForceCoefs(1) * (actuators(CA_ELEVON_RIGHT) + actuators(CA_ELEVON_LEFT));
    matrix::Vector2f thrustDesiredMinusLift = thrustDesired - forcesLiftDrag;
    float thrust_angle = atan2f(-thrustDesiredMinusLift(1), thrustDesiredMinusLift(0));

    if (thrust_angle < tilt_servo_min)
        thrust_angle += 2.f * (float) M_PI;   // Looking at range min to 2pi+min
    if (thrust_angle > tilt_servo_max) {
        // if projected thrust vector would still be negative, just set thrust vector to 0 vector
        if (thrust_angle > tilt_servo_max + (float) M_PI / 2.f &&
            thrust_angle < tilt_servo_min - (float) M_PI / 2.f + 2.f * (float) M_PI) {
            (*thrustTorqueDesired)(0) = 0.f;
	        (*thrustTorqueDesired)(1) = 0.f;
        } else {
            float halfway_vector =          // Vector halfway between and not within actuator bounds
                tilt_servo_max + (tilt_servo_max - tilt_servo_min) / 2.f;
            matrix::Vector2f projection_unit_vec;
            matrix::Vector2f thrust_2d((*thrustTorqueDesired)(0), -(*thrustTorqueDesired)(1));
            if (thrust_angle < halfway_vector)  // project onto max angle
                projection_unit_vec = matrix::Vector2f(cosf(tilt_servo_max), sinf(tilt_servo_max));
            else                                // project onto min angle
                projection_unit_vec = matrix::Vector2f(cosf(tilt_servo_min), sinf(tilt_servo_min));
            // Project desired thrust onto achievable axis
            matrix::Vector2f achievable_thrust = thrust_2d.dot(projection_unit_vec) * projection_unit_vec;
            (*thrustTorqueDesired)(0) = achievable_thrust(0);
            (*thrustTorqueDesired)(1) = -achievable_thrust(1);
        }
    }
}

/**
 * Runs the nonlinear optimization
 * thrustTorqueDesired: desired thrust/torque
 * x0: Inital guess
 * vBody: velocity in the body frame (ideally should be in the airframe)
 * airspeed: vehicle airspeed
 * iterMax: maximum number of optimization iterations
 */
matrix::Vector<float, VTOL_NUM_ACTUATORS>
ControlAllocationNonlinearOptimization::computeNonlinearOpt(
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> x0,
    NonlinearEffectiveness_ControlAllocationData *controlAllocationData, size_t iterMax) {

    bfgs_settings_t settings;

    // TODO: need to use parameters for actuator bounds
    // right now just switch lowerBoundsf and upperBoundsf commented out lines
    float tiltServoMax = 115.f * (float) M_PI / 180.f;
    float lowerBoundsf[VTOL_NUM_ACTUATORS] = {0.00, 0.00, 0.00, 0.0, 0.0, -1.0, -1.0};
    float upperBoundsf[VTOL_NUM_ACTUATORS] = {1.0, 1.0, 1.0, tiltServoMax, tiltServoMax, 1.0, 1.0};
    // float lowerBoundsf[VTOL_NUM_ACTUATORS] = {0.00, 0.00, 0.00, 0.0, 0.0, -1.0, -1.0};
    // float upperBoundsf[VTOL_NUM_ACTUATORS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    matrix::Vector<float, VTOL_NUM_ACTUATORS> lowerBounds(lowerBoundsf);
    matrix::Vector<float, VTOL_NUM_ACTUATORS> upperBounds(upperBoundsf);
    settings.lower_bounds = lowerBounds;
    settings.upper_bounds = upperBounds;
    settings.iter_max = iterMax;

    std::function<float(const matrix::Vector<float, VTOL_NUM_ACTUATORS>&,
        matrix::Vector<float, VTOL_NUM_ACTUATORS>*,  void*)> f =
        std::bind(&ControlAllocationNonlinearOptimization::nonlinearCtrlOptFun, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);


    ctrlalloc_bfgs(x0, f, controlAllocationData, settings);

    return x0;
}

/**
 * initialGuessGuard guards against an actuator becoming locked at saturation
 */
void
ControlAllocationNonlinearOptimization::initialGuessGuard() {
    for (uint8_t i = 0; i < VTOL_NUM_ACTUATORS; ++i) {
        if (_initGuessGuardCnt(i) > 20) {
            if (_actuator_sp(i) > (_actuator_max(i) - ((_actuator_max(i) - _actuator_min(i)) * 0.01f))) {
                _actuator_sp(i) = _actuator_max(i) - ((_actuator_max(i) - _actuator_min(i)) * 0.05f);
            } else {
                _actuator_sp(i) = _actuator_min(i) + ((_actuator_max(i) - _actuator_min(i)) * 0.05f);
            }
            _initGuessGuardCnt(i) = 0;
        }
    }
    // checks whether the actuators have not been set yet
    for (uint8_t i = 0; i < VTOL_NUM_ACTUATORS; ++i) {
        if (!(_actuator_sp(i) < 0.0001f && _actuator_sp(i) > -0.0001f))
            break;
        if (i == (VTOL_NUM_ACTUATORS - 1))
            for(uint8_t j = 0; j < VTOL_NUM_ACTUATORS; ++j) {
                _actuator_sp(j) = (_actuator_max(j) + _actuator_min(j)) / 2.f;
            }
    }
}

void
ControlAllocationNonlinearOptimization::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_trim, int num_actuators)
{
	// does nothing

}

void
ControlAllocationNonlinearOptimization::allocate()
{
	matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired;
	matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueAchieved;
	matrix::Vector<float, VTOL_NUM_ACTUATORS> solution_d1;
	matrix::Vector3f vBody = _q.conjugate_inversed(_interial_velocity);

	thrustTorqueDesired(0) = _control_sp(3);
	thrustTorqueDesired(1) = _control_sp(5);
	thrustTorqueDesired(2) = _control_sp(0);
	thrustTorqueDesired(3) = _control_sp(1);
	thrustTorqueDesired(4) = _control_sp(2);

    initialGuessGuard();

    solution_d1 = _nonlin_effectiveness->getSolutionFromActuatorSp(_actuator_sp);

    // Set up NonlinearEffectiveness_ControlAllocationData
    NonlinearEffectiveness_ControlAllocationData controlAllocationData;
    controlAllocationData.Gamma = .5f * _air_density * powf(_airspeed, 2.f) * _param_ca_wing_area.get();
    controlAllocationData.VaRear = (matrix::Vector3f(0.0, 0.0, -1.0).transpose() * vBody)(0, 0);
    controlAllocationData.thrustTorqueDesired = thrustTorqueDesired;
    controlAllocationData.vBody = vBody;
    controlAllocationData.airDensity = _air_density;
    controlAllocationData.airspeed = _airspeed;

	matrix::Vector<float, VTOL_NUM_ACTUATORS> solution = computeNonlinearOpt(thrustTorqueDesired, solution_d1, &controlAllocationData, _param_ca_nonlin_iter_max.get());
	_nonlin_effectiveness->calcThrustTorqueAchieved(&thrustTorqueAchieved, solution, vBody, _airspeed, _air_density);

    _actuator_sp = _nonlin_effectiveness->getActuatorSpFromSolution(solution);

	_control_allocated(0) = thrustTorqueAchieved(2);
	_control_allocated(1) = thrustTorqueAchieved(3);
	_control_allocated(2) = thrustTorqueAchieved(4);
	_control_allocated(3) = thrustTorqueAchieved(0);
	_control_allocated(4) = 0.f;
	_control_allocated(5) = thrustTorqueAchieved(1);

    // BFGS algorithm can get stuck at saturation limits sometimes. initGuessGuard helps protect against that
    for (uint8_t i = 0; i < VTOL_NUM_ACTUATORS; ++i) {
        if (_actuator_sp(i) > (_actuator_max(i) - ((_actuator_max(i) - _actuator_min(i)) * 0.01f)) ||
            _actuator_sp(i) < (_actuator_min(i) + ((_actuator_max(i) - _actuator_min(i)) * 0.01f)))
            _initGuessGuardCnt(i) += 1;
        else
            _initGuessGuardCnt(i) = 0;
    }

}

ControlAllocationNonlinearOptimization::ControlAllocationNonlinearOptimization():
	ModuleParams(nullptr)
{

}

void ControlAllocationNonlinearOptimization::setNonlinearEffectiveness(size_t effectivenessID) {
    if (effectivenessID == TILTROTOR_VTOL_COMBINED_MODES) {
        _nonlin_effectiveness = new NonlinearEffectivenessVTOLTiltrotor();
    } else if (effectivenessID == QUADPLANE_COMBINED_MODES) {
        _nonlin_effectiveness = new NonlinearEffectivenessVTOLQuadplane();
    }
}
