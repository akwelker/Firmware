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
 * @file NonlinearEffectivenessVTOLTiltrotor.cpp
 *
 * Nonlinear functions for calculating optimal actuator setpoint for a tiltrotor
 * being flown in continuous mode (no separate hover/fixed-wing flight)
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#include "NonlinearEffectivenessVTOLTiltrotor.hpp"
#include <px4_log.h>

#define CA_ROTOR_RIGHT 0
#define CA_ROTOR_LEFT 1
#define CA_ROTOR_REAR 2
#define CA_SERVO_RIGHT 3
#define CA_SERVO_LEFT 4
#define CA_ELEVON_RIGHT 5
#define CA_ELEVON_LEFT 6

matrix::Vector<float, VTOL_NUM_ACTUATORS>
NonlinearEffectivenessVTOLTiltrotor::getSolutionFromActuatorSp(
	matrix::Vector<float, 16> &actuator_sp)
{
	matrix::Vector<float, VTOL_NUM_ACTUATORS> solution;
	float tiltServoMax = _param_ca_tlt_servo_max.get() * (float) M_PI / 180.f;
	// float elevonMax = _param_ca_surf0_max_ang.get() * (float) M_PI / 180.f;


	solution(0) = actuator_sp(0);
	solution(1) = actuator_sp(1);
	solution(2) = actuator_sp(2);
	solution(3) = actuator_sp(4) * tiltServoMax;
	solution(4) = (1.f - actuator_sp(5)) * tiltServoMax;
	solution(5) = -actuator_sp(6);// * elevonMax; // positive right elevon value makes elevon go up. Needs to be negated
	solution(6) = actuator_sp(7);// * elevonMax;
    solution(7) = 0.f; // empty

    return solution;
}

matrix::Vector<float, 16>
NonlinearEffectivenessVTOLTiltrotor::getActuatorSpFromSolution(
	matrix::Vector<float, VTOL_NUM_ACTUATORS> &solution)
{
	matrix::Vector<float, 16> actuator_sp;
	float tiltServoMax = _param_ca_tlt_servo_max.get() * (float) M_PI / 180.f;
	// float elevonMax = _param_ca_surf0_max_ang.get() * (float) M_PI / 180.f;

	actuator_sp(0) = solution(0);
	actuator_sp(1) = solution(1);
	actuator_sp(2) = solution(2);
	actuator_sp(3) = 0.f;
	actuator_sp(4) = solution(3) / tiltServoMax;
	actuator_sp(5) = 1.f - (solution(4) / tiltServoMax);
	actuator_sp(6) = -solution(5);// / elevonMax; // positive right elevon value makes elevon go up. Needs to be negated
	actuator_sp(7) = solution(6);// / elevonMax;

    return actuator_sp;
}

/**
 * Calculates forces produced by elevons
 * @param data ControlAllocationData function data used by minimization function
 * @returns 2-dimensional elevon force vector representing the force produced
 * by each vector.
 */
matrix::Vector2f
NonlinearEffectivenessVTOLTiltrotor::calcControlSurfaceForce(struct NonlinearEffectiveness_ControlAllocationData *data) {
    float liftCoef = data->Gamma * _param_ca_C_L_delta_e.get();
    float dragCoef = data->Gamma * _param_ca_C_D_delta_e.get();
    float alpha;
    if (data->vBody(0) > -0.00001f && data->vBody(0) < 0.00001f) {
        alpha = atan2f(data->vBody(2), data->vBody(0));
    } else {
        alpha = 0.f;
    }
    matrix::Vector2f forceCoefs(-cosf(alpha) * dragCoef + sinf(alpha) * liftCoef,
                                      -sinf(alpha) * dragCoef - cosf(alpha) * liftCoef);
    return forceCoefs;
}

/**
 * Calculates the thrust and torque achieved by a given set of rotor/servo/elevon setpoints
 * thrustTorqueAchieved: Will contain the thrust and torque achieved
 * x: Actuator setpoints
 * vBody: velocity in the body frame
 * airspeed: vehicle airspeed
 */
void
NonlinearEffectivenessVTOLTiltrotor::calcThrustTorqueAchieved(
    matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> &x, matrix::Vector3f vBody, float airspeed, float airDensity)
{
	float x_0 = std::cos(x(CA_SERVO_RIGHT));
    	float z_0 = std::sin(x(CA_SERVO_RIGHT));
    	float x_1 = std::cos(x(CA_SERVO_LEFT));
    	float z_1 = std::sin(x(CA_SERVO_LEFT));

	float Gamma = .5f * airDensity * pow(airspeed, 2.f) * _param_ca_wing_area.get();
	float VaRear = (matrix::Vector3f(0.0, 0.0, -1.0).transpose() * vBody)(0, 0);
	float VaRight = (matrix::Vector3f(x_0, 0.0, -z_0).transpose() * vBody)(0, 0);
	float VaLeft = (matrix::Vector3f(x_1, 0.0, -z_1).transpose() * vBody)(0, 0);

	float thrustTorqueDersRotor0[4];
	float thrustTorqueDersRotor1[4];
	float thrustTorqueDersRotor2[4];

	rotorThrustTorque(thrustTorqueDersRotor0, x(0), VaRight, airDensity, _rotor_params[0]);
	rotorThrustTorque(thrustTorqueDersRotor1, x(1), VaLeft, airDensity, _rotor_params[1]);
	rotorThrustTorque(thrustTorqueDersRotor2, x(2), VaRear, airDensity, _rotor_params[2]);

	float thrust[3] = {thrustTorqueDersRotor0[0], thrustTorqueDersRotor1[0], thrustTorqueDersRotor2[0]};
	float torque[3] = {thrustTorqueDersRotor0[1], thrustTorqueDersRotor1[1], thrustTorqueDersRotor2[1]};

    	NonlinearEffectiveness_ControlAllocationData controlAllocationData;
    	controlAllocationData.Gamma = Gamma;
    	controlAllocationData.vBody = vBody;
    	matrix::Vector2f elevonForceCoefs = calcControlSurfaceForce(&controlAllocationData);

	calcThrustTorqueAchieved(thrustTorqueAchieved, thrust, torque, elevonForceCoefs, x, Gamma);

    return;
}

/**
 * Optimization function that is used to calculate the norm of the difference between desired thrust/torque
 * and achieved thrust/torque, as well as that function's gradient.
 * vals_inp: actuator setpoints, the input to the function
 * grad_out: The gradient of the function at the vals_inp point
 * opt_data: Extra arguments to the function (Gamma, VaRear, vBody, and thrustTorqueDesired)
 */
float
NonlinearEffectivenessVTOLTiltrotor::nonlinearCtrlOptFun(
    const matrix::Vector<float, VTOL_NUM_ACTUATORS>& vals_inp,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> *grad_out, void *opt_data) {

    NonlinearEffectiveness_ControlAllocationData* controlAllocationData =
    	reinterpret_cast<NonlinearEffectiveness_ControlAllocationData*>(opt_data);

    float x_0 = std::cos(vals_inp(CA_SERVO_RIGHT));
    float z_0 = std::sin(vals_inp(CA_SERVO_RIGHT));
    float x_1 = std::cos(vals_inp(CA_SERVO_LEFT));
    float z_1 = std::sin(vals_inp(CA_SERVO_LEFT));
    matrix::Matrix<float, VTOL_NUM_AXES, VTOL_NUM_AXES> K;
    matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_ACTUATORS> K_delta;
    K.setIdentity();
    float wrenchMin = 1.0f;
    K(0, 0) = abs(controlAllocationData->thrustTorqueDesired(0)) > wrenchMin ?
        1.f / powf(controlAllocationData->thrustTorqueDesired(0), 2.f) : powf(wrenchMin, 2.f);
    K(1, 1) = abs(controlAllocationData->thrustTorqueDesired(1)) > wrenchMin ?
        1.f / powf(controlAllocationData->thrustTorqueDesired(1), 2.f) : powf(wrenchMin, 2.f);
    K(2, 2) = abs(controlAllocationData->thrustTorqueDesired(2)) > wrenchMin ?
        1.f / powf(controlAllocationData->thrustTorqueDesired(2), 2.f) : powf(wrenchMin, 2.f);
    K(3, 3) = abs(controlAllocationData->thrustTorqueDesired(3)) > wrenchMin ?
        1.f / powf(controlAllocationData->thrustTorqueDesired(3), 2.f) : powf(wrenchMin, 2.f);
    K(4, 4) = abs(controlAllocationData->thrustTorqueDesired(4)) > wrenchMin ?
        1.f / powf(controlAllocationData->thrustTorqueDesired(4), 2.f) : powf(wrenchMin, 2.f);
    K_delta(0, 0) = 10.0f;
    K_delta(1, 1) = 10.0f;
    K_delta(2, 2) = 10.0f;
    K_delta(3, 3) = .03f;
    K_delta(4, 4) = .03f;
    K_delta = K_delta * powf(controlAllocationData->airspeed, 2.f) * 1e-6;

    float VaRight = (matrix::Vector3f(x_0, 0.0, -z_0).transpose() * controlAllocationData->vBody)(0, 0);
    float VaLeft = (matrix::Vector3f(x_1, 0.0, -z_1).transpose() * controlAllocationData->vBody)(0, 0);

    float thrustTorqueDersRotor0[4];
    float thrustTorqueDersRotor1[4];
    float thrustTorqueDersRotor2[4];

    rotorThrustTorque(thrustTorqueDersRotor0, vals_inp(0), VaRight, controlAllocationData->airDensity, _rotor_params[0]);
    rotorThrustTorque(thrustTorqueDersRotor1, vals_inp(1), VaLeft, controlAllocationData->airDensity, _rotor_params[1]);
    rotorThrustTorque(thrustTorqueDersRotor2, vals_inp(2), controlAllocationData->VaRear, controlAllocationData->airDensity, _rotor_params[2]);

    float thrust[3] = {thrustTorqueDersRotor0[0], thrustTorqueDersRotor1[0], thrustTorqueDersRotor2[0]};
    float torque[3] = {thrustTorqueDersRotor0[1], thrustTorqueDersRotor1[1], thrustTorqueDersRotor2[1]};
    float thrustDer[3] = {thrustTorqueDersRotor0[2], thrustTorqueDersRotor1[2], thrustTorqueDersRotor2[2]};
    float torqueDer[3] = {thrustTorqueDersRotor0[3], thrustTorqueDersRotor1[3], thrustTorqueDersRotor2[3]};
    matrix::Vector2f elevonForceCoefs = calcControlSurfaceForce(controlAllocationData);

    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueAchieved;
    calcThrustTorqueAchieved(&thrustTorqueAchieved, thrust, torque, elevonForceCoefs, vals_inp, controlAllocationData->Gamma);
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDiff = controlAllocationData->thrustTorqueDesired - thrustTorqueAchieved;
    float normDiff = (0.5f * thrustTorqueDiff.transpose() * K * thrustTorqueDiff)(0, 0);
    matrix::Vector3f rotorOutput(vals_inp(CA_ROTOR_RIGHT), vals_inp(CA_ROTOR_LEFT), vals_inp(CA_ROTOR_REAR));
    normDiff += 0.5f * (vals_inp.transpose() * K_delta * vals_inp)(0, 0);

    matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> thrustTorqueDer;
    calcThrustTorqueAchievedDer(&thrustTorqueDer, thrust, torque, thrustDer, torqueDer,
        elevonForceCoefs, vals_inp, controlAllocationData->Gamma);
    matrix::Vector<float, VTOL_NUM_ACTUATORS> normDiffDer = -1.f * thrustTorqueDer * K * thrustTorqueDiff;
    normDiffDer += K_delta * vals_inp;

    if (grad_out) {
        *grad_out = normDiffDer;
    }

    return normDiff;
}

/**
 * Calculates the thrust and torque achieved by a given set of rotor/servo/elevon setpoints
 * thrustTorqueAchieved: Will contain the thrust and torque achieved
 * thrust: thrust array (length 3) of each of the rotors
 * torque: torque array (length 3) of each of the rotors
 * x: Actuator setpoints
 * Gamma: Term used to determine elevon effectiveness
 */
void
NonlinearEffectivenessVTOLTiltrotor::calcThrustTorqueAchieved(
    matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    float *thrust, float *torque, matrix::Vector2f elevonForceCoefs,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma) {
    float x_0 = std::cos(x(CA_SERVO_RIGHT));
    float z_0 = std::sin(x(CA_SERVO_RIGHT));
    float x_1 = std::cos(x(CA_SERVO_LEFT));
    float z_1 = std::sin(x(CA_SERVO_LEFT));

    float T_x =  (thrust[0] * x_0) + (thrust[1] * x_1) +
        elevonForceCoefs(0) * (x(CA_ELEVON_RIGHT) + x(CA_ELEVON_LEFT));
    float T_z = -(thrust[0] * z_0) - (thrust[1] * z_1) - thrust[2] +
        elevonForceCoefs(1) * (x(CA_ELEVON_RIGHT) + x(CA_ELEVON_LEFT));
    float Tau_x =   - (torque[0] * x_0) - (thrust[0] * _rotor_params[0].position_y * z_0)
                    - (torque[1] * x_1) - (thrust[1] * _rotor_params[1].position_y * z_1)
                    - (thrust[2] * _rotor_params[2].position_y)
                    - Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get() * x(CA_ELEVON_RIGHT)
                    + Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get() * x(CA_ELEVON_LEFT);
    float Tau_y =   thrust[0] * (_rotor_params[0].position_z * x_0 + _rotor_params[0].position_x * z_0) +
                    thrust[1] * (_rotor_params[1].position_z * x_1 + _rotor_params[1].position_x * z_1) +
                    thrust[2] * _rotor_params[2].position_x +
                    Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get() * x(CA_ELEVON_RIGHT) +
                    Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get() * x(CA_ELEVON_LEFT);
    float Tau_z =   - (thrust[0] * _rotor_params[0].position_y * x_0) + (torque[0] * z_0)
                    - (thrust[1] * _rotor_params[1].position_y * x_1) + (torque[1] * z_1)
                    + torque[2];

    (*thrustTorqueAchieved)(0) = T_x;
    (*thrustTorqueAchieved)(1) = T_z;
    (*thrustTorqueAchieved)(2) = Tau_x;
    (*thrustTorqueAchieved)(3) = Tau_y;
    (*thrustTorqueAchieved)(4) = Tau_z;
}

/**
 * Calculates the thrust and torque achieved by a given set of rotor/servo/elevon setpoints
 * thrustTorqueAchieved: Will contain the thrust and torque achieved
 * thrust: thrust array (length 3) of each of the rotors
 * torque: torque array (length 3) of each of the rotors
 * thrustDer: thrust derivative array (length 3) of each of the rotors
 * torqueDer: torque derivative array (length 3) of each of the rotors
 * x: Actuator setpoints
 * Gamma: Term used to determine elevon effectiveness
 */
void
NonlinearEffectivenessVTOLTiltrotor::calcThrustTorqueAchievedDer(
    matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> *thrustTorqueAchievedDer,
    float *thrust, float *torque, float *thrustDer, float *torqueDer,
    matrix::Vector2f elevonForceCoefs, matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma) {
    float x_0 = std::cos(x(CA_SERVO_RIGHT));
    float z_0 = std::sin(x(CA_SERVO_RIGHT));
    float x_1 = std::cos(x(CA_SERVO_LEFT));
    float z_1 = std::sin(x(CA_SERVO_LEFT));

    matrix::Vector<float, VTOL_NUM_ACTUATORS> T_x_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> T_z_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> Tau_x_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> Tau_y_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> Tau_z_der;

    T_x_der(0) = thrustDer[0] * x_0;
    T_x_der(1) = thrustDer[1] * x_1;
    T_x_der(2) = 0;
    T_x_der(3) = -thrust[0] * z_0;
    T_x_der(4) = -thrust[1] * z_1;
    T_x_der(5) = elevonForceCoefs(0);
    T_x_der(6) = elevonForceCoefs(0);

    T_z_der(0) = -thrustDer[0] * z_0;
    T_z_der(1) = -thrustDer[1] * z_1;
    T_z_der(2) = -thrustDer[2];
    T_z_der(3) = -thrust[0] * x_0;
    T_z_der(4) = -thrust[1] * x_1;
    T_z_der(5) = elevonForceCoefs(1);
    T_z_der(6) = elevonForceCoefs(1);

    Tau_x_der(0) = -(torqueDer[0] * x_0) - (thrustDer[0] * _rotor_params[0].position_y * z_0);
    Tau_x_der(1) = -(torqueDer[1] * x_1) - (thrustDer[1] * _rotor_params[1].position_y * z_1);
    Tau_x_der(2) = - (thrustDer[2] * _rotor_params[2].position_y);
    Tau_x_der(3) = torque[0] * z_0 - thrust[0] * _rotor_params[0].position_y * x_0;
    Tau_x_der(4) = torque[1] * z_1 - thrust[1] * _rotor_params[1].position_y * x_1;
    Tau_x_der(5) = - Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get();
    Tau_x_der(6) = + Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get();

    Tau_y_der(0) = thrustDer[0] * (_rotor_params[0].position_z * x_0 + _rotor_params[0].position_x * z_0);
    Tau_y_der(1) = thrustDer[1] * (_rotor_params[1].position_z * x_1 + _rotor_params[1].position_x * z_1);
    Tau_y_der(2) = thrustDer[2] * _rotor_params[2].position_x;
    Tau_y_der(3) = thrust[0] * (-_rotor_params[0].position_z * z_0 + _rotor_params[0].position_x * x_0);
    Tau_y_der(4) = thrust[1] * (-_rotor_params[1].position_z * z_1 + _rotor_params[1].position_x * x_1);
    Tau_y_der(5) = Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get();
    Tau_y_der(6) = Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get();

    Tau_z_der(0) = -(thrustDer[0] * _rotor_params[0].position_y * x_0) + (torqueDer[0] * z_0);
    Tau_z_der(1) = -(thrustDer[1] * _rotor_params[1].position_y * x_1) + (torqueDer[1] * z_1);
    Tau_z_der(2) = torqueDer[2];
    Tau_z_der(3) = thrust[0] * _rotor_params[0].position_y * z_0 + torque[0] * x_0;
    Tau_z_der(4) = thrust[1] * _rotor_params[1].position_y * z_1 + torque[1] * x_1;
    Tau_z_der(5) = 0;
    Tau_z_der(6) = 0;

    thrustTorqueAchievedDer->setCol(0, T_x_der);
    thrustTorqueAchievedDer->setCol(1, T_z_der);
    thrustTorqueAchievedDer->setCol(2, Tau_x_der);
    thrustTorqueAchievedDer->setCol(3, Tau_y_der);
    thrustTorqueAchievedDer->setCol(4, Tau_z_der);
}
