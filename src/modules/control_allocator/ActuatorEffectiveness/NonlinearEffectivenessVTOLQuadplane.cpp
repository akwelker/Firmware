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
 * @file NonlinearEffectivenessVTOLQuadplane.cpp
 *
 * Nonlinear functions for calculating optimal actuator setpoint for a tiltrotor
 * being flown in continuous mode (no separate hover/fixed-wing flight)
 *
 * @author Mason Peterson <mbpeterson70@gmail.com>
 */

#include "NonlinearEffectivenessVTOLQuadplane.hpp"
#include <px4_log.h>

#define CA_ROTOR_FR 0
#define CA_ROTOR_FL 1
#define CA_ROTOR_BR 2
#define CA_ROTOR_BL 3
#define CA_ROTOR_PUSH 4
#define CA_ELEVON_RIGHT 5
#define CA_ELEVON_LEFT 6

matrix::Vector<float, VTOL_NUM_ACTUATORS>
NonlinearEffectivenessVTOLQuadplane::getSolutionFromActuatorSp(
	matrix::Vector<float, 16> &actuator_sp)
{
	matrix::Vector<float, VTOL_NUM_ACTUATORS> solution;
	// float elevonMax = _param_ca_surf0_max_ang.get() * (float) M_PI / 180.f;


	solution(0) = actuator_sp(0);
	solution(1) = actuator_sp(1);
	solution(2) = actuator_sp(2);
	solution(3) = actuator_sp(3);
	solution(4) = actuator_sp(4);
	solution(5) = -actuator_sp(6);// * elevonMax; // positive right elevon value makes elevon go up. Needs to be negated
	solution(6) = actuator_sp(7);// * elevonMax;
    solution(7) = 0.f; // empty

    return solution;
}

matrix::Vector<float, 16>
NonlinearEffectivenessVTOLQuadplane::getActuatorSpFromSolution(
	matrix::Vector<float, VTOL_NUM_ACTUATORS> &solution)
{
	matrix::Vector<float, 16> actuator_sp;
	// float elevonMax = _param_ca_surf0_max_ang.get() * (float) M_PI / 180.f;

	actuator_sp(0) = solution(0);
	actuator_sp(1) = solution(1);
	actuator_sp(2) = solution(2);
	actuator_sp(3) = solution(3);
	actuator_sp(4) = solution(4);
	actuator_sp(5) = 0.f;
	actuator_sp(6) = -solution(5);// / elevonMax; // positive right elevon value makes elevon go up. Needs to be negated
	actuator_sp(7) = solution(6); // elevonMax;

    return actuator_sp;
}

/**
 * Calculates forces produced by elevons
 * @param data ControlAllocationData function data used by minimization function
 * @returns 2-dimensional elevon force vector representing the force produced
 * by each vector.
 */
matrix::Vector2f
NonlinearEffectivenessVTOLQuadplane::calcControlSurfaceForce(struct NonlinearEffectiveness_ControlAllocationData *data) {
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
NonlinearEffectivenessVTOLQuadplane::calcThrustTorqueAchieved(
    matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> &x, matrix::Vector3f vBody, float airspeed, float airDensity)
{
	float Gamma = .5f * airDensity * pow(airspeed, 2.f) * _param_ca_wing_area.get();
    float Va[5];
    for (uint8_t i = 0; i < 4; ++i) {
        Va[i] = (matrix::Vector3f(0.0, 0.0, -1.0).transpose() * vBody)(0, 0);
    }
	Va[4] = (matrix::Vector3f(1.0, 0.0, 0.0).transpose() * vBody)(0, 0);

    float thrustTorqueDers[4];
    float thrust[5];
    float torque[5];

    for (uint8_t i = 0; i < 5; ++i) {
        rotorThrustTorque(thrustTorqueDers, x(i), Va[i], airDensity, _rotor_params[i]);
        thrust[i] = thrustTorqueDers[0];
        torque[i] = thrustTorqueDers[1];
    }

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
NonlinearEffectivenessVTOLQuadplane::nonlinearCtrlOptFun(
    const matrix::Vector<float, VTOL_NUM_ACTUATORS>& vals_inp,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> *grad_out, void *opt_data) {

    NonlinearEffectiveness_ControlAllocationData* controlAllocationData =
    	reinterpret_cast<NonlinearEffectiveness_ControlAllocationData*>(opt_data);

    matrix::Matrix<float, VTOL_NUM_AXES, VTOL_NUM_AXES> K;
    K.setIdentity();
    float Va[5];
    for (uint8_t i = 0; i < 4; ++i) {
        Va[i] = (matrix::Vector3f(0.0, 0.0, -1.0).transpose() * controlAllocationData->vBody)(0, 0);
    }
	Va[4] = (matrix::Vector3f(1.0, 0.0, 0.0).transpose() * controlAllocationData->vBody)(0, 0);

    float thrustTorqueDers[5][4];
    float thrust[5];
    float torque[5];
    float thrustDer[5];
    float torqueDer[5];

    for (uint8_t i = 0; i < 5; ++i) {
        rotorThrustTorque(thrustTorqueDers[i], vals_inp(i), Va[i], controlAllocationData->airDensity, _rotor_params[i]);
        thrust[i] = thrustTorqueDers[i][0];
        torque[i] = thrustTorqueDers[i][1];
        thrustDer[i] = thrustTorqueDers[i][2];
        torqueDer[i] = thrustTorqueDers[i][3];
    }

    matrix::Vector2f elevonForceCoefs = calcControlSurfaceForce(controlAllocationData);

    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueAchieved;
    calcThrustTorqueAchieved(&thrustTorqueAchieved, thrust, torque, elevonForceCoefs, vals_inp, controlAllocationData->Gamma);
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDiff = controlAllocationData->thrustTorqueDesired - thrustTorqueAchieved;
    float normDiff = (0.5f * thrustTorqueDiff.transpose() * K * thrustTorqueDiff)(0, 0);

    matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> thrustTorqueDer;
    calcThrustTorqueAchievedDer(&thrustTorqueDer, thrust, torque, thrustDer, torqueDer,
        elevonForceCoefs, vals_inp, controlAllocationData->Gamma);
    matrix::Vector<float, VTOL_NUM_ACTUATORS> normDiffDer = -thrustTorqueDer * K * thrustTorqueDiff;

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
NonlinearEffectivenessVTOLQuadplane::calcThrustTorqueAchieved(
    matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    float *thrust, float *torque, matrix::Vector2f elevonForceCoefs,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma) {

    float T_x =  thrust[4] +
        elevonForceCoefs(0) * (x(CA_ELEVON_RIGHT) + x(CA_ELEVON_LEFT));
    float T_z = -(thrust[0] + thrust[1] + thrust[2] + thrust[3]) +
        elevonForceCoefs(1) * (x(CA_ELEVON_RIGHT) + x(CA_ELEVON_LEFT));

    float Tau_x = - _rotor_params[0].position_y * thrust[0]
                  - _rotor_params[1].position_y * thrust[1]
                  - _rotor_params[2].position_y * thrust[2]
                  - _rotor_params[3].position_y * thrust[3]
                  - torque[4]
                  - Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get() * x(CA_ELEVON_RIGHT)
                  + Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get() * x(CA_ELEVON_LEFT);

    float Tau_y = _rotor_params[0].position_x * thrust[0] +
                  _rotor_params[1].position_x * thrust[1] +
                  _rotor_params[2].position_x * thrust[2] +
                  _rotor_params[3].position_x * thrust[3] +
                  Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get() * x(CA_ELEVON_RIGHT) +
                  Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get() * x(CA_ELEVON_LEFT);

    float Tau_z = torque[0] - torque[1] - torque[2] + torque[3];

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
NonlinearEffectivenessVTOLQuadplane::calcThrustTorqueAchievedDer(
    matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> *thrustTorqueAchievedDer,
    float *thrust, float *torque, float *thrustDer, float *torqueDer,
    matrix::Vector2f elevonForceCoefs, matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma) {

    matrix::Vector<float, VTOL_NUM_ACTUATORS> T_x_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> T_z_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> Tau_x_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> Tau_y_der;
    matrix::Vector<float, VTOL_NUM_ACTUATORS> Tau_z_der;

    for(uint8_t  i = 0; i < 4; ++i) {
        T_x_der(i) = 0.f;
        T_z_der(i) = -thrustDer[i];
        Tau_x_der(i) = -_rotor_params[i].position_y * thrustDer[i];
        Tau_y_der(i) = _rotor_params[i].position_x * thrustDer[i];
    }
    T_x_der(4) = thrustDer[4];
    T_x_der(5) = elevonForceCoefs(0);
    T_x_der(6) = elevonForceCoefs(0);

    T_z_der(4) = 0.f;
    T_z_der(5) = elevonForceCoefs(1);
    T_z_der(6) = elevonForceCoefs(1);

    Tau_x_der(4) = -torqueDer[4];
    Tau_x_der(5) = - Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get();
    Tau_x_der(6) = + Gamma * _param_ca_span.get() * _param_ca_C_ell_delta_a.get();

    Tau_y_der(4) = 0.f;
    Tau_y_der(5) = Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get();
    Tau_y_der(6) = Gamma * _param_ca_chord_len.get() * _param_ca_C_m_delta_e.get();

    Tau_z_der(0) = torqueDer[0];
    Tau_z_der(1) = -torqueDer[1];
    Tau_z_der(2) = -torqueDer[2];
    Tau_z_der(3) = torqueDer[3];
    Tau_z_der(4) = 0.f;
    Tau_z_der(5) = 0.f;
    Tau_z_der(6) = 0.f;

    thrustTorqueAchievedDer->setCol(0, T_x_der);
    thrustTorqueAchievedDer->setCol(1, T_z_der);
    thrustTorqueAchievedDer->setCol(2, Tau_x_der);
    thrustTorqueAchievedDer->setCol(3, Tau_y_der);
    thrustTorqueAchievedDer->setCol(4, Tau_z_der);
}
