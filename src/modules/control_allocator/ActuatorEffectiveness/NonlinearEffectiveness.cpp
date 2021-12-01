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

#include "NonlinearEffectiveness.hpp"
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

// VTOL Vehicle parameters
#define VTOL_NCELLS 3
#define VTOL_V_MAX (3.7f * VTOL_NCELLS)
#define VTOL_S_WING 0.2589f
#define VTOL_B 1.4224f
#define VTOL_C 0.3305f
#define VTOL_C_ELL_DELTA_A 0.018f
#define VTOL_C_M_DELTA_E (-0.05f)
#define VTOL_C_L_DELTA_E 0.2f
#define VTOL_C_D_DELTA_E 0.005f

// #define VTOL_K (matrix::Matrix<float, NUM_AXIS, NUM_AXIS>
// Identity(VTOL_NUM_AXES, VTOL_NUM_AXES))
#define VTOL_SERVO_MAX (115.0f * (float) M_PI / 180)
#define VTOL_SERVO_LIMITED_MAX (115.0f * (float) M_PI / 180)
#define VTOL_AIRSPEED_START_LIMIT 10.0f
#define VTOL_AIRSPEED_END_LIMIT 20.0f
#define VTOL_MAX_THRUST_NEG_X (-4.34f)
#define VTOL_MAX_THRUST_POS_X 18.f //(10.28f)
#define VTOL_MAX_THRUST_NEG_Z (-13.98f)

// Rotor positions
#define VTOL_Q0_X 0.12f
#define VTOL_Q0_Y 0.2f
#define VTOL_Q0_Z 0.0f
#define VTOL_Q1_X 0.12f
#define VTOL_Q1_Y (-0.2f)
#define VTOL_Q1_Z 0.0f
#define VTOL_Q2_X (-0.24f)
#define VTOL_Q2_Y 0.0f
#define VTOL_Q2_Z 0.0f

// Rear rotor paramaters
#define C_Q0_REAR 0.0216f
#define C_Q1_REAR 0.0292f
#define C_Q2_REAR (-0.0368f)
#define C_T0_REAR 0.2097f
#define C_T1_REAR 0.0505f
#define C_T2_REAR (-0.1921f)
#define D_PROP_REAR (5.5f*(0.0254f))
#define KV_REAR 1550.0f
#define KQ_REAR ((1.f / KV_REAR) * 60.f / (2.f * (float) M_PI))
#define R_MOTOR_REAR 0.4f
#define I0_REAR 0.6f

// Front rotor parameters
#define C_Q0_FRONT 0.0088f
#define C_Q1_FRONT 0.0129f
#define C_Q2_FRONT (-0.0216f)
#define C_T0_FRONT 0.1167f
#define C_T1_FRONT 0.0144f
#define C_T2_FRONT (-0.1480f)
#define D_PROP_FRONT (7.0f*0.0254f)
#define KV_FRONT 1450.0f
#define KQ_FRONT ((1.f / KV_FRONT) * 60.f / (2.f * (float) M_PI))
#define R_MOTOR_FRONT 0.3f
#define I0_FRONT 0.83f

#define ITER_MAX 5


/**
 * Returns the thrust and torque of a given rotor.
 * thrustTorqueDers is an array that contains at array indices:
 * - 1: thrust
 * - 2: torque
 * - 3: thrust derivative
 * - 4: torque derivative
 * Va: airspeed coming into rotor
 * rotorNum: indicates which rotor's thrust and torque should be found
 */
void
NonlinearEffectiveness::rotorThrustTorque(
    float *thrustTorqueDers, float delta, float Va, float airDensity, NonlinearEffectiveness_RotorParams rotor) {

    // map delta_t throttle command(0 to 1) into motor input voltage
    float V_in = rotor.V_max * delta;
    float V_in_der = rotor.V_max;
    // Quadratic formula to solve for motor speed
    float a = rotor.CQ0 * airDensity * pow(rotor.prop_diam, 5.f)
        / (pow((2.f * (float) M_PI), 2.f));
    float b = (rotor.CQ1 * airDensity * pow(rotor.prop_diam, 4.f)
        / (2.f * (float) M_PI)) * Va + pow(rotor.KQ, 2.f) / rotor.resistance;
    float c = rotor.CQ2 * airDensity * pow(rotor.prop_diam, 3.f)
        * pow(Va, 2.f) - (rotor.KQ / rotor.resistance) * V_in + rotor.KQ * rotor.i0;
    float c_der = (rotor.KQ / rotor.resistance) * V_in_der;
    // Consider only positive root
    float Omega_op = (-b + std::sqrt(pow(b, 2.f) - 4.f*a*c)) / (2.f*a);
    float Omega_op_der = c_der / std::sqrt(pow(b, 2.f) - 4.f*a*c);
    // compute advance ratio
    float J_op = 2.f * (float) M_PI * Va / (Omega_op * rotor.prop_diam);
    float J_op_der = -2.f * (float) M_PI * Va * Omega_op_der / (pow(Omega_op, 2.f) * rotor.prop_diam);
    // compute non-dimensionalized coefficients of thrust and torque
    float C_T = rotor.CT2 * pow(J_op, 2.f) + rotor.CT1 * J_op + rotor.CT0;
    float C_Q = rotor.CQ2 * pow(J_op, 2.f) + rotor.CQ1 * J_op + rotor.CQ0;
    float C_T_der = 2 * rotor.CT2 * J_op * J_op_der + rotor.CT1 * J_op_der;
    float C_Q_der = 2 * rotor.CQ2 * J_op * J_op_der + rotor.CQ1 * J_op_der;
    // add thrust and torque due to propeller
    float n = Omega_op / (2 * (float) M_PI);
    thrustTorqueDers[0] = airDensity * pow(n, 2.f) * pow(rotor.prop_diam, 4.f) * C_T; // thrust value
    thrustTorqueDers[1] = airDensity * pow(n, 2.f) * pow(rotor.prop_diam, 5.f) * C_Q; // torque value
    thrustTorqueDers[2] = airDensity * Omega_op * Omega_op_der * pow(rotor.prop_diam, 4.f) * C_T / (2 * pow((float) M_PI, 2.f)) +
            airDensity * pow(Omega_op, 2.f) * pow(rotor.prop_diam, 4.f) * C_T_der / pow(2 * (float) M_PI, 2.f); // thrust derivative
    thrustTorqueDers[3] = airDensity * Omega_op * Omega_op_der * pow(rotor.prop_diam, 5.f) * C_Q / (2 * pow((float) M_PI, 2.f)) +
            airDensity * pow(Omega_op, 2.f) * pow(rotor.prop_diam, 5.f) * C_Q_der / pow(2 * (float) M_PI, 2.f); // torque derivative
    // Negates torque moment for left rotor to account for its CW direction of rotation
    if (rotor.CW) {
        thrustTorqueDers[1] *= -1;
        thrustTorqueDers[3] *= -1;
    }
}

