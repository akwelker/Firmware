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
 * ControlAllocationData contains information to be passed into nonlinear minimization function
 */
struct ControlAllocationData {
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired;
    matrix::Vector3f vBody;
    float Gamma;
    float VaRear;
    float servoMax;
    float airspeed;
    float airDensity;
};

// Forward function declarations
static matrix::Vector2f calcElevonForce(struct ControlAllocationData* data);
static void rotorThrustTorque(float *thrustTorqueDers, float delta, float Va, float airDensity, uint8_t rotorNum);
static void calcThrustTorqueAchieved(matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    float *thrust, float *torque, matrix::Vector2f elevonForceCoefs,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma);
static void calcThrustTorqueAchieved(
    matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> x, matrix::Vector3f vBody, float airspeed, float airDensity);
static void calcThrustTorqueAchievedDer(
    matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> *thrustTorqueAchievedDer,
    float *thrust, float *torque, float *thrustDer, float *torqueDer,
    matrix::Vector2f elevonForceCoefs, matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma);
static float nonlinearCtrlOptFun(const matrix::Vector<float, VTOL_NUM_ACTUATORS>& vals_inp,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> *grad_out, void *opt_data);
static matrix::Vector<float, VTOL_NUM_ACTUATORS> computeNonlinearOpt(
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> x0,
    ControlAllocationData *controlAllocationData, size_t iterMax);
static void getAchievableThrust(matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueDesired,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> actuators, ControlAllocationData *controlAllocationData);

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
void rotorThrustTorque(float *thrustTorqueDers, float delta, float Va, float airDensity, uint8_t rotorNum) {
    float C_Q0;
    float C_Q1;
    float C_Q2;
    float C_T0;
    float C_T1;
    float C_T2;
    float D_prop;
    float KQ;
    float R_motor;
    float i0;

    // Set parameters for correct rotor
    if (rotorNum == CA_ROTOR_REAR) {
        C_Q0 = C_Q0_REAR;
        C_Q1 = C_Q1_REAR;
        C_Q2 = C_Q2_REAR;
        C_T0 = C_T0_REAR;
        C_T1 = C_T1_REAR;
        C_T2 = C_T2_REAR;
        D_prop = D_PROP_REAR;
        KQ = KQ_REAR;
        R_motor = R_MOTOR_REAR;
        i0 = I0_REAR;
    } else {
        C_Q0 = C_Q0_FRONT;
        C_Q1 = C_Q1_FRONT;
        C_Q2 = C_Q2_FRONT;
        C_T0 = C_T0_FRONT;
        C_T1 = C_T1_FRONT;
        C_T2 = C_T2_FRONT;
        D_prop = D_PROP_FRONT;
        KQ = KQ_FRONT;
        R_motor = R_MOTOR_FRONT;
        i0 = I0_FRONT;
    }

    // map delta_t throttle command(0 to 1) into motor input voltage
    float V_in = VTOL_V_MAX * delta;
    float V_in_der = VTOL_V_MAX;
    // Quadratic formula to solve for motor speed
    float a = C_Q0 * airDensity * pow(D_prop, 5.f)
        / (pow((2.f * (float) M_PI), 2.f));
    float b = (C_Q1 * airDensity * pow(D_prop, 4.f)
        / (2.f * (float) M_PI)) * Va + pow(KQ, 2.f) / R_motor;
    float c = C_Q2 * airDensity * pow(D_prop, 3.f)
        * pow(Va, 2.f) - (KQ / R_motor) * V_in + KQ * i0;
    float c_der = (KQ / R_motor) * V_in_der;
    // Consider only positive root
    float Omega_op = (-b + std::sqrt(pow(b, 2.f) - 4.f*a*c)) / (2.f*a);
    float Omega_op_der = c_der / std::sqrt(pow(b, 2.f) - 4.f*a*c);
    // compute advance ratio
    float J_op = 2.f * (float) M_PI * Va / (Omega_op * D_prop);
    float J_op_der = -2.f * (float) M_PI * Va * Omega_op_der / (pow(Omega_op, 2.f) * D_prop);
    // compute non-dimensionalized coefficients of thrust and torque
    float C_T = C_T2 * pow(J_op, 2.f) + C_T1 * J_op + C_T0;
    float C_Q = C_Q2 * pow(J_op, 2.f) + C_Q1 * J_op + C_Q0;
    float C_T_der = 2 * C_T2 * J_op * J_op_der + C_T1 * J_op_der;
    float C_Q_der = 2 * C_Q2 * J_op * J_op_der + C_Q1 * J_op_der;
    // add thrust and torque due to propeller
    float n = Omega_op / (2 * (float) M_PI);
    thrustTorqueDers[0] = airDensity * pow(n, 2.f) * pow(D_prop, 4.f) * C_T; // thrust value
    thrustTorqueDers[1] = airDensity * pow(n, 2.f) * pow(D_prop, 5.f) * C_Q; // torque value
    thrustTorqueDers[2] = airDensity * Omega_op * Omega_op_der * pow(D_prop, 4.f) * C_T / (2 * pow((float) M_PI, 2.f)) +
            airDensity * pow(Omega_op, 2.f) * pow(D_prop, 4.f) * C_T_der / pow(2 * (float) M_PI, 2.f); // thrust derivative
    thrustTorqueDers[3] = airDensity * Omega_op * Omega_op_der * pow(D_prop, 5.f) * C_Q / (2 * pow((float) M_PI, 2.f)) +
            airDensity * pow(Omega_op, 2.f) * pow(D_prop, 5.f) * C_Q_der / pow(2 * (float) M_PI, 2.f); // torque derivative
    // Negates torque moment for left rotor to account for its CW direction of rotation
    if (rotorNum == CA_ROTOR_LEFT) {
        thrustTorqueDers[1] *= -1;
        thrustTorqueDers[3] *= -1;
    }
}

/**
 * Calculates the thrust and torque achieved by a given set of rotor/servo/elevon setpoints
 * thrustTorqueAchieved: Will contain the thrust and torque achieved
 * thrust: thrust array (length 3) of each of the rotors
 * torque: torque array (length 3) of each of the rotors
 * x: Actuator setpoints
 * Gamma: Term used to determine elevon effectiveness
 */
void calcThrustTorqueAchieved(matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    float *thrust, float *torque, matrix::Vector2f elevonForceCoefs, matrix::Vector<float, VTOL_NUM_ACTUATORS> x, float Gamma) {
    float x_0 = std::cos(x(CA_SERVO_RIGHT));
    float z_0 = std::sin(x(CA_SERVO_RIGHT));
    float x_1 = std::cos(x(CA_SERVO_LEFT));
    float z_1 = std::sin(x(CA_SERVO_LEFT));

    float T_x =  (thrust[0] * x_0) + (thrust[1] * x_1) +
        elevonForceCoefs(0) * (x(CA_ELEVON_RIGHT) + x(CA_ELEVON_LEFT));
    float T_z = -(thrust[0] * z_0) - (thrust[1] * z_1) - thrust[2] +
        elevonForceCoefs(1) * (x(CA_ELEVON_RIGHT) + x(CA_ELEVON_LEFT));
    float Tau_x =   - (torque[0] * x_0) - (thrust[0] * VTOL_Q0_Y * z_0)
                    - (torque[1] * x_1) - (thrust[1] * VTOL_Q1_Y * z_1)
                    - (thrust[2] * VTOL_Q2_Y)
                    - Gamma * VTOL_B * VTOL_C_ELL_DELTA_A * x(CA_ELEVON_RIGHT)
                    + Gamma * VTOL_B * VTOL_C_ELL_DELTA_A * x(CA_ELEVON_LEFT);
    float Tau_y =   thrust[0] * (VTOL_Q0_Z * x_0 + VTOL_Q0_X * z_0) +
                    thrust[1] * (VTOL_Q1_Z * x_1 + VTOL_Q1_X * z_1) +
                    thrust[2] * VTOL_Q2_X +
                    Gamma * VTOL_C * VTOL_C_M_DELTA_E * x(CA_ELEVON_RIGHT) +
                    Gamma * VTOL_C * VTOL_C_M_DELTA_E * x(CA_ELEVON_LEFT);
    float Tau_z =   - (thrust[0] * VTOL_Q0_Y * x_0) + (torque[0] * z_0)
                    - (thrust[1] * VTOL_Q1_Y * x_1) + (torque[1] * z_1)
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
 * x: Actuator setpoints
 * vBody: velocity in the body frame
 * airspeed: vehicle airspeed
 */
void calcThrustTorqueAchieved(
    	matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueAchieved,
    	matrix::Vector<float, VTOL_NUM_ACTUATORS> x, matrix::Vector3f vBody, float airspeed, float airDensity)
{
	float x_0 = std::cos(x(CA_SERVO_RIGHT));
    	float z_0 = std::sin(x(CA_SERVO_RIGHT));
    	float x_1 = std::cos(x(CA_SERVO_LEFT));
    	float z_1 = std::sin(x(CA_SERVO_LEFT));

	float Gamma = .5f * airDensity * pow(airspeed, 2.f) * VTOL_S_WING;
	float VaRear = (matrix::Vector3f(0.0, 0.0, -1.0).transpose() * vBody)(0, 0);
	float VaRight = (matrix::Vector3f(x_0, 0.0, -z_0).transpose() * vBody)(0, 0);
	float VaLeft = (matrix::Vector3f(x_1, 0.0, -z_1).transpose() * vBody)(0, 0);

	float thrustTorqueDersRotor0[4];
	float thrustTorqueDersRotor1[4];
	float thrustTorqueDersRotor2[4];

	rotorThrustTorque(thrustTorqueDersRotor0, x(0), VaRight, airDensity, 0);
	rotorThrustTorque(thrustTorqueDersRotor1, x(1), VaLeft, airDensity, 1);
	rotorThrustTorque(thrustTorqueDersRotor2, x(2), VaRear, airDensity, 2);

	float thrust[3] = {thrustTorqueDersRotor0[0], thrustTorqueDersRotor1[0], thrustTorqueDersRotor2[0]};
	float torque[3] = {thrustTorqueDersRotor0[1], thrustTorqueDersRotor1[1], thrustTorqueDersRotor2[1]};

    ControlAllocationData controlAllocationData;
    controlAllocationData.Gamma = Gamma;
    controlAllocationData.vBody = vBody;
    matrix::Vector2f elevonForceCoefs = calcElevonForce(&controlAllocationData);

	calcThrustTorqueAchieved(thrustTorqueAchieved, thrust, torque, elevonForceCoefs, x, Gamma);
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
void calcThrustTorqueAchievedDer(
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

    Tau_x_der(0) = -(torqueDer[0] * x_0) - (thrustDer[0] * VTOL_Q0_Y * z_0);
    Tau_x_der(1) = -(torqueDer[1] * x_1) - (thrustDer[1] * VTOL_Q1_Y * z_1);
    Tau_x_der(2) = - (thrustDer[2] * VTOL_Q2_Y);
    Tau_x_der(3) = torque[0] * z_0 - thrust[0] * VTOL_Q0_Y * x_0;
    Tau_x_der(4) = torque[1] * z_1 - thrust[1] * VTOL_Q1_Y * x_1;
    Tau_x_der(5) = - Gamma * VTOL_B * VTOL_C_ELL_DELTA_A;
    Tau_x_der(6) = + Gamma * VTOL_B * VTOL_C_ELL_DELTA_A;

    Tau_y_der(0) = thrustDer[0] * (VTOL_Q0_Z * x_0 + VTOL_Q0_X * z_0);
    Tau_y_der(1) = thrustDer[1] * (VTOL_Q1_Z * x_1 + VTOL_Q1_X * z_1);
    Tau_y_der(2) = thrustDer[2] * VTOL_Q2_X;
    Tau_y_der(3) = thrust[0] * (-VTOL_Q0_Z * z_0 + VTOL_Q0_X * x_0);
    Tau_y_der(4) = thrust[1] * (-VTOL_Q1_Z * z_1 + VTOL_Q1_X * x_1);
    Tau_y_der(5) = Gamma * VTOL_C * VTOL_C_M_DELTA_E;
    Tau_y_der(6) = Gamma * VTOL_C * VTOL_C_M_DELTA_E;

    Tau_z_der(0) = -(thrustDer[0] * VTOL_Q0_Y * x_0) + (torqueDer[0] * z_0);
    Tau_z_der(1) = -(thrustDer[1] * VTOL_Q1_Y * x_1) + (torqueDer[1] * z_1);
    Tau_z_der(2) = torqueDer[2];
    Tau_z_der(3) = thrust[0] * VTOL_Q0_Y * z_0 + torque[0] * x_0;
    Tau_z_der(4) = thrust[1] * VTOL_Q1_Y * z_1 + torque[1] * x_1;
    Tau_z_der(5) = 0;
    Tau_z_der(6) = 0;

    thrustTorqueAchievedDer->setCol(0, T_x_der);
    thrustTorqueAchievedDer->setCol(1, T_z_der);
    thrustTorqueAchievedDer->setCol(2, Tau_x_der);
    thrustTorqueAchievedDer->setCol(3, Tau_y_der);
    thrustTorqueAchievedDer->setCol(4, Tau_z_der);
}

/**
 * Optimization function that is used to calculate the norm of the difference between desired thrust/torque
 * and achieved thrust/torque, as well as that function's gradient.
 * vals_inp: actuator setpoints, the input to the function
 * grad_out: The gradient of the function at the vals_inp point
 * opt_data: Extra arguments to the function (Gamma, VaRear, vBody, and thrustTorqueDesired)
 */
float nonlinearCtrlOptFun(const matrix::Vector<float, VTOL_NUM_ACTUATORS>& vals_inp,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> *grad_out, void *opt_data) {

    ControlAllocationData* controlAllocationData = reinterpret_cast<ControlAllocationData*>(opt_data);

    float x_0 = std::cos(vals_inp(CA_SERVO_RIGHT));
    float z_0 = std::sin(vals_inp(CA_SERVO_RIGHT));
    float x_1 = std::cos(vals_inp(CA_SERVO_LEFT));
    float z_1 = std::sin(vals_inp(CA_SERVO_LEFT));
    matrix::Matrix<float, VTOL_NUM_AXES, VTOL_NUM_AXES> K;
    K.setIdentity();
    if (controlAllocationData->airspeed < 50.f) {
        K(2,2) = 1.f;
        K(3,3) = 1.f;
        K(4,4) = 1.f;
    } else {
        K(2,2) = 2.0f;
        K(3,3) = 2.0f;
        K(4,4) = 2.0f;
    }

    float VaRight = (matrix::Vector3f(x_0, 0.0, -z_0).transpose() * controlAllocationData->vBody)(0, 0);
    float VaLeft = (matrix::Vector3f(x_1, 0.0, -z_1).transpose() * controlAllocationData->vBody)(0, 0);

    float thrustTorqueDersRotor0[4];
    float thrustTorqueDersRotor1[4];
    float thrustTorqueDersRotor2[4];

    rotorThrustTorque(thrustTorqueDersRotor0, vals_inp(0), VaRight, controlAllocationData->airDensity, 0);
    rotorThrustTorque(thrustTorqueDersRotor1, vals_inp(1), VaLeft, controlAllocationData->airDensity, 1);
    rotorThrustTorque(thrustTorqueDersRotor2, vals_inp(2), controlAllocationData->VaRear, controlAllocationData->airDensity, 2);

    float thrust[3] = {thrustTorqueDersRotor0[0], thrustTorqueDersRotor1[0], thrustTorqueDersRotor2[0]};
    float torque[3] = {thrustTorqueDersRotor0[1], thrustTorqueDersRotor1[1], thrustTorqueDersRotor2[1]};
    float thrustDer[3] = {thrustTorqueDersRotor0[2], thrustTorqueDersRotor1[2], thrustTorqueDersRotor2[2]};
    float torqueDer[3] = {thrustTorqueDersRotor0[3], thrustTorqueDersRotor1[3], thrustTorqueDersRotor2[3]};
    matrix::Vector2f elevonForceCoefs = calcElevonForce(controlAllocationData);

    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueAchieved;
    calcThrustTorqueAchieved(&thrustTorqueAchieved, thrust, torque, elevonForceCoefs, vals_inp, controlAllocationData->Gamma);
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDiff = controlAllocationData->thrustTorqueDesired - thrustTorqueAchieved;
    float normDiff = (0.5f * thrustTorqueDiff.transpose() * K * thrustTorqueDiff)(0, 0);
    matrix::Vector3f rotorOutput(vals_inp(CA_ROTOR_RIGHT), vals_inp(CA_ROTOR_LEFT), vals_inp(CA_ROTOR_REAR));
    float rotorScaling = 0.0f;//.1f;
    normDiff += rotorScaling * (rotorOutput.transpose() * rotorOutput)(0, 0);

    matrix::Matrix<float, VTOL_NUM_ACTUATORS, VTOL_NUM_AXES> thrustTorqueDer;
    calcThrustTorqueAchievedDer(&thrustTorqueDer, thrust, torque, thrustDer, torqueDer,
        elevonForceCoefs, vals_inp, controlAllocationData->Gamma);
    matrix::Vector<float, VTOL_NUM_ACTUATORS> normDiffDer = -1.f * thrustTorqueDer * K * thrustTorqueDiff;
    normDiffDer(CA_ROTOR_RIGHT) += rotorScaling * 2.f * vals_inp(CA_ROTOR_RIGHT);
    normDiffDer(CA_ROTOR_LEFT) += rotorScaling * 2.f * vals_inp(CA_ROTOR_LEFT);
    normDiffDer(CA_ROTOR_REAR) += rotorScaling * 2.f * vals_inp(CA_ROTOR_REAR);

    if (grad_out) {
        *grad_out = normDiffDer;
    }

    return normDiff;
}

/**
 * Calculates forces produced by elevons
 * @param data ControlAllocationData function data used by minimization function
 * @returns 2-dimensional elevon force vector representing the force produced
 * by each vector.
 */
matrix::Vector2f calcElevonForce(struct ControlAllocationData *data) {
    float elevonLiftCoef = data->Gamma * VTOL_C_L_DELTA_E;
    float elevonDragCoef = data->Gamma * VTOL_C_D_DELTA_E;
    float alpha;
    if (data->vBody(0) > -0.00001f && data->vBody(0) < 0.00001f) {
        alpha = atan2f(data->vBody(2), data->vBody(0));
    } else {
        alpha = 0.f;
    }
    matrix::Vector2f elevonForceCoefs(-cosf(alpha) * elevonDragCoef + sinf(alpha) * elevonLiftCoef,
                                      -sinf(alpha) * elevonDragCoef - cosf(alpha) * elevonLiftCoef);
    return elevonForceCoefs;
}

/**
 * If thrust is outside of the range achievable by the servos, projects the thrust onto an
 * achievable axis.
 * @param thrustTorqueDesired will be updated with new thrustTorqueDesired
 * @param actuators current actuator setpoints used to calculate lift and drag forces
 */
void getAchievableThrust(matrix::Vector<float, VTOL_NUM_AXES> *thrustTorqueDesired,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> actuators, ControlAllocationData *controlAllocationData) {
    float tilt_servo_max = 115.f * (float) M_PI / 180.f;
    float tilt_servo_min = 0.f * (float) M_PI / 180.f;

    // Get the thrust needed to be produced by rotors
    matrix::Vector2f thrustDesired((*thrustTorqueDesired)(0), (*thrustTorqueDesired)(1));
    matrix::Vector2f elevonForceCoefs = calcElevonForce(controlAllocationData);
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
matrix::Vector<float, VTOL_NUM_ACTUATORS> computeNonlinearOpt(
    matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired,
    matrix::Vector<float, VTOL_NUM_ACTUATORS> x0,
    ControlAllocationData *controlAllocationData, size_t iterMax) {

    bfgs_settings_t settings;
    float tiltServoMax = 115.f * (float) M_PI / 180.f;
    float lowerBoundsf[VTOL_NUM_ACTUATORS] = {0.00, 0.00, 0.00, 0.0, 0.0, -1.0, -1.0};
    float upperBoundsf[VTOL_NUM_ACTUATORS] = {1.0, 1.0, 1.0, tiltServoMax, tiltServoMax, 1.0, 1.0};
    matrix::Vector<float, VTOL_NUM_ACTUATORS> lowerBounds(lowerBoundsf);
    matrix::Vector<float, VTOL_NUM_ACTUATORS> upperBounds(upperBoundsf);
    settings.lower_bounds = lowerBounds;
    settings.upper_bounds = upperBounds;
    settings.iter_max = iterMax;

    ctrlalloc_bfgs(x0, nonlinearCtrlOptFun, controlAllocationData, settings);

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
    float elevon_angle_max = 45.f * (float) M_PI / 180.f;

	matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueDesired;
	matrix::Vector<float, VTOL_NUM_AXES> thrustTorqueAchieved;
	matrix::Vector<float, VTOL_NUM_ACTUATORS> solution;
	matrix::Vector3f vBody = _q.conjugate_inversed(_interial_velocity);

	thrustTorqueDesired(0) = _control_sp(3);
	thrustTorqueDesired(1) = _control_sp(5);
	thrustTorqueDesired(2) = _control_sp(0);
	thrustTorqueDesired(3) = _control_sp(1);
	thrustTorqueDesired(4) = _control_sp(2);

    initialGuessGuard();

    solution(0) = _actuator_sp(0);
    solution(1) = _actuator_sp(1);
    solution(2) = _actuator_sp(2);
    solution(3) = _actuator_sp(4) * VTOL_SERVO_MAX;
    solution(4) = (1.f - _actuator_sp(5)) * VTOL_SERVO_MAX;
    solution(5) = -_actuator_sp(6) * elevon_angle_max; // positive right elevon value makes elevon go up. Needs to be negated
    solution(6) = _actuator_sp(7) * elevon_angle_max;

    // Set up ControlAllocationData
    ControlAllocationData controlAllocationData;
    controlAllocationData.Gamma = .5f * _air_density * powf(_airspeed, 2.f) * VTOL_S_WING;
    controlAllocationData.VaRear = (matrix::Vector3f(0.0, 0.0, -1.0).transpose() * vBody)(0, 0);
    controlAllocationData.thrustTorqueDesired = thrustTorqueDesired;
    controlAllocationData.vBody = vBody;
    controlAllocationData.airDensity = _air_density;
    controlAllocationData.airspeed = _airspeed;

    getAchievableThrust(&thrustTorqueDesired, solution, &controlAllocationData);

    // Limits the thrust magnitude to roughly be within achievable range
    // This only changes thrust vector magnitude, not the angle of the thrust vector
    if (thrustTorqueDesired(1) < VTOL_MAX_THRUST_NEG_Z) {
        float scaleFactor = VTOL_MAX_THRUST_NEG_Z / thrustTorqueDesired(1);
        thrustTorqueDesired(0) = thrustTorqueDesired(0) * scaleFactor;
        thrustTorqueDesired(1) = VTOL_MAX_THRUST_NEG_Z;
    }
    if (thrustTorqueDesired(0) < VTOL_MAX_THRUST_NEG_X) {
        float scaleFactor = VTOL_MAX_THRUST_NEG_X / thrustTorqueDesired(0);
        thrustTorqueDesired(0) = VTOL_MAX_THRUST_NEG_X;
        thrustTorqueDesired(1) = thrustTorqueDesired(1) * scaleFactor;
    } else if (thrustTorqueDesired(0) > VTOL_MAX_THRUST_POS_X) {
        float scaleFactor = VTOL_MAX_THRUST_POS_X / thrustTorqueDesired(0);
        thrustTorqueDesired(0) = VTOL_MAX_THRUST_POS_X;
        thrustTorqueDesired(1) = thrustTorqueDesired(1) * scaleFactor;
    }

	solution = computeNonlinearOpt(thrustTorqueDesired, solution, &controlAllocationData, ITER_MAX);
	calcThrustTorqueAchieved(&thrustTorqueAchieved, solution, vBody, _airspeed, _air_density);

	_actuator_sp(0) = solution(0);
	_actuator_sp(1) = solution(1);
	_actuator_sp(2) = solution(2);
	_actuator_sp(3) = 0.f;
	_actuator_sp(4) = solution(3) / VTOL_SERVO_MAX;
	_actuator_sp(5) = 1.f - (solution(4) / VTOL_SERVO_MAX);
	_actuator_sp(6) = -solution(5) / elevon_angle_max; // positive right elevon value makes elevon go up. Needs to be negated
	_actuator_sp(7) = solution(6) / elevon_angle_max;

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
    _rotor_params[0].position_x = _param_ca_mc_r0_px.get();
    _rotor_params[0].position_y = _param_ca_mc_r0_py.get();
    _rotor_params[0].position_z = _param_ca_mc_r0_pz.get();
    _rotor_params[0].axis_x = _param_ca_mc_r0_ax.get();
    _rotor_params[0].axis_y = _param_ca_mc_r0_ay.get();
    _rotor_params[0].axis_z = _param_ca_mc_r0_az.get();
    _rotor_params[0].CQ0 = _param_ca_rot0_CQ0.get();
    _rotor_params[0].CQ1 = _param_ca_rot0_CQ1.get();
    _rotor_params[0].CQ2 = _param_ca_rot0_CQ2.get();
    _rotor_params[0].CT0 = _param_ca_rot0_CT0.get();
    _rotor_params[0].CT1 = _param_ca_rot0_CT1.get();
    _rotor_params[0].CT2 = _param_ca_rot0_CT2.get();
    _rotor_params[0].prop_diam = _param_ca_rot0_prop_diam.get();
    _rotor_params[0].KV = _param_ca_rot0_KV.get();
    _rotor_params[0].KQ = (1.f / _rotor_params[0].KV) * 60.f / (2.f * (float) M_PI);
    _rotor_params[0].resistance = _param_ca_rot0_res.get();
    _rotor_params[0].i0 = _param_ca_rot0_i0.get();

    _rotor_params[1].position_x = _param_ca_mc_r1_px.get();
    _rotor_params[1].position_y = _param_ca_mc_r1_py.get();
    _rotor_params[1].position_z = _param_ca_mc_r1_pz.get();
    _rotor_params[1].axis_x = _param_ca_mc_r1_ax.get();
    _rotor_params[1].axis_y = _param_ca_mc_r1_ay.get();
    _rotor_params[1].axis_z = _param_ca_mc_r1_az.get();
    _rotor_params[1].CQ0 = _param_ca_rot1_CQ0.get();
    _rotor_params[1].CQ1 = _param_ca_rot1_CQ1.get();
    _rotor_params[1].CQ2 = _param_ca_rot1_CQ2.get();
    _rotor_params[1].CT0 = _param_ca_rot1_CT0.get();
    _rotor_params[1].CT1 = _param_ca_rot1_CT1.get();
    _rotor_params[1].CT2 = _param_ca_rot1_CT2.get();
    _rotor_params[1].prop_diam = _param_ca_rot1_prop_diam.get();
    _rotor_params[1].KV = _param_ca_rot1_KV.get();
    _rotor_params[1].KQ = (1.f / _rotor_params[1].KV) * 60.f / (2.f * (float) M_PI);
    _rotor_params[1].resistance = _param_ca_rot1_res.get();
    _rotor_params[1].i0 = _param_ca_rot1_i0.get();

    _rotor_params[2].position_x = _param_ca_mc_r2_px.get();
    _rotor_params[2].position_y = _param_ca_mc_r2_py.get();
    _rotor_params[2].position_z = _param_ca_mc_r2_pz.get();
    _rotor_params[2].axis_x = _param_ca_mc_r2_ax.get();
    _rotor_params[2].axis_y = _param_ca_mc_r2_ay.get();
    _rotor_params[2].axis_z = _param_ca_mc_r2_az.get();
    _rotor_params[2].CQ0 = _param_ca_rot2_CQ0.get();
    _rotor_params[2].CQ1 = _param_ca_rot2_CQ1.get();
    _rotor_params[2].CQ2 = _param_ca_rot2_CQ2.get();
    _rotor_params[2].CT0 = _param_ca_rot2_CT0.get();
    _rotor_params[2].CT1 = _param_ca_rot2_CT1.get();
    _rotor_params[2].CT2 = _param_ca_rot2_CT2.get();
    _rotor_params[2].prop_diam = _param_ca_rot2_prop_diam.get();
    _rotor_params[2].KV = _param_ca_rot2_KV.get();
    _rotor_params[2].KQ = (1.f / _rotor_params[2].KV) * 60.f / (2.f * (float) M_PI);
    _rotor_params[2].resistance = _param_ca_rot2_res.get();
    _rotor_params[2].i0 = _param_ca_rot2_i0.get();

}
