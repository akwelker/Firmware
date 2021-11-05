#include "ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes.hpp"

// #define NUM_ACTUATORS 8
#define ROTOR_RIGHT 0
#define ROTOR_LEFT 1
#define ROTOR_REAR 2
#define EMPTY_LINE 3
#define SERVO_RIGHT 4
#define SERVO_LEFT 5
#define ELEVON_RIGHT 6
#define ELEVON_LEFT 7

#define MIX_ROTOR_R_X 0
#define MIX_ROTOR_R_Z 1
#define MIX_ROTOR_L_X 2
#define MIX_ROTOR_L_Z 3
#define MIX_ROTOR_REAR_Z 4
#define MIX_ELEVON_R 5
#define MIX_ELEVON_L 6

void ControlAllocationPseudoInverseTiltrotorVTOLCombinedModes::allocate() {
	float deg2rad = (float) M_PI / 180.f;
	float tilt_servo_max = _param_ca_tlt_srvo_max.get() * deg2rad;

	//Compute new gains if needed
	updatePseudoInverse();

	// Allocate
	matrix::Vector<float, NUM_ACTUATORS> mix_solution =
		_actuator_trim + _mix * (_control_sp - _control_trim);

	// Actuator setpoints are solved based on the mix solution
	// Throttle and servo angles are calculated from x and z components
	_actuator_sp(ROTOR_RIGHT) = sqrt(powf(mix_solution(MIX_ROTOR_R_X), 2.f) +
					 powf(mix_solution(MIX_ROTOR_R_Z), 2.f));
	_actuator_sp(ROTOR_LEFT) = sqrt(powf(mix_solution(MIX_ROTOR_L_X), 2.f) +
					powf(mix_solution(MIX_ROTOR_L_Z), 2.f));
	_actuator_sp(ROTOR_REAR) = mix_solution(MIX_ROTOR_REAR_Z);
	_actuator_sp(EMPTY_LINE) = 0.f;
	_actuator_sp(SERVO_RIGHT) = atan2f(mix_solution(MIX_ROTOR_R_Z), mix_solution(MIX_ROTOR_R_X)) /
					(tilt_servo_max);
	_actuator_sp(SERVO_LEFT) = 1.f - atan2f(mix_solution(MIX_ROTOR_L_Z), mix_solution(MIX_ROTOR_L_X)) /
					(tilt_servo_max);
	_actuator_sp(ELEVON_RIGHT) = mix_solution(MIX_ELEVON_R);
	_actuator_sp(ELEVON_LEFT) = mix_solution(MIX_ELEVON_L);


	// Clip
	clipActuatorSetpoint(_actuator_sp);

	// clipped_mix_solution performs backwards calculations to find mix_solution as adjusted for
	// clipped actuator values
	matrix::Vector<float, NUM_ACTUATORS> clipped_mix_solution;
	clipped_mix_solution(MIX_ROTOR_R_X) = _actuator_sp(ROTOR_RIGHT) /
		(float) sqrt(1 + pow(tanf(tilt_servo_max * _actuator_sp(SERVO_RIGHT)), 2));
	clipped_mix_solution(MIX_ROTOR_R_Z) = _actuator_sp(ROTOR_RIGHT) /
		(float) sqrt(1 + 1 / pow(tanf(tilt_servo_max * _actuator_sp(SERVO_RIGHT)), 2));
	clipped_mix_solution(MIX_ROTOR_L_X) = _actuator_sp(ROTOR_LEFT) /
		(float) sqrt(1 + pow(tanf(tilt_servo_max * (1.f - _actuator_sp(SERVO_LEFT))), 2));
	clipped_mix_solution(MIX_ROTOR_L_Z) = _actuator_sp(ROTOR_LEFT) /
		(float) sqrt(1 + 1 / pow(tanf(tilt_servo_max * (1.f - _actuator_sp(SERVO_LEFT))), 2));
	clipped_mix_solution(MIX_ROTOR_REAR_Z) = _actuator_sp(ROTOR_REAR);
	clipped_mix_solution(MIX_ELEVON_R) = _actuator_sp(ELEVON_RIGHT);
	clipped_mix_solution(MIX_ELEVON_L) = _actuator_sp(ELEVON_LEFT);

	// Sign information is lost from squaring/taking the square root so we infer the clipped_mix_solution
	// has the same sign as the original mix_solution
	if ((mix_solution(MIX_ROTOR_R_X) < 0 && clipped_mix_solution(MIX_ROTOR_R_X) > 0) ||
		(mix_solution(MIX_ROTOR_R_X) > 0 && clipped_mix_solution(MIX_ROTOR_R_X) < 0))
		clipped_mix_solution(MIX_ROTOR_R_X) *= -1;
	if ((mix_solution(MIX_ROTOR_L_X) < 0 && clipped_mix_solution(MIX_ROTOR_L_X) > 0) ||
		(mix_solution(MIX_ROTOR_L_X) > 0 && clipped_mix_solution(MIX_ROTOR_L_X) < 0))
		clipped_mix_solution(MIX_ROTOR_L_X) *= -1;

	// Compute achieved control
	_control_allocated = _effectiveness * clipped_mix_solution;
}
