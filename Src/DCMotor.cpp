/*
 * DCMotor.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotor.h"

DCMotor::DCMotor(DCMotorHardware* a_hardware, MCP3002* a_current_reader) : hardware(a_hardware), current_reader(a_current_reader) {
	resetMotors();
	max_speed = SPEED_MAX;
	max_speed_delta = ACCEL_MAX;
	set_max_current(0.5f);
	set_max_current(10.f, 10.f);
	pid_p = 0.0222;
	pid_i = 0.00625;
	pid_d = 0.0197;

	override_pwm = false;
	stopped_timeout = 0;

	for (int i = 0; i< NB_MOTORS; i++) {
		last_position[i] = 0;
		current[i] = 0;
		accumulated_current[i] = 0;
		speed[i] = 0;
		dir[i] = 0;
		speed_integ_error[i] = 0;
		voltage[i] = 0;
		speed_command[i] = 0;
		speed_order[i] = 0;
		stopped_timeouts[i] = 0;
		speed_error[i] = 0;
		last_speed_error[i] = 0;
		override_pwms[i] = 0;
	}

}

void DCMotor::override_PWM(int pwm_left, int pwm_right)
{
	override_pwm = true;
	override_pwms[M_L] = pwm_left;
	override_pwms[M_R] = pwm_right;

	// Reset overCurrentProtection
	// It should reenable itself if needed
	stopped_timeout = hardware->getMilliSecondsElapsed();
	for (int i = 0; i< NB_MOTORS; i++) {
		stopped_timeouts[i] = hardware->getMilliSecondsElapsed();
	}
}

void DCMotor::stop_pwm_override()
{
	if (override_pwm) {
		// Reset asserv that probably diverged during override
		resetMotors();

		// Reset overCurrentProtection
		// It should reenable itself if needed
		stopped_timeout = hardware->getMilliSecondsElapsed();
		for (int i = 0; i< NB_MOTORS; i++) {
			stopped_timeouts[i] = hardware->getMilliSecondsElapsed();
		}
	}

	override_pwm = false;
}

void DCMotor::resetMotor(int motor_id) {
	dir[motor_id] = 0;
	speed_integ_error[motor_id] = 0;
	voltage[motor_id] = 0;
	speed_command[motor_id] = 0;
	speed_order[motor_id] = 0;
	speed_error[motor_id] = 0;
	override_pwms[motor_id] = 0;
}

void DCMotor::resetMotors() {
	for (int i = 0; i< NB_MOTORS; i++) {
		resetMotor(i);
	}

	hardware->setPWM(0, 0);
}

DCMotor::DCMotor() {}

DCMotor::~DCMotor() {}

void DCMotor::overCurrentProtection() {
	for(int i = 0; i < NB_MOTORS; i++){
		current[i] = current_reader->readCurrent(i);
		if (current[i] == CURRENT_READER_OFFLINE) {
			stopped_timeout = hardware->getMilliSecondsElapsed() + 3000;
		}

		constexpr uint8_t current_averaging_period = 20;// measure over 20 iterations
		constexpr float current_averaging_factor = 1.f - (1.f/current_averaging_period);

		accumulated_current[i] = current_averaging_factor * accumulated_current[i] + current[i];

		if (accumulated_current[i] > max_current * current_averaging_period) {
			stopped_timeout = hardware->getMilliSecondsElapsed() + 3000;
		}

		if (accumulated_current[i] > max_currents[i] * current_averaging_period) {
			stopped_timeouts[i] = hardware->getMilliSecondsElapsed() + 1000;
		}
	}
}

void DCMotor::update() {
	get_speed();

	overCurrentProtection();

	if (stopped_timeout > hardware->getMilliSecondsElapsed()) {
		resetMotors();
	}
	else {
		for(int i = 0; i < NB_MOTORS; i++){
			if (stopped_timeouts[i] > hardware->getMilliSecondsElapsed()){
				resetMotor(i);
				override_pwms[i] = 0;
			}
		}
		control_ramp_speed();

		//hardware->setPWM(speed_order[M_L], speed_order[M_R]);// no asserv

		if (override_pwm)
		{
			hardware->setPWM(override_pwms[M_L], override_pwms[M_R]);
		}
		else {
			hardware->setPWM(voltage[M_L], voltage[M_R]);
		}
	}
}

void DCMotor::get_speed(){
    int32_t current_speed = 0;

    for(int i = 0; i < NB_MOTORS; i++){
    	int32_t new_position = hardware->getTicks(i);
        current_speed =  new_position - last_position[i];
        last_position[i] = new_position;

        //rebase
        if( current_speed > HALF_ENC_BUF_SIZE) {
        	current_speed -= ENC_BUF_SIZE;
        }
        if( current_speed < -HALF_ENC_BUF_SIZE ) {
        	current_speed += ENC_BUF_SIZE;
        }

        speed[i] = current_speed * SAMPLING_PER_SEC;// ticks per second
    }
}

int32_t DCMotor::get_speed(uint8_t motor_id) {
	return speed[motor_id];
}

int32_t DCMotor::get_encoder_ticks(uint8_t encoder_id) {
	return hardware->getTicks(encoder_id);
}

int32_t DCMotor::get_accumulated_current(uint8_t motor_id) {
	return accumulated_current[motor_id];
}

int32_t DCMotor::get_current(uint8_t motor_id) {
	return current[motor_id];
}

void DCMotor::set_speed_order(float lin, float rot) {
	constexpr int32_t meters_to_tick = 4096/(M_PI * 0.068);
	constexpr int32_t rad_to_tick = meters_to_tick*(0.25 /2);

	int32_t linear_speed = meters_to_tick * lin;// = resolution/perimeter = 4096/(pi*68mm) to convert from m/s => 19172
	int32_t rotational_speed_order = rad_to_tick * rot;// = radius when turning on the spot (=half entraxe) / speed in m/s = (250mm/2) * 19172 to convert from rad/s => 2396

	int32_t left_speed_order = linear_speed+rotational_speed_order;
	left_speed_order = MIN(left_speed_order, max_speed);
	left_speed_order = MAX(left_speed_order, -max_speed);
	speed_order[M_L] = left_speed_order;

	int32_t right_speed_order = linear_speed-rotational_speed_order;
	right_speed_order = MIN(right_speed_order, max_speed);
	right_speed_order = MAX(right_speed_order, -max_speed);
	speed_order[M_R] = right_speed_order;
}

void DCMotor::control_ramp_speed(void) {
    //if( stopped ) return;

    for(int i = 0; i < NB_MOTORS; i++){
        if( (int32_t)(speed_order[i]) - speed[i] >= max_speed_delta) {
        	speed_command[i] = speed[i]+max_speed_delta;
        }
        else if ( (int32_t)(speed_order[i]) - speed[i] <= -max_speed_delta ) {
        	speed_command[i] = speed[i]-max_speed_delta;
        }
        else {
        	speed_command[i] = speed_order[i];
        }

        speed_error[i] = speed_command[i] - speed[i];
        speed_integ_error[i] += speed_error[i]; // dt is included in pid_i because it is constant. If we change dt, pid_i must be scaled

        voltage[i] =
             (pid_p*speed_error[i] +
             pid_i*speed_integ_error[i] + pid_d * (speed_error[i] - last_speed_error[i]));

        last_speed_error[i] = speed_error[i];

        voltage[i] = MIN(voltage[i], DUTYMAX);
        voltage[i] = MAX(voltage[i], -DUTYMAX);
    }
}

int32_t DCMotor::get_voltage(int8_t a_motor_id)
{
	return voltage[a_motor_id];
}

int32_t DCMotor::get_error(int8_t a_motor_id)
{
	return speed_error[a_motor_id];
}

void DCMotor::set_max_speed(int32_t a_max_speed)
{
	max_speed = a_max_speed;
}

void DCMotor::set_max_acceleration(int32_t a_max_acceleration)
{
	max_speed_delta = static_cast<int32_t>(static_cast<float>(a_max_acceleration)/static_cast<float>(SAMPLING_PER_SEC));
}

void DCMotor::set_pid_p(float a_pid_p)
{
	pid_p = a_pid_p;
}

void DCMotor::set_pid_i(float a_pid_i)
{
	pid_i = a_pid_i;
}

void DCMotor::set_pid_d(float a_pid_d)
{
	pid_d = a_pid_d;
}

void DCMotor::set_max_current(float a_max_current)
{
	max_current = a_max_current * ONE_AMP;
}

void DCMotor::set_max_current(float a_max_current_left, float a_max_current_right)
{
	max_currents[M_L] = a_max_current_left * ONE_AMP;
	max_currents[M_R] = a_max_current_right * ONE_AMP;
}
