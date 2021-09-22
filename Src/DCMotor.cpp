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
	max_acceleration = ACCEL_MAX;
	set_max_current(0.5f);
	set_max_current(10.f, 10.f);
	pid_p = S_KP;
	pid_i = S_KI;
	for (int i = 0; i< NB_MOTORS; i++) {
		last_position[i] = 0;
		current[i] = 0;
		accumulated_current[i] = 0;
		speed[i] = 0;
	}
}

void DCMotor::resetMotor(int motor_id) {
	dir[motor_id] = 0;
	speed_integ_error[motor_id] = 0;
	voltage[motor_id] = 0;
	speed_command[motor_id] = 0;
	speed_order[motor_id] = 0;
	speed_error[motor_id] = 0;
}

void DCMotor::resetMotors() {
	for (int i = 0; i< NB_MOTORS; i++) {
		resetMotor(i);
	}

	hardware->setPWM(0, 0);
}

DCMotor::DCMotor() {
	max_speed = SPEED_MAX;
	max_acceleration = ACCEL_MAX;
	pid_p = S_KP;
	pid_i = S_KI;
	set_max_current(0.5f);
	set_max_current(10.f, 10.f);
	stopped_timeout = 0;
	for (int i = 0; i< NB_MOTORS; i++) {
		last_position[i] = 0;
		dir[i] = 0;
		speed_integ_error[i] = 0;
		voltage[i] = 0;
		speed_command[i] = 0;
		speed_order[i] = 0;
		accumulated_current[i] = 0;
		current[i] = 0;
		stopped_timeouts[i] = 0;
		speed[i] = 0;
		speed_error[i] = 0;
	}
}

DCMotor::~DCMotor() {}

void DCMotor::update() {
	get_speed();

	if (stopped_timeout > 0) {
		stopped_timeout--;
		resetMotors();
	}
	else {
		for(int i = 0; i < NB_MOTORS; i++){
			if (stopped_timeouts[i] > 0){
				stopped_timeouts[i]--;
				resetMotor(i);
			}
		}
		control_ramp_speed();

		//hardware->setPWM(speed_order[M_L], speed_order[M_R]);// no asserv
		hardware->setPWM(voltage[M_L], voltage[M_R]);
	}

	for(int i = 0; i < NB_MOTORS; i++){
		current[i] = current_reader->readCurrent(i);
		if (current[i] == CURRENT_READER_OFFLINE) {
			stopped_timeout = 300;
		}

		constexpr uint8_t current_averaging_period = 20;// measure over 20 iterations
		constexpr float current_averaging_factor = 1.f - (1.f/current_averaging_period);

		accumulated_current[i] = current_averaging_factor * accumulated_current[i] + current[i];

		if (accumulated_current[i] > max_current * current_averaging_period) {
			stopped_timeout = 300;
		}

		if (accumulated_current[i] > max_currents[i] * current_averaging_period) {
			stopped_timeouts[i] = 100;
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
        if( (int32_t)(speed_order[i]) - speed[i] >= max_acceleration) {
        	speed_command[i] = speed[i]+max_acceleration;
        }
        else if ( (int32_t)(speed_order[i]) - speed[i] <= -max_acceleration ) {
        	speed_command[i] = speed[i]-max_acceleration;
        }
        else {
        	speed_command[i] = speed_order[i];
        }

        speed_error[i] = speed_command[i] - speed[i];
        speed_integ_error[i] += speed_error[i];

        voltage[i] =
             (pid_p*speed_error[i] +
             pid_i*speed_integ_error[i]);

        voltage[i] = MIN(voltage[i], DUTYMAX);
        voltage[i] = MAX(voltage[i], -DUTYMAX);
    }
}

int32_t DCMotor::get_voltage(int8_t a_motor_id)
{
	return voltage[a_motor_id];
}

void DCMotor::set_max_speed(int32_t a_max_speed)
{
	max_speed = a_max_speed;
}

void DCMotor::set_max_acceleration(int32_t a_max_acceleration)
{
	max_acceleration = a_max_acceleration;
}

void DCMotor::set_pid_p(float a_pid_p)
{
	pid_p = a_pid_p;
}

void DCMotor::set_pid_i(float a_pid_i)
{
	pid_i = a_pid_i;
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
