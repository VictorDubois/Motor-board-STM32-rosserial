/*
 * DCMotor.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotor.h"

DCMotor::DCMotor(DCMotorHardware* a_hardware) : hardware(a_hardware) {
	resetMotors();
	for (int i = 0; i< NB_MOTORS; i++) {
		last_position[i] = 0;
	}
}

void DCMotor::resetMotors() {
	for (int i = 0; i< NB_MOTORS; i++) {
		dir[i] = 0;
		speed_ID[i] = 0;
		speed_error_ID[i] = 0;
		speed_integ_error[i] = 0;
		voltage[i] = 0;
		speed_command[i] = 0;
		speed_order[i] = 0;
		for (int j = 0; j < SMPL; j++) {
			speed[i][j] = 0;
			speed_error[i][j] = 0;
		}
	}

	hardware->setPWM(0, 0);
}

DCMotor::DCMotor() {
	for (int i = 0; i< NB_MOTORS; i++) {
		last_position[i] = 0;
		dir[i] = 0;
		speed_ID[i] = 0;
		speed_error_ID[i] = 0;
		speed_integ_error[i] = 0;
		voltage[i] = 0;
		speed_command[i] = 0;
		speed_order[i] = 0;
		for (int j = 0; j < SMPL; j++) {
			speed[i][j] = 0;
			speed_error[i][j] = 0;
		}
	}
}

DCMotor::~DCMotor() {}

void DCMotor::update() {
	get_speed();

	control_ramp_speed();

	//hardware->setPWM(speed_order[M_L], speed_order[M_R]);// no asserv
	hardware->setPWM(voltage[M_L], voltage[M_R]);

	for(int i = 0; i < NB_MOTORS; i++){
		speed_ID[i]++;
		if(speed_ID[i] >= SMPL) {
			speed_ID[i]=0;
		}
		speed_error_ID[i]++;
		if(speed_error_ID[i] >= SMPL) {
			speed_error_ID[i]=0;
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

        // average
        if(speed_ID[i] == 0) {
        	speed[i][speed_ID[i]] = speed[i][SMPL-1];
        }
        else {
        	speed[i][speed_ID[i]] = speed[i][speed_ID[i]-1];
        }

        speed[i][speed_ID[i]] *= SPEED_SMPL;
        //speed[i][speed_ID[i]] -= speed[i][(uint8_t)(speed_ID[i]+SMPL-SPEED_SMPL +1)%SMPL];
        speed[i][speed_ID[i]] -= speed[i][(uint8_t)(speed_ID[i]+SMPL-SPEED_SMPL+1)%SMPL];
        // format
        speed[i][speed_ID[i]] += // ticks per second
        		current_speed * SAMPLING_PER_SEC;

        speed[i][speed_ID[i]] /= SPEED_SMPL; //mean
    }
}

int32_t DCMotor::get_speed(uint8_t motor_id) {
	return speed[motor_id][speed_ID[motor_id]];
}

int32_t DCMotor::get_encoder_ticks(uint8_t encoder_id) {
	return hardware->getTicks(encoder_id);
}

void DCMotor::set_speed_order(float lin, float rot) {
	constexpr int32_t meters_to_tick = 4096/(M_PI * 0.068);
	constexpr int32_t rad_to_tick = meters_to_tick*(0.25 /2);

	int32_t linear_speed = meters_to_tick * lin;// = resolution/perimeter = 4096/(pi*68mm) to convert from m/s => 19172
	int32_t rotational_speed_order = rad_to_tick * rot;// = radius when turning on the spot (=half entraxe) / speed in m/s = (250mm/2) * 19172 to convert from rad/s => 2396

	int32_t left_speed_order = linear_speed+rotational_speed_order;
	left_speed_order = MIN(left_speed_order, SPEED_MAX);
	left_speed_order = MAX(left_speed_order, -SPEED_MAX);
	speed_order[M_L] = left_speed_order;

	int32_t right_speed_order = linear_speed-rotational_speed_order;
	right_speed_order = MIN(right_speed_order, SPEED_MAX);
	right_speed_order = MAX(right_speed_order, -SPEED_MAX);
	speed_order[M_R] = right_speed_order;
}

void DCMotor::control_ramp_speed(void) {
    //if( stopped ) return;

    for(int i = 0; i < NB_MOTORS; i++){
        if( (int32_t)(speed_order[i]) - speed[i][speed_ID[i]] >= ACCEL_MAX) {
        	speed_command[i] = speed[i][speed_ID[i]]+ACCEL_MAX;
        }
        else if ( (int32_t)(speed_order[i]) - speed[i][speed_ID[i]] <= -ACCEL_MAX ) {
        	speed_command[i] = speed[i][speed_ID[i]]-ACCEL_MAX;
        }
        else {
        	speed_command[i] = speed_order[i];
        }

        speed_error[i][speed_error_ID[i]] = speed_command[i] - speed[i][speed_ID[i]];
        for(int j = 0; j < SMPL; j++){
            speed_integ_error[i] += speed_error[i][j];
        }

        voltage[i] =
             (S_KP*speed_error[i][speed_error_ID[i]] +
             S_KI*speed_integ_error[i]);

        voltage[i] = MIN(voltage[i], DUTYMAX);
        voltage[i] = MAX(voltage[i], -DUTYMAX);
    }
}
