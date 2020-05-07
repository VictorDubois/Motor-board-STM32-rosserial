/*
 * DCMotor.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotor.h"

DCMotor::DCMotor(DCMotorHardware* a_hardware) : hardware(a_hardware) {
}

DCMotor::~DCMotor() {}

void DCMotor::update() {

	rebase_position();

	hardware->setPWM(M_L, 10);
	hardware->setPWM(M_R, 10);
}

void DCMotor::get_speed(){
    int32_t current_speed = 0;

    for(int i = 0; i < NB_MOTORS; i++){
        current_speed =  position[i] - last_position[i];
        last_position[i] = position[i];

        //rebase
        if( current_speed > HALF_ENC_RESO ) {
        	current_speed -= ENC_RESO;
        }
        if( current_speed < -HALF_ENC_RESO ) {
        	current_speed += ENC_RESO;
        }

        // average
        if(speed_ID[i] == 0) {
        	speed[i][speed_ID[i]] = speed[i][SMPL-1];
        }
        else {
        	speed[i][speed_ID[i]] = speed[i][speed_ID[i]-1];
        }

        speed[i][speed_ID[i]] *= SPEED_SMPL;
        speed[i][speed_ID[i]] -= speed[i][(uint8_t)(speed_ID[i]+SMPL-SPEED_SMPL)%SMPL];
        //speed[i][speed_ID[i]] -= speed[i][(uint8_t)(speed_ID[MG]+SMPL-SPEED_SMPL)%SMPL];
        // format
        speed[i][speed_ID[i]] += // ticks per second
        		current_speed * SAMPLING_PER_SEC;

        speed[i][speed_ID[i]] /= SPEED_SMPL; //mean
    }
}

void DCMotor::rebase_position() {

    for(int i = 0; i < NB_MOTORS; i++){
    	ticks[i] = hardware->getTicks(i);
        if(ticks[i] > ENC_RESO) {
        	ticks[i] -= ENC_RESO;
        }
        if(ticks[i] < 0) {
        	ticks[i] += ENC_RESO;
        }
        position[i] = ticks[i];
    }
}

void DCMotor::set_speed_order(float lin, float rot) {
	constexpr int32_t meters_to_tick = 1024/(M_PI * 0.068);
	constexpr int32_t rad_to_tick = (0.25 * M_PI/2)/meters_to_tick;
	int32_t linear_speed = meters_to_tick * lin;// = resolution/perimeter = 1024/(pi*68mm) to convert from m/s => 4793
	int32_t rotational_speed_order = rad_to_tick * rot;// = distance by wheel for half turn / speed in m/s = (250mm*pi/2) / 4793 to convert from rad/s => 1882

	int32_t left_speed_order = linear_speed+rotational_speed_order;
	left_speed_order = MIN(left_speed_order, SPEED_MAX);
	left_speed_order = MAX(left_speed_order, -SPEED_MAX);
	speed_order[M_L] = left_speed_order;

	int32_t right_speed_order = linear_speed-rotational_speed_order;
	right_speed_order = MIN(right_speed_order, SPEED_MAX);
	right_speed_order = MAX(right_speed_order, -SPEED_MAX);
	speed_order[M_R] = right_speed_order;
}
