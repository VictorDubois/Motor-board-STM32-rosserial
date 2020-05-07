/*
 * DCMotor.h
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "DCMotorHardware.h"
#include "stm32f3xx_hal.h"

#define M_L             0
#define M_R             1
#define NB_MOTORS 		2

#define ENC_RESO        1024
#define HALF_ENC_RESO   512

#define SMPL            10 // nb of samples in memory
#define SPEED_SMPL      10 // nb of sample to perform average

#define SAMPLING_USEC   10000 //microseconds
#define SAMPLING_PER_SEC 100 // Hz

#define AWINDUP         512 //255

#define SPEED_MAX       2048//2048 // ticks per s = 2tr/s
#define ACCEL_MAX       1024//1024 // ticks per s per s
#define S_KP            0.07 //0.08
#define S_KI            0.0015 //0.002

#ifndef M_PI
#define M_PI 3.14159265359
#endif

class DCMotor
{

public:
	DCMotor(DCMotorHardware* hardware);
	~DCMotor();

	/**
	 * @brief Sets the target linear and rotational speeds
	 * @param lin target linear speed, in m/s
	 * @param rot target rotational speed, in rad/s
	 */
	void set_speed_order(float lin, float rot);

	/**
	 * Runs the main loop
	 */
	void update();
private:
	int32_t speed_order[NB_MOTORS];
	DCMotorHardware* hardware;

	int32_t position[NB_MOTORS];
	int32_t last_position[NB_MOTORS];

	/**
	 * @brief
	 */
	void rebase_position();

	void get_speed();

	volatile long ticks[NB_MOTORS];

	int dir[NB_MOTORS];

	volatile int32_t total_ticks[NB_MOTORS];

	volatile int32_t speed[NB_MOTORS][SMPL];// samples to average
	volatile uint8_t speed_ID[NB_MOTORS];

	volatile int32_t speed_error[NB_MOTORS][SMPL];// samples to integrate
	volatile uint8_t speed_error_ID[NB_MOTORS];

	volatile int32_t speed_integ_error[NB_MOTORS];

	volatile int32_t voltage[NB_MOTORS];//no need to have mutliple samples

	int32_t speed_command[NB_MOTORS];
};


#endif /* DCMOTOR_H_ */
