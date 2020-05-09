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

#define ENC_BUF_SIZE 65536
#define HALF_ENC_BUF_SIZE 32768

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
	DCMotor();
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

	/**
	 * @brief getter for the current speed of a given motor
	 * @param motor_id the ID of the motor for which we want the speed
	 * @return the motor's current speed
	 */
	int32_t get_speed(uint8_t motor_id);

	/**
	 * @brief getter for the current ticks of a given encoder
	 * @param encoder_id the ID of the encoder
	 * @return the encoder's current positions
	 */
	int32_t get_encoder_ticks(uint8_t encoder_id);
private:
	int32_t speed_order[NB_MOTORS];
	DCMotorHardware* hardware;

	volatile int32_t last_position[NB_MOTORS];

	void get_speed();

	volatile int32_t ticks[NB_MOTORS];

	int dir[NB_MOTORS];

	volatile int32_t speed[NB_MOTORS][SMPL];// samples to average
	volatile uint8_t speed_ID[NB_MOTORS];

	volatile int32_t speed_error[NB_MOTORS][SMPL];// samples to integrate
	volatile uint8_t speed_error_ID[NB_MOTORS];

	volatile int32_t speed_integ_error[NB_MOTORS];

	volatile int32_t voltage[NB_MOTORS];//no need to have mutliple samples

	int32_t speed_command[NB_MOTORS];
};


#endif /* DCMOTOR_H_ */
