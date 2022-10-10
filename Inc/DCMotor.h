/*
 * DCMotor.h
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#include "DCMotorHardware.h"
#include "MCP3002.h"
#include "stm32f3xx_hal.h"

#define M_L             0
#define M_R             1
#define NB_MOTORS 		2

#define ENC_RESO        1024 * 4

#define ENC_BUF_SIZE 65536
#define HALF_ENC_BUF_SIZE 32768

#define SAMPLING_USEC   10000 //microseconds
#define SAMPLING_PER_SEC 1e6/SAMPLING_USEC // Hz

#define SPEED_MAX       20 * 2048// Not used anymore //2048 // ticks per s = 20 * 0.5tr/s
#define ACCEL_MAX       20 * 1024// Not used anymore //1024 // ticks per s per s = 20 * 0.25tr/s/s

#define KU  0.037
#define TU  0.071
#define S_KP            KU * 0.45//Ku = 0.037//0.07 //0.08
#define S_KI            0.54 * (KU/TU)//0.00015//0.0015 //0.002
#define S_KD            0//0.00015//0.0015 //0.002

#ifndef M_PI
#define M_PI 3.14159265359
#endif

class DCMotor
{

public:
	DCMotor();
	DCMotor(DCMotorHardware* hardware, MCP3002* current_reader);
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
	int32_t get_speed(const uint8_t motor_id);

	/**
	 * @brief getter for the current ticks of a given encoder
	 * @param encoder_id the ID of the encoder
	 * @return the encoder's current positions
	 */
	int32_t get_encoder_ticks(const uint8_t encoder_id);

	/**
	 * @brief getter for the accumulated current consumed by the motor over 20 iterations
	 * @param motor_id the ID of the motor
	 * @return the current
	 */
	int32_t get_accumulated_current(const uint8_t motor_id);

	/**
	 * @brief getter for the current consumed by the motor
	 * @param motor_id the ID of the motor
	 * @return the current
	 */
	int32_t get_current(const uint8_t motor_id);


	int32_t get_linear_error();
	float get_linear_error_integ();
	float get_dt(){ return dt;}

	/**
	 * @brief stop the motors and reset the asserv
	 */
	void resetMotors();

	/**
	 * @brief stop the motor and reset the asserv
	 */
	void resetMotor(int motor_id);

	int32_t get_voltage(int8_t a_motor_id);

	/**
	 * @brief setter for the max speed (per motor)
	 * @param a_max_speed the new max speed, in tick/s
	 */
	void set_max_speed(int32_t a_max_speed);

	/**
	 * @brief setter for the max acceleration (per motor)
	 * @param a_max_acceleration the new max acceleration, in tick/(s^2)
	 */
	void set_max_acceleration(int32_t a_max_acceleration);

	/**
	 * @brief setter pid coefficient proportional
	 * @param a_pid_p the new p coefficient
	 */
	void set_pid_p(float a_pid_p);

	/**
	 * @brief setter pid coefficient integral
	 * @param a_pid_p the new i coefficient
	 */
	void set_pid_i(float a_pid_i);

	void set_pid_d(float a_pid_d);

	/**
	 * @brief setter for the overcurrent
	 * @param a_max_current the overcurrent threshold
	 */
	void set_max_current(float a_max_current);

	/**
	 * @brief setter for the overcurrent per motor. Used to be able to stop one motor only
	 * @param a_max_current_left the overcurrent threshold for the left motor
	 * @param a_max_current_right the overcurrent threshold for the right motor
	 */
	void set_max_current(float a_max_current_left, float a_max_current_right);

	void override_PWM(int pwm_left, int pwm_right);
	void stop_pwm_override();

	void set_enable_motors(bool a_enable_motors);


	int32_t get_linear_speed_order() {return linear_speed_order;}
	int32_t get_angular_speed_order() {return angular_speed_order;}
private:
	float dt;
	volatile int32_t speed_order[NB_MOTORS];
	DCMotorHardware* hardware;
	MCP3002* current_reader;

	volatile int32_t last_position[NB_MOTORS];

	void get_speed();
	void control_ramp_speed(void);
	void control_ramp_speed_polar(void);


	void overCurrentProtection();


	int dir[NB_MOTORS];

	volatile int32_t speed[NB_MOTORS];// samples to average

	//volatile int32_t speed_error[NB_MOTORS];// samples to integrate
	volatile int32_t last_speed_error[NB_MOTORS];// samples to integrate

	volatile int32_t speed_integ_error[NB_MOTORS];

	volatile int32_t voltage[NB_MOTORS];//no need to have multiple samples

	int32_t accumulated_current[NB_MOTORS];
	int32_t current[NB_MOTORS];

	volatile int32_t stopped_timeout;
	volatile int32_t stopped_timeouts[NB_MOTORS];

	int32_t refined_speed_order[NB_MOTORS];

	int32_t max_speed;
	int32_t max_speed_delta;
	float pid_p;
	float pid_i;
	float pid_d;
	float max_current;

	float max_currents[NB_MOTORS];

	bool override_pwm;
	int override_pwms[NB_MOTORS];

	bool m_enable_motors;


	int32_t linear_speed_order;
	int32_t angular_speed_order;
	int32_t linear_refined_speed_order;
	int32_t angular_refined_speed_order;
	int32_t linear_last_speed_error;
	int32_t angular_last_speed_error;
	int32_t linear_speed;
	int32_t angular_speed;
	float linear_speed_integ_error;
	float angular_speed_integ_error;

	volatile int32_t last_update_time;
};


#endif /* DCMOTOR_H_ */
