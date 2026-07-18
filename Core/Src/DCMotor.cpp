/*
 * DCMotor.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotor.h"
#include <cmath>        // std::abs
#include <constants.h>


template <class T>
T get_linear(T* array) {
	return (array[M_L] + array[M_R])/2;
}

template <class T>
T get_angular(T* array) {
	return (array[M_L] - array[M_R])/2;
}

int32_t fixOverflow(int16_t a_after, int32_t before)
{
	int32_t after = a_after;
    while (after - before > TICKS_half_OVERFLOW)
    {
        after -= TICKS_OVERFLOW;
    }
    while (after - before < -TICKS_half_OVERFLOW)
    {
        after += TICKS_OVERFLOW;
    }
    return after;
}

int32_t diffWithFixOverflow(int32_t after, int32_t before)
{
    if (after - before > TICKS_half_OVERFLOW)
    {
        return after - before - TICKS_OVERFLOW;
    }
    if (after - before < -TICKS_half_OVERFLOW)
    {
        return after - before + TICKS_OVERFLOW;
    }
    return after - before;
}

DCMotor::DCMotor(DCMotorHardware* a_hardware, CurrentReader* a_current_reader) : hardware(a_hardware), current_reader(a_current_reader) {
	resetMotors();
	max_speed = SPEED_MAX;
	max_speed_delta = ACCEL_MAX;
	set_max_current(0.5f);
	set_max_current(10.f, 10.f);
	
	pid_p = 0.0222;
	pid_i = 0.00625;
	pid_d = 0.0197;

	m_linear_pid_p = 0.15f;
	m_linear_pid_i = 0.3f;
	m_linear_pid_d = 0.0f;
	m_angular_pid_p = 0.20f;
	m_angular_pid_i = 0.10f;
	m_angular_pid_d = 0.0f;

	last_update_time = HAL_GetTick();

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
		refined_speed_order[i] = 0;
		speed_order[i] = 0;
		stopped_timeouts[i] = 0;
		last_speed_error[i] = 0;
		override_pwms[i] = 0;
		last_encoder_value[i] = hardware->getTicks(i);
	}

	linear_speed_order = 0;
	angular_speed_order = 0;
	linear_refined_speed_order = 0;
	angular_refined_speed_order = 0;
	linear_last_speed_error = 0;
	angular_last_speed_error = 0;
	linear_speed = 0;
	angular_speed = 0;
	linear_speed_integ_error = 0;
	angular_speed_integ_error = 0;

	m_enable_motors = false;
	dt=0.01f;
}

void DCMotor::resetEncodersCounter()
{
	hardware->resetEncodersCounter();
}


void DCMotor::override_PWM(int pwm_left, int pwm_right)
{
	// Reset all the motors variables, without sending a 0 command
	resetMotor(M_L);
	resetMotor(M_R);

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
	refined_speed_order[motor_id] = 0;
	speed_order[motor_id] = 0;
	override_pwms[motor_id] = 0;

	linear_speed_order = 0;
	angular_speed_order = 0;
	linear_refined_speed_order = 0;
	angular_refined_speed_order = 0;
	linear_last_speed_error = 0;
	angular_last_speed_error = 0;
	linear_speed = 0;
	angular_speed = 0;
	linear_speed_integ_error = 0;
	angular_speed_integ_error = 0;

	last_update_time = HAL_GetTick();
}

void DCMotor::resetMotors() {
	for (int i = 0; i< NB_MOTORS; i++) {
		resetMotor(i);
	}

	hardware->sendMotorSpeed(0, 0);
}

DCMotor::DCMotor() {}

DCMotor::~DCMotor() {}

/**
 * Computes the (simplified) integral of the current, over 1s
 * If it goes above a threshold, stops the motor for some time
 * There is one short threshold (1s), per motor (used for re-calibration against a wall, for instance)
 * And a longer threshold (3s), used for collisions
 */
void DCMotor::overCurrentProtection() {
	for(int i = 0; i < NB_MOTORS; i++){
		current[i] = current_reader->readCurrent(i);
		if (current[i] == CURRENT_READER_OFFLINE) {
			stopped_timeout = hardware->getMilliSecondsElapsed() + 3000;
			resetMotors();
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

	int32_t l_remaining_time_stopped = stopped_timeout - hardware->getMilliSecondsElapsed();

	if (l_remaining_time_stopped > 0) {
		m_remaining_time_stopped[0] = l_remaining_time_stopped;
		m_remaining_time_stopped[1] = l_remaining_time_stopped;
		resetMotors();
		return;
	}
	m_remaining_time_stopped[0] = 0;
	m_remaining_time_stopped[1] = 0;


	for(int i = 0; i < NB_MOTORS; i++){
		l_remaining_time_stopped = stopped_timeouts[i] - hardware->getMilliSecondsElapsed();
		if (l_remaining_time_stopped > 0){
			m_remaining_time_stopped[i] = l_remaining_time_stopped;
			resetMotor(i);
			override_pwms[i] = 0;
		}
	}
	//control_ramp_speed();
	control_ramp_speed_polar();

	if (!m_enable_motors) {
		hardware->sendMotorSpeed(0, 0);
		return;
	}

	if (override_pwm)
	{
		hardware->sendMotorSpeed(override_pwms[M_L], override_pwms[M_R]);
	}
	else {
		hardware->sendMotorSpeed(voltage[M_L], voltage[M_R]);
	}
}

void DCMotor::set_speed(int MOTOR_ID, int32_t a_new_speed)
{
	speed[MOTOR_ID] = a_new_speed;
}

void DCMotor::set_ticks(int MOTOR_ID, int16_t a_new_ticks)
{
	hardware->setTicks(MOTOR_ID, a_new_ticks);
}

void DCMotor::set_current(int MOTOR_ID, int16_t a_new_current)
{
#ifdef USE_C620_CURRENT
	current_reader->setCurrent(MOTOR_ID, a_new_current);
#endif
}


void DCMotor::get_speed(){
#ifndef USE_CAN_SPEED_ODOMETRY
    int32_t current_speed = 0;

    for(int i = 0; i < NB_MOTORS; i++){
    	int32_t new_position = hardware->getTicks(i);

    	current_speed = diffWithFixOverflow(new_position, last_position[i]);
    	last_position[i] = new_position;

        speed[i] = current_speed * SAMPLING_PER_SEC;// ticks per second
    }
#endif

    linear_speed = get_linear(speed);
    angular_speed = get_angular(speed);
}

int32_t DCMotor::get_linear_error() {
	return linear_last_speed_error;
}

float DCMotor::get_linear_error_integ() {
	return linear_speed_integ_error;
}

int32_t DCMotor::get_speed(uint8_t motor_id) {
	return speed[motor_id];
}

uint32_t DCMotor::get_remaining_time_stopped(uint8_t motor_id) {
	return m_remaining_time_stopped[motor_id];
}

int16_t DCMotor::get_encoder_ticks(uint8_t encoder_id) {
	int32_t l_encoder_value = hardware->getTicks(encoder_id);
	l_encoder_value = fixOverflow(l_encoder_value, last_encoder_value[encoder_id]);
	last_encoder_value[encoder_id] = l_encoder_value;
	return l_encoder_value;
}

int32_t DCMotor::get_accumulated_current(uint8_t motor_id) {
	return accumulated_current[motor_id];
}

int32_t DCMotor::get_current(uint8_t motor_id) {
	return current[motor_id];
}

void DCMotor::set_speed_order(int32_t lin, int32_t rot) {
	linear_speed_order = lin;
	angular_speed_order = rot;

	limitLinearFirst(linear_speed_order, angular_speed_order, max_speed);

	if (linear_speed_order != lin || angular_speed_order!=rot)
	{
		// Debug only
		limitLinFirstWasUsed++;
	}

	speed_order[M_L] = linear_speed_order + angular_speed_order;
	speed_order[M_R] = linear_speed_order - angular_speed_order;
}

void DCMotor::limitLinearFirst(int32_t& linear, int32_t& angular, const int32_t max)
{
	linear = LIMIT(linear, max, -max);

	angular = LIMIT(angular, max, -max);

	// Limit the linear first => the robot must not be prevented from turning
	linear = LIMIT(linear, max - abs(angular), -max + abs(angular));
}

/**
 * PID in polar:
 * One PID for the linear speed
 * One PID for the angular speed
 */
void DCMotor::control_ramp_speed_polar(void) {
	float linear_pid_p = m_linear_pid_p;
	float linear_pid_i = m_linear_pid_i;
	float linear_pid_d = m_linear_pid_d;
	float angular_pid_p = m_angular_pid_p;
	float angular_pid_i = m_angular_pid_i;
	float angular_pid_d = m_angular_pid_d;

	int32_t linear_speed_error = linear_speed_order - linear_speed;
	int32_t angular_speed_error = angular_speed_order - angular_speed;

	limitLinearFirst(linear_speed_error, angular_speed_error, max_speed_delta);

	uint32_t current_time = HAL_GetTick(); // in ms
	dt = (current_time - last_update_time)/1000.f; // is seconds
	last_update_time = current_time;

	linear_speed_integ_error += linear_speed_error * dt;


	int32_t linear_voltage =
		 (linear_speed_error*linear_pid_p +
				 linear_speed_integ_error * linear_pid_i+ (linear_speed_error - linear_last_speed_error) * linear_pid_d/dt);

	linear_last_speed_error = linear_speed_error;


	angular_speed_integ_error += angular_speed_error* dt;

	int32_t angular_voltage =
		 (angular_speed_error * angular_pid_p +
				 angular_speed_integ_error * angular_pid_i + (angular_speed_error - angular_last_speed_error) * angular_pid_d/dt);

	angular_last_speed_error = angular_speed_error;


	voltage[M_L] = linear_voltage + angular_voltage;
	voltage[M_R] = linear_voltage - angular_voltage;

	for(int i = 0; i < NB_MOTORS; i++){
        voltage[i] = LIMIT(voltage[i], DUTYMAX, -DUTYMAX);
	}
}


/**
 * PID per-motor:
 * One PID for the left motor
 * One PID for the right motor
 */
void DCMotor::control_ramp_speed(void) {
    //if( stopped ) return;

	int32_t linear_speed_error = linear_speed_order - linear_speed;
	int32_t angular_speed_error = angular_speed_order - angular_speed;
	limitLinearFirst(linear_speed_error, angular_speed_error, max_speed_delta);

	refined_speed_order[M_L] = linear_speed_error + angular_speed_error;
	refined_speed_order[M_R] = linear_speed_error - angular_speed_error;



    for(int i = 0; i < NB_MOTORS; i++){
        volatile int32_t speed_error = refined_speed_order[i];
        speed_integ_error[i] += speed_error; // dt is included in pid_i because it is constant. If we change dt, pid_i must be scaled

        voltage[i] =
             (pid_p*speed_error +
             pid_i*speed_integ_error[i] + pid_d * (speed_error - last_speed_error[i]));

        last_speed_error[i] = speed_error;

        voltage[i] = LIMIT(voltage[i], DUTYMAX, -DUTYMAX);
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

void DCMotor::set_linear_pi(float p, float i)
{
	if (m_linear_pid_p != p || m_linear_pid_i !=i)
	{
		resetMotors();
	}
	m_linear_pid_p = p;
	m_linear_pid_i = i;
}

void DCMotor::set_angular_pi(float p, float i)
{
	if (m_angular_pid_p != p || m_angular_pid_i !=i)
	{
		resetMotors();
	}
	m_angular_pid_p = p;
	m_angular_pid_i = i;
}

void DCMotor::set_derivative(float linear_d, float angular_d)
{
	m_linear_pid_d = linear_d;
	m_angular_pid_d = angular_d;
}

void DCMotor::set_max_current(float a_max_current)
{
	max_current = a_max_current * current_reader->getOneAmp();
}

void DCMotor::set_max_current(float a_max_current_left, float a_max_current_right)
{
	max_currents[M_L] = a_max_current_left * current_reader->getOneAmp();
	max_currents[M_R] = a_max_current_right * current_reader->getOneAmp();
}

void DCMotor::set_enable_motors(bool a_enable_motors)
{
	m_enable_motors = a_enable_motors;

	if (!m_enable_motors)
	{
		resetMotor(M_L);
		resetMotor(M_R);
	}
}
