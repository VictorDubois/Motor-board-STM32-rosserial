/*
 * DCMotor.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotor.h"
#include <cmath>        // std::abs


template <class T>
T get_linear(T* array) {
	return (array[M_L] + array[M_R])/2;
}

template <class T>
T get_angular(T* array) {
	return (array[M_L] - array[M_R])/2;
}

DCMotor::DCMotor(DCMotorHardware* a_hardware, MCP3002* a_current_reader) : hardware(a_hardware), current_reader(a_current_reader) {
	resetMotors();
	max_speed = SPEED_MAX;
	max_speed_delta = ACCEL_MAX;
	set_max_current(0.5f);
	set_max_current(10.f, 10.f);
	pid_p = 0.0222;
	pid_i = 0.00625;
	pid_d = 0.0197;

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
	dt=0.42f;
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
		return;
	}

	for(int i = 0; i < NB_MOTORS; i++){
		if (stopped_timeouts[i] > hardware->getMilliSecondsElapsed()){
			resetMotor(i);
			override_pwms[i] = 0;
		}
	}
	//control_ramp_speed();
	control_ramp_speed_polar();

	if (!m_enable_motors) {
		hardware->setPWM(0, 0);
		return;
	}

	if (override_pwm)
	{
		hardware->setPWM(override_pwms[M_L], override_pwms[M_R]);
	}
	else {
		hardware->setPWM(voltage[M_L], voltage[M_R]);
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

int16_t DCMotor::get_encoder_ticks(uint8_t encoder_id) {
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

	linear_speed_order = meters_to_tick * lin;// = resolution/perimeter = 4096/(pi*68mm) to convert from m/s => 19172
	angular_speed_order = rad_to_tick * rot;// = radius when turning on the spot (=half entraxe) / speed in m/s = (250mm/2) * 19172 to convert from rad/s => 2396

	limitLinearFirst(linear_speed_order, angular_speed_order, max_speed);

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


void DCMotor::control_ramp_speed_polar(void) {
	// Ziegler Nichols: Ku = 0.1, Tu = 0.0844
	float linear_Ku = 0.1f;//0.085f;//0.1;
	float linear_Tu = 0.0813f;//0.0844;
	float linear_pid_p = 0.45f*linear_Ku;//0.06f;//0.38;//0.5 => explose. 0.1, 0.2 => marche.0.35, 0.38 => marche avec légère oscillation.0.4 oscille
	float linear_pid_i = 0.54f*linear_Ku/linear_Tu;//1.421800948f;
	float linear_pid_d = 0.0f*linear_Ku*linear_Tu;//0.000633f;

	/*linear_pid_p = 0.0f;//0.085f;//0.06f;//0.38;//0.5 => explose. 0.1, 0.2 => marche.0.35, 0.38 => marche avec légère oscillation.0.4 oscilleu
	linear_pid_i = 0;//1.421800948f;
	linear_pid_d = 0;*/


	// Ziegler Nichols: Ku = 0.15, Tu = 0.10855
	//0.1 => oscille beaucoup mais lin actif. 0.01 => ne bouge pas 0.03 lent. 0.09 marche, oscill un peu. 0.15 marche bien :)
	/*float angular_pid_p = 0.09f;
	float angular_pid_i = 1.105481345f;
	float angular_pid_d = 0.000814125f;*/

	float angular_Ku = 0.12f;
	float angular_Tu = 0.111f;
	float angular_pid_p = 0.45f*angular_Ku;
	float angular_pid_i = 0.54f*angular_Ku/angular_Tu;
	float angular_pid_d = 0.0f*angular_Ku*angular_Tu;

	/*angular_pid_p = 0.12f;
	angular_pid_i = 0;
	angular_pid_d = 0;*/

	int32_t linear_speed_error = linear_speed_order - linear_speed;
	int32_t angular_speed_error = angular_speed_order - angular_speed;

	limitLinearFirst(linear_speed_error, angular_speed_error, max_speed_delta);


	//linear_refined_speed_order


	uint32_t current_time = HAL_GetTick(); // in ms
	dt = (current_time - last_update_time)/1000.f; // is seconds
	last_update_time = current_time;

	linear_speed_integ_error += linear_speed_error * dt; // dt is included in pid_i because it is constant. If we change dt, pid_i must be scaled


	int32_t linear_voltage =
		 (linear_pid_p*linear_speed_error +
				 linear_pid_i*linear_speed_integ_error + linear_pid_d * (linear_speed_error - linear_last_speed_error)/dt);

	linear_last_speed_error = linear_speed_error;


	angular_speed_integ_error += angular_speed_error* dt; // dt is included in pid_i because it is constant. If we change dt, pid_i must be scaled

	int32_t angular_voltage =
		 (angular_pid_p*angular_speed_error +
				 angular_pid_i*angular_speed_integ_error + angular_pid_d * (angular_speed_error - angular_last_speed_error)/dt);

	angular_last_speed_error = angular_speed_error;


	voltage[M_L] = linear_voltage + angular_voltage;
	voltage[M_R] = linear_voltage - angular_voltage;

	for(int i = 0; i < NB_MOTORS; i++){
        voltage[i] = LIMIT(voltage[i], DUTYMAX, -DUTYMAX);
	}
}

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

void DCMotor::set_max_current(float a_max_current)
{
	max_current = a_max_current * ONE_AMP;
}

void DCMotor::set_max_current(float a_max_current_left, float a_max_current_right)
{
	max_currents[M_L] = a_max_current_left * ONE_AMP;
	max_currents[M_R] = a_max_current_right * ONE_AMP;
}

void DCMotor::set_enable_motors(bool a_enable_motors)
{
	m_enable_motors = a_enable_motors;

	if (!m_enable_motors)
	{
		resetMotors();
	}
}
