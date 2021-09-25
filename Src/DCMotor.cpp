/*
 * DCMotor.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotor.h"
#include "constants.h"
#include "odometry.h"
#include <math.h>

DCMotor::DCMotor(DCMotorHardware* a_hardware, MCP3002* a_current_reader) : hardware(a_hardware), current_reader(a_current_reader) {
	resetMotors();
	max_speed = SPEED_MAX;
	max_speed_delta = ACCEL_MAX;
	set_max_current(0.5f);
	set_max_current(10.f, 10.f);
	pid_p = S_KP;
	pid_i = 0.0028;
	for (int i = 0; i< NB_MOTORS; i++) {
		last_position[i] = 0;
		current[i] = 0;
		accumulated_current[i] = 0;
		speed[i] = 0;
	}

	l_distance_to_goal = 42.f;
	time_to_stop = 69.f;
}

void DCMotor::setOdometry(Odometry* a_odometry)
{
	odometry = a_odometry;
}

void DCMotor::resetMotor(int motor_id) {
	use_distance_asserv = false;
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
	max_speed_delta = ACCEL_MAX;
	pid_p = S_KP;
	pid_i = 0.0028;
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

	if (stopped_timeout > 0 && stopped_timeout < 10000) {
		stopped_timeout--;
		resetMotors();
	}
	else {
		for(int i = 0; i < NB_MOTORS; i++){
			if (stopped_timeouts[i] > 0&& stopped_timeouts[i] < 10000){
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

	limitLinearSpeedCmdByGoal();

    for(int i = 0; i < NB_MOTORS; i++){
        if( (int32_t)(speed_order[i]) - speed[i] >= max_speed_delta) {
        	speed_command[i] = speed[i]+max_speed_delta;
        }
        else if ( (int32_t)(speed_order[i]) - speed[i] <= -max_speed_delta ) {
        	speed_command[i] = speed[i]-max_speed_delta;
        }
        else {
        	speed_command[i] = speed_order_limited[i];
        }

        speed_error[i] = speed_command[i] - speed[i];
        speed_integ_error[i] += speed_error[i]; // dt is included in pid_i because it is constant. If we change dt, pid_i must be scaled

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

int32_t DCMotor::get_error(int8_t a_motor_id)
{
	return speed_error[a_motor_id];
}

float DCMotor::get_time()
{
	return time_to_stop;
}

float DCMotor::get_distance()
{
	return distance_to_stop;//sqrt((goal_X - odometry->getX()) * (goal_X - odometry->getX()) + (goal_Y - odometry->getY()) * (goal_Y - odometry->getY()));//m

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

void DCMotor::set_max_current(float a_max_current)
{
	max_current = a_max_current * ONE_AMP;
}

void DCMotor::set_max_current(float a_max_current_left, float a_max_current_right)
{
	max_currents[M_L] = a_max_current_left * ONE_AMP;
	max_currents[M_R] = a_max_current_right * ONE_AMP;
}

void DCMotor::set_distance_asserv_params(bool a_use_distance_asserv, float a_goal_X, float a_goal_Y, float a_max_speed_at_arrival)
{
	use_distance_asserv = a_use_distance_asserv;
	goal_X = a_goal_X;
	goal_Y = a_goal_Y;
	max_speed_at_arrival = a_max_speed_at_arrival;
}

void DCMotor::limitLinearSpeedCmdByGoal()
{
	for(int ii = 0; ii< NB_MOTORS; ii++)
	{
		speed_order_limited[ii] = speed_order[ii];
	}

	if (odometry == nullptr || !use_distance_asserv)
	{
		return;
	}

    float max_acceleration = 0.15f; // m*s-2
    float max_deceleration = 0.15f; // m*s-2

    float new_speed_order = 0; // m/s

    //odometry->update();

    float l_linear_speed = odometry->getLinearSpeed();// m/s
    float desired_final_speed = max_speed_at_arrival; // m/s
    //float speed_order = Odometry::ticksToMillimeters((speed_command[M_L] + speed_command[M_R])/2)/1000.f;// m/s

    l_distance_to_goal =sqrt((goal_X - odometry->getX()) * (goal_X - odometry->getX()) + (goal_Y - odometry->getY()) * (goal_Y - odometry->getY()));//m

    time_to_stop = (l_linear_speed - desired_final_speed) / max_deceleration;
    time_to_stop = MAX(0.f, time_to_stop);
    //ROS_INFO_STREAM("time to stop = " << time_to_stop << "s, ");

    distance_to_stop
      = time_to_stop * (l_linear_speed - desired_final_speed) / 2.;
    //ROS_INFO_STREAM(", distance to stop = " << distance_to_stop << "m, ");

    // Compute extra time if accelerating
    float average_extra_speed =
      l_linear_speed + (max_acceleration / 2. + max_deceleration / 2.) / float(UPDATE_RATE);
    float extra_distance = 2 * average_extra_speed / float(UPDATE_RATE);

    if (l_distance_to_goal < distance_to_stop)
    {
        //ROS_INFO_STREAM("decelerate");
        new_speed_order = l_linear_speed - max_deceleration / float(UPDATE_RATE);
    }
    else if (l_distance_to_goal < l_linear_speed / float(UPDATE_RATE))
    {
        //ROS_INFO_STREAM("EMERGENCY BRAKE");
        new_speed_order = 0;
    }
    else if (l_distance_to_goal > distance_to_stop + extra_distance)
    {
        //ROS_INFO_STREAM("accelerate");
        new_speed_order = l_linear_speed + max_acceleration / float(UPDATE_RATE);
    }
    else
    {
        //ROS_INFO_STREAM("cruise speed");
        new_speed_order = l_linear_speed;
    }
    //ROS_INFO_STREAM("new speed: " << new_speed_order << " => " << l_linear_speed_cmd << std::endl);

    int32_t new_speed_order_tick = Odometry::millimetersToTicks(new_speed_order*1000);

    speed_order_limited[M_L] = MAX(-new_speed_order_tick, speed_order[M_L]);
    speed_order_limited[M_L] = MIN(new_speed_order_tick, speed_order[M_L]);
    speed_order_limited[M_R] = MAX(-new_speed_order_tick, speed_order[M_R]);
    speed_order_limited[M_R] = MIN(new_speed_order_tick, speed_order[M_R]);
}
