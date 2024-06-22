/*
 * IntegratorBackstepping.cpp
 *
 *  Created on: Jun 22, 2024
 *      Author: pauloone
 **/
#include <IntegratorBackstepping.h>
#include <cmath>

IntegratorBackstepping::IntegratorBackstepping(float kx1,float k2x, float ky1, float k2y, float static_friction_speed, uint32_t interval_in_millis):
	kx1(kx1), k2x(k2x), ky1(ky1), k2y(k2y),
	static_friction_speed(static_friction_speed), interval(0.001f*interval_in_millis),
	command({0,0}), previous_command({0,0}), d_command({0,0}), dd_command({0,0}),
	pos({0,0,0}), dpos({0,0}), forward_speed(0){};

void IntegratorBackstepping::set_command(const float& dest_x, float& dest_y){
	this->command[0] = dest_x;
	this->command[1] = dest_y;
}

void IntegratorBackstepping::set_pos(const float& x, const float& y, const float& theta){
	this->dpos[0] = x - this->pos[0];
	this->dpos[1] = y - this->pos[1];
	this->pos[0] = x;
	this->pos[1] = y;
	this->pos[2] = theta;
}

void IntegratorBackstepping::calculate(float& linear_velocity, float& angular_velocity){
	//see https://github.com/pauloone/hercule_asserv/blob/main/hercule/backstepping.py for simulation


	//We calculate the command derivatives
	std::array<float, 2> d_command;
	copy_array(d_command, command);
	substract_array_from_array(d_command, this->previous_command);
	copy_array(this->previous_command, this->command);
	copy_array(this->dd_command, this->d_command);
	substract_array_from_array(this->dd_command, this->d_command);
	this->d_command = std::move(d_command);

	//Integrator backstepping
	float v1 = - this->kx1 * (this->dpos[0] - this->d_command[0]) + this->dd_command[0] - this->k2x * (this->dpos[0] - this->d_command[0] + this->kx1 * (this->pos[0] - this->command[0]));
	float v2 = - this->ky1 * (this->dpos[1] - this->d_command[1]) + this->dd_command[1] - this->k2y * (this->dpos[1] - this->d_command[1] + this->ky1 * (this->pos[1] - this->command[1]));

	//decoupling
	float d_forward_speed = v1 * std::cos(this->pos[2]) + v2 * std::sin(this->pos[2]);
	this->forward_speed += d_forward_speed * this->interval;

	float forward_speed_with_friction = this->forward_speed;
	if (forward_speed_with_friction < 0){
		forward_speed_with_friction = std::min(forward_speed_with_friction, -this->static_friction_speed);
	} else {
		forward_speed_with_friction = std::max(forward_speed_with_friction, this->static_friction_speed);
	}
	linear_velocity = this->forward_speed;
	angular_velocity = (v2 * std::cos(this->pos[2]) - v1 * std::sin(this->pos[2]))/forward_speed_with_friction;
}
