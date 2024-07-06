/**
	integrator_backstepping.h
	
	Created on: 2024/06/22
		Author: Paulin LEDESERT
*/

#ifndef INTEGRATOR_BACKSTEPPING_H
#define INTEGRATOR_BACKSTEPPING_H
#include <array>
#include "stm32f3xx_hal.h"

static inline void substract_array_from_array(std::array<float, 2>& array, const std::array<float, 2>& array_to_substract) {
	array[0] -= array_to_substract[0];
	array[1] -= array_to_substract[1];
}

static inline void copy_array(std::array<float,2>& dest_array, const std::array<float, 2>& src_array){
	dest_array[0] = src_array[0];
	dest_array[1] = src_array[1];
}

class IntegratorBackstepping{
public:
	IntegratorBackstepping(float kx1, float k2x, float ky1, float k2y, float static_friction_speed, uint32_t interval_in_millis);
	void calculate(float& linear_velocity, float& angular_velocity);
	void set_command(const float& dest_x, float& des_y);
	void set_pos(const float& x, const float& y, const float& theta);
private:
	const float kx1;
	const float k2x;
	const float ky1;
	const float k2y;
	float static_friction_speed;
	const float interval;
	std::array<float, 2> command;
	std::array<float, 2> previous_command;
	std::array<float, 2> d_command;
	std::array<float, 2> dd_command;
	std::array<float, 3> pos;
	std::array<float, 2> dpos;
	float forward_speed;
};

#endif
