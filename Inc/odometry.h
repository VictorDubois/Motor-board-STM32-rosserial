/*
 * odometry.h
 *
 *  Created on: 31 juil. 2021
 *      Author: victo
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

class DCMotor;

class Odometry
{
public:
	Odometry();
	~Odometry();
	static float ticksToMillimeters(int32_t ticks);
	static int32_t millimetersToTicks(float millimeters);

	void update();
	float getX(){return X;};
	float getY(){return Y;};
	float getTheta(){return current_theta_rad;};
	float getLinearSpeed(){return m_linear_speed;};
	float getAngularSpeed(){return m_angular_speed;};
	void setDCMotor(DCMotor* motors);

private:
	float compute_linear_dist(const long encoder_left, const long encoder_right);
	float get_orientation_float(long encoder1, long encoder2);
	int fixOverflow(long after, long before);

	DCMotor* motors;
	volatile long last_encoder_left = 0;
	volatile long last_encoder_right = 0;
	float X;
	float Y;
	float theta_offset;
	float current_theta_rad;
	float m_linear_speed;
	float m_angular_speed;
};
#endif /* ODOMETRY_H_ */
