/*
 * odometry.cpp
 *
 *  Created on: 31 juil. 2021
 *      Author: victo
 */

#include "odometry.h"
#include "constants.h"
#include "math.h"
#include "DCMotor.h"

Odometry::Odometry():
	last_encoder_left(0),
	last_encoder_right(0),
	X(0),
	Y(0),
	theta_offset(0),
	current_theta_rad(0),
	m_linear_speed(0),
	m_angular_speed(0)
{
}

Odometry::~Odometry()
{
}

void Odometry::setDCMotor(DCMotor* a_motors)
{
	motors = a_motors;
}

/*
*    Return the Robot's orientation, in degrees, with respect to the last encoder reset.
*/
float Odometry::get_orientation_float(long encoder1, long encoder2)
{
    int absolute_orientation = fmod((encoder2 - encoder1) / TICKS_PER_DEG, 360);

    if (absolute_orientation >= 0)
        return (absolute_orientation);
    else
        return (360.f + absolute_orientation); // reminder: abs_ori is < 0 here
}

int Odometry::fixOverflow(long after, long before)
{
    if (after - before > TICKS_half_OVERFLOW)
    {
        // printf("before (%ld) - after (%ld) > TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // before, after, TICKS_half_OVERFLOW, after - before - TICKS_OVERFLOW);
        return after - before - TICKS_OVERFLOW;
    }
    if (after - before < -TICKS_half_OVERFLOW)
    {
        // printf("after (%ld) - before (%ld) < -TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // after, before, -TICKS_half_OVERFLOW, after - before + TICKS_OVERFLOW);
        return after - before + TICKS_OVERFLOW;
    }
    return after - before;
}


/*
        Given current value of both encoders
        return the linear dist by approximating it as the average of both wheels' linear distances.
        Static variables are used to keep last value of encoders.
*/
float Odometry::compute_linear_dist(const long encoder1, const long encoder2)
{
    float dist1, dist2, dist;
    int diff_encoder1, diff_encoder2;

    // Compute difference in nb of ticks between last measurements and now
    diff_encoder1 = fixOverflow(encoder1, last_encoder_left);
    diff_encoder2 = fixOverflow(encoder2, last_encoder_right);

    // Compute each wheel's dist and approximate linear dist as their average
    dist1 = ticksToMillimeters(diff_encoder1);
    dist2 = ticksToMillimeters(diff_encoder2);
    dist = (dist1 + dist2) / 2.0f;

    if (fabsf(dist) > 500.)
    {
        //printf("\n/!\\ HIGH SPEED DETECTED: %.2f /!\\\n\n", dist);
        // exit(4);
    }

    // Update static variables' values (current encoder values become old ones)
    last_encoder_left = encoder1;
    last_encoder_right = encoder2;

    // Return the computed linear dist
    return dist / 1000.f; // convert to meters
}

float Odometry::ticksToMillimeters(int32_t ticks)
{
	return (DIST_PER_REVOLUTION * (float)ticks / TICKS_PER_REVOLUTION);
}

int32_t Odometry::millimetersToTicks(float millimeters)
{
	return static_cast<int32_t>(millimeters * TICKS_PER_REVOLUTION/DIST_PER_REVOLUTION);
}

void Odometry::update()
{
	int32_t encoder_left = motors->get_encoder_ticks(M_L);
	int32_t encoder_right = motors->get_encoder_ticks(M_R);
	int32_t right_speed = motors->get_speed(M_R);
	int32_t left_speed = motors->get_speed(M_L);

	float linear_dist = compute_linear_dist(encoder_left, encoder_right);
	float current_theta = get_orientation_float(encoder_left, encoder_right);
	current_theta += theta_offset;

	current_theta_rad = current_theta * M_PI / 180.f;

	X += linear_dist * cos(current_theta_rad);
	Y += linear_dist * sin(current_theta_rad);

	m_linear_speed = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	m_angular_speed = ticksToMillimeters((left_speed-right_speed)/2)/1000.f;
}
