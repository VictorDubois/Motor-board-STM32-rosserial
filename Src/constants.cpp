/*
 * constants.cpp
 *
 *  Created on: 28 juil. 2021
 *      Author: victo
 */

 #include "constants.h"

float ticksToMillimeters(int32_t ticks)
{
	return (DIST_PER_REVOLUTION * (float)ticks / TICKS_PER_REVOLUTION);
}

int32_t millimetersToTicks(float millimeters)
{
	return static_cast<int32_t>(millimeters * TICKS_PER_REVOLUTION/DIST_PER_REVOLUTION);
}
