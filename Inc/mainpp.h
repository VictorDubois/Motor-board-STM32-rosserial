/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#include <geometry_msgs/Twist.h>
#include "stm32f3xx_hal.h"

void cmd_vel_cb(const geometry_msgs::Twist& twist);

#ifdef __cplusplus
 extern "C" {
#endif


void setup(TIM_HandleTypeDef* motorTimHandler);
void loop(void);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
