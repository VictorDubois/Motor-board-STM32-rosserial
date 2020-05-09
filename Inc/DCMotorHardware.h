/*
 * DCMotorHardware.h
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#ifndef DCMOTORHARDWARE_H_
#define DCMOTORHARDWARE_H_
#include <stdint.h>
#include "stm32f3xx_hal.h"
#define M_L             0
#define M_R             1
#define NB_MOTORS 		2
#define DUTYMAX         0xFF//PWM_MAX

#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))

class DCMotorHardware
{
public:
	DCMotorHardware();
	DCMotorHardware(GPIO_TypeDef* dir_right_gpio_bank,
			const uint16_t dir_right_gpio,
			GPIO_TypeDef* dir_left_gpio_bank,
			const uint16_t dir_left_gpio,
			TIM_TypeDef* encoder_right_timer,
			TIM_TypeDef* encoder_left_timer,
			TIM_HandleTypeDef* motor_right_timer,
			const int32_t motor_right_timer_channel,
			TIM_HandleTypeDef* motor_left_timer,
			const int32_t motor_left_timer_channel);
	~DCMotorHardware();

	uint32_t getTicks(uint32_t motorId);

	/**
	 *
	 * @brief Send PWM values to timers, and control direction pins
	 * @param pwm_left the pwm intensity for motor left
	 * @param pwm_right the pwm intensity for motor right
	 */
	void setPWM(int32_t pwm_left, int32_t pwm_right);
private:
	GPIO_TypeDef* dir_right_gpio_bank;
	uint16_t dir_right_gpio;
	GPIO_TypeDef* dir_left_gpio_bank;
	uint16_t dir_left_gpio;
	TIM_TypeDef* encoder_right_timer;
	TIM_TypeDef* encoder_left_timer;
	TIM_HandleTypeDef* motor_right_timer;
	int32_t motor_right_timer_channel;
	TIM_HandleTypeDef* motor_left_timer;
	int32_t motor_left_timer_channel;
};


#endif /* DCMOTORHARDWARE_H_ */
