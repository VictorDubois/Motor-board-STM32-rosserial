/*
 * DCMotorHardware.cpp
 *
 *  Created on: May 7, 2020
 *      Author: victor
 */

#include "DCMotorHardware.h"


DCMotorHardware::DCMotorHardware(GPIO_TypeDef* a_dir_right_gpio_bank,
		const uint16_t a_dir_right_gpio,
		GPIO_TypeDef* a_dir_left_gpio_bank,
		const uint16_t a_dir_left_gpio,
		TIM_TypeDef* a_encoder_right_timer,
		TIM_TypeDef* a_encoder_left_timer,
		TIM_HandleTypeDef* a_motor_right_timer,
		const int32_t a_motor_right_timer_channel,
		TIM_HandleTypeDef* a_motor_left_timer,
		const int32_t a_motor_left_timer_channel) {
	dir_right_gpio_bank = a_dir_right_gpio_bank;
	dir_right_gpio = a_dir_right_gpio;
	dir_left_gpio_bank = a_dir_left_gpio_bank;
	dir_left_gpio = a_dir_left_gpio;
	encoder_right_timer = a_encoder_right_timer;
	encoder_left_timer = a_encoder_left_timer;
	motor_right_timer = a_motor_right_timer;
	motor_right_timer_channel = a_motor_right_timer_channel;
	motor_left_timer = a_motor_left_timer;
	motor_left_timer_channel = a_motor_left_timer_channel;
}
DCMotorHardware::DCMotorHardware() {}
DCMotorHardware::~DCMotorHardware() {}

uint32_t DCMotorHardware::getTicks(const uint32_t encoderId) {
	if (encoderId == M_L) {
		return encoder_left_timer->CNT;
	}
	return encoder_right_timer->CNT;
}

void DCMotorHardware::setPWM(const int32_t pwm_left, const int32_t pwm_right) {
	// Write dir according to pwm sign
	if (pwm_left > 0) {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dir_left_gpio_bank, dir_left_gpio, GPIO_PIN_RESET);
	}
	else {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dir_left_gpio_bank, dir_left_gpio, GPIO_PIN_SET);
	}

	if (pwm_right > 0) {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dir_right_gpio_bank, dir_right_gpio, GPIO_PIN_RESET);
	}
	else {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dir_right_gpio_bank, dir_right_gpio, GPIO_PIN_SET);
	}

	// Limit to pwm boundaries and write PWM
	int32_t command = 0;
	command = MAX(pwm_left, -pwm_left);
	command = MIN(command, DUTYMAX);
	__HAL_TIM_SET_COMPARE(motor_left_timer, TIM_CHANNEL_1, command);
	//__HAL_TIM_SET_COMPARE(motor_left_timer, motor_left_timer_channel, command);

	command = MAX(pwm_right, -pwm_right);
	command = MIN(command, DUTYMAX);
	__HAL_TIM_SET_COMPARE(motor_right_timer, TIM_CHANNEL_4, command);
	//__HAL_TIM_SET_COMPARE(motor_right_timer, motor_right_timer_channel, command);
}
