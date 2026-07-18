/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#include "stm32g4xx_hal.h"
#include "DCMotor.h"
#include <CurrentReader.h>
#include "canManager.h"

#define UPDATE_FREQ 10
#define MS_BETWEEN_UPDATES 1000/UPDATE_FREQ

float get_orientation_float(int32_t encoder1, int32_t encoder2, float offset);


class MotorBoard
{
public:
	MotorBoard(TIM_HandleTypeDef* motorTimHandler, UART_HandleTypeDef * huart2, FDCAN_HandleTypeDef* hcan, ADC_HandleTypeDef* hadc2);
	MotorBoard();
	~MotorBoard();

	static DCMotor& getDCMotor(void);
	static void set_odom(float a_x, float a_y, float a_theta);

	void update();
	void update_inputs();
	void updateCurrent();
private:
	static DCMotorHardware motorsHardware;
	static DCMotor motors;
	static CurrentReaderCan currentReader;
	static volatile int32_t last_encoder_left;
	static volatile int32_t last_encoder_right;
	CanBroker canBroker;

	static volatile int32_t int32_t_encoder_left;
	static volatile int32_t int32_t_encoder_right;
	float compute_linear_dist(const long encoder_left, const long encoder_right);
	static float X;
	static float Y;
	static float theta_offset;
	volatile static long long message_counter;
	UART_HandleTypeDef * huart2;
	FDCAN_HandleTypeDef * hcan;
};

void doResetUart(UART_HandleTypeDef * huart2);
void toggleLed();

#ifdef __cplusplus
 extern "C" {
#endif


void setup();
void loop(TIM_HandleTypeDef* motorTimHandler, TIM_HandleTypeDef* loopTimHandler, UART_HandleTypeDef * huart2, FDCAN_HandleTypeDef* hcan, ADC_HandleTypeDef* hadc2);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
