/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <goal_strategy/motors.h>
#include <goal_strategy/motors_cmd.h>
#include "DCMotor.h"

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";
goal_strategy::encoders encoders_msg;

ros::Publisher encoders_pub("encoders", &encoders_msg);
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", cmd_vel_cb);

void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
    //set_speed_order(twist.linear.x, twist.angular.z);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(TIM_HandleTypeDef* motorTimHandler)
{
	DCMotorHardware motorsHardware = DCMotorHardware(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, TIM1, TIM2, motorTimHandler, TIM_CHANNEL_4, motorTimHandler, TIM_CHANNEL_1);
	DCMotor motors = DCMotor(&motorsHardware);
  nh.initNode();
  nh.advertise(chatter);

  //pinMode(BRAKE, OUTPUT);
  //digitalWrite(BRAKE, LOW);

  nh.advertise(encoders_pub);
  nh.subscribe(twist_sub);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);//LED
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//BRAKE
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);//DIR_A
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);//DIR_B
}

void loop(void)
{
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);


	encoders_msg.encoder_left = TIM1->CNT;;
	encoders_msg.encoder_right = TIM2->CNT;;
	encoders_pub.publish(&encoders_msg);


	str_msg.data = hello;
	chatter.publish(&str_msg);
	nh.spinOnce();

	HAL_Delay(1000);
}
