/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

ros::Publisher encoders_pub("encoders", &str_msg);
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

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);

  //pinMode(BRAKE, OUTPUT);
  //digitalWrite(BRAKE, LOW);

  nh.advertise(encoders_pub);
  nh.subscribe(twist_sub);
}

void loop(void)
{
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	str_msg.data = hello;
	chatter.publish(&str_msg);
	nh.spinOnce();

	HAL_Delay(1000);
}
