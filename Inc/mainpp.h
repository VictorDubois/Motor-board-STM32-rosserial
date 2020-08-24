/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#include <geometry_msgs/Twist.h>
#include <goal_strategy/motors.h>
#include <std_msgs/Bool.h>
#include <ros.h>
#include <std_msgs/String.h>
//#include <nav_msgs/Odometry.h>
#include <goal_strategy/motors.h>
#include <goal_strategy/motors_cmd.h>
#include <MCP3002.h>
#include "stm32f3xx_hal.h"
#include "DCMotor.h"
goal_strategy::encoders encoders_msg;
goal_strategy::motors motors_msg;
ros::Publisher encoders_pub("encoders", &encoders_msg);
ros::Publisher motors_pub("motors", &motors_msg);
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
//nav_msgs::Odometry odom_msg;
//ros::Publisher odom_pub("odom", &odom_msg);


void cmd_vel_cb(const geometry_msgs::Twist& twist);
void enable_motor_cb(const std_msgs::Bool& enable);

class MotorBoard
{
public:
	MotorBoard(TIM_HandleTypeDef* motorTimHandler);
	MotorBoard();
	~MotorBoard();

	static ros::NodeHandle& getNodeHandle(void);
	static DCMotor& getDCMotor(void);

	void update();
private:
	static ros::NodeHandle nh;
	static DCMotorHardware motorsHardware;
	static DCMotor motors;
	static MCP3002 currentReader;
};

#ifdef __cplusplus
 extern "C" {
#endif


void setup();
void loop(TIM_HandleTypeDef* motorTimHandler, TIM_HandleTypeDef* loopTimHandler);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
