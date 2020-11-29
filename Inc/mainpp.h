/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#include <geometry_msgs/Twist.h>
#include <krabi_msgs/motors.h>
#include <krabi_msgs/odom_light.h>
#include <krabi_msgs/motorsParams.h>
#include <std_msgs/Bool.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <krabi_msgs/motors_cmd.h>
#include <krabi_msgs/SetOdom.h>
#include <MCP3002.h>
#include "stm32f3xx_hal.h"
#include "DCMotor.h"

#define UPDATE_FREQ 10
#define MS_BETWEEN_UPDATES 1000/UPDATE_FREQ

krabi_msgs::encoders encoders_msg;
krabi_msgs::motors motors_msg;
//krabi_msgs::odom_light odom_light_msg;
//ros::Publisher encoders_pub("encoders", &encoders_msg);
//ros::Publisher motors_pub("motors", &motors_msg);
//ros::Publisher odom_light_pub("odom_light", &odom_light_msg);

std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

float get_orientation_float(long encoder1, long encoder2);
int fixOverflow(long after, long before);
float ticksToMillimeters(int32_t ticks);
int32_t millimetersToTicks(float millimeters);

void cmd_vel_cb(const geometry_msgs::Twist& twist);
void enable_motor_cb(const std_msgs::Bool& enable);
void set_odom_cb(const krabi_msgs::SetOdomRequest &req, krabi_msgs::SetOdomResponse &res);
void set_motors_param_cb(const krabi_msgs::motorsParamsRequest &req, krabi_msgs::motorsParamsResponse &res);

class MotorBoard
{
public:
	MotorBoard(TIM_HandleTypeDef* motorTimHandler);
	MotorBoard();
	~MotorBoard();

	static ros::NodeHandle& getNodeHandle(void);
	static DCMotor& getDCMotor(void);
	static void set_odom(float a_x, float a_y, float a_theta);

	void update();
private:
	static ros::NodeHandle nh;
	static DCMotorHardware motorsHardware;
	static DCMotor motors;
	static MCP3002 currentReader;
	volatile long last_encoder_left = 0;
	volatile long last_encoder_right = 0;
	float compute_linear_dist(const long encoder_left, const long encoder_right);
	static float X;
	static float Y;
	static float theta_offset;
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
