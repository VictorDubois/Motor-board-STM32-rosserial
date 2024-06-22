/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

/*#include <geometry_msgs/Twist.h>
#include <krabi_msgs/motors.h>
#include <krabi_msgs/odom_light.h>
#include <krabi_msgs/odom_lighter.h>
#include <krabi_msgs/motors_parameters.h>
#include <std_msgs/Bool.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <krabi_msgs/motors_cmd.h>
#include <krabi_msgs/SetOdom.h>*/
#include "msgs.h"
#include <MCP3002.h>
#include "stm32f3xx_hal.h"
#include "DCMotor.h"

#define UPDATE_FREQ 10
#define MS_BETWEEN_UPDATES 1000/UPDATE_FREQ

krabi_msgs::encoders encoders_msg;
krabi_msgs::motors motors_msg;
krabi_msgs::motors_parameters asserv_msg;
//krabi_msgs::odom_light odom_light_msg;
krabi_msgs::odom_lighter odom_lighter_msg;
/*ros::Publisher encoders_pub("encoders", &encoders_msg);
ros::Publisher motors_pub("motors", &motors_msg);
ros::Publisher odom_light_pub("odom_light", &odom_light_msg);
ros::Publisher odom_lighter_pub("odom_lighter", &odom_lighter_msg);*/

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//nav_msgs::Odometry odom_msg;
//ros::Publisher odom_pub("odom", &odom_msg);
//ros::Publisher asserv_pub("asserv", &asserv_msg);

float get_orientation_float(int32_t encoder1, int32_t encoder2);
constexpr float ticksToMillimeters(int32_t ticks);
constexpr int32_t millimetersToTicks(float millimeters);


void motors_cmd_cb(const krabi_msgs::motors_cmd &motors_cmd_msg);
void cmd_vel_cb(const geometry_msgs::Twist& twist);
void parameters_cb(const krabi_msgs::motors_parameters& parameters);
void enable_motor_cb(const std_msgs::Bool& enable);

class MotorBoard
{
public:
	MotorBoard(TIM_HandleTypeDef* motorTimHandler, UART_HandleTypeDef * huart2);
	MotorBoard();
	~MotorBoard();

	//static ros::NodeHandle& getNodeHandle(void);
	static DCMotor& getDCMotor(void);
	static void set_odom(float a_x, float a_y, float a_theta);
	//void set_odom_cb(const krabi_msgs::SetOdomRequest &req, krabi_msgs::SetOdomResponse &res);

	void update();
	void update_inputs();
private:
	//static ros::NodeHandle nh;
	static DCMotorHardware motorsHardware;
	static DCMotor motors;
	static MCP3002 currentReader;
	volatile long last_encoder_left = 0;
	volatile long last_encoder_right = 0;
	volatile int32_t last_encoder_left_angular = 0;
	volatile int32_t last_encoder_right_angular = 0;
	float compute_linear_dist(const long encoder_left, const long encoder_right);
	static float X;
	static float Y;
	static float theta_offset;
	volatile static long long message_counter;
	UART_HandleTypeDef * huart2;
	void resetUart();
};

void doResetUart(UART_HandleTypeDef * huart2);
void toggleLed();

#ifdef __cplusplus
 extern "C" {
#endif


void setup();
void loop(TIM_HandleTypeDef* motorTimHandler, TIM_HandleTypeDef* loopTimHandler, UART_HandleTypeDef * huart2);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
