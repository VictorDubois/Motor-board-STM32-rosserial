/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", cmd_vel_cb);

void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
	MotorBoard::getDCMotor().set_speed_order(twist.linear.x, twist.angular.z);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	MotorBoard::getNodeHandle().getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	MotorBoard::getNodeHandle().getHardware()->reset_rbuf();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim->Instance == TIM15) {
		MotorBoard::getDCMotor().update();
	}
}

ros::NodeHandle MotorBoard::nh;
DCMotorHardware MotorBoard::motorsHardware;
DCMotor MotorBoard::motors;

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler) {
	motorsHardware = DCMotorHardware(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, TIM1, TIM2, a_motorTimHandler, TIM_CHANNEL_4, a_motorTimHandler, TIM_CHANNEL_1);
	motors = DCMotor(&motorsHardware);
	nh.initNode();
	//nh.advertise(odom_pub);
	nh.advertise(chatter);
	nh.advertise(encoders_pub);
	nh.subscribe(twist_sub);
}
MotorBoard::MotorBoard() {}
MotorBoard::~MotorBoard() {}

ros::NodeHandle& MotorBoard::getNodeHandle(void) {
	return nh;
}

DCMotor& MotorBoard::getDCMotor(void) {
	return motors;
}

void MotorBoard::update() {
	//int32_t right_speed = motors.get_speed(M_R);
	//int32_t left_speed = motors.get_speed(M_L);

	/*odom_msg.header.frame_id = "toto";
	odom_msg.header.stamp.nsec = 0;
	odom_msg.header.stamp.sec = 0;
	odom_msg.header.seq = 0;
	odom_msg.child_frame_id = "toto";

	for (unsigned int i = 0; i < (sizeof(odom_msg.pose.covariance)/sizeof(*(odom_msg.pose.covariance))); i++){
		odom_msg.pose.covariance[i] = 0;
	}

	odom_msg.pose.pose.position.x = 0;
	odom_msg.pose.pose.position.y = 0;
	odom_msg.pose.pose.position.z = 0;

	odom_msg.pose.pose.orientation.x = 0;
	odom_msg.pose.pose.orientation.y = 0;
	odom_msg.pose.pose.orientation.z = 0;

	for (unsigned int i = 0; i < (sizeof(odom_msg.twist.covariance)/sizeof(*(odom_msg.twist.covariance))); i++){
		odom_msg.twist.covariance[i] = 0;
	}

	odom_msg.twist.twist.linear.x = 0;//(left_speed+right_speed)/2;
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.linear.z = 0;

	odom_msg.twist.twist.angular.x = 0;
	odom_msg.twist.twist.angular.y = 0;
	odom_msg.twist.twist.angular.z = 0;//(left_speed-right_speed)/2;
	odom_pub.publish(&odom_msg);*/

//	encoders_msg.encoder_left = motors.get_encoder_ticks(M_L);//get_speed(M_L);
//	encoders_msg.encoder_right = motors.get_encoder_ticks(M_R);//get_speed(M_R);
	encoders_msg.encoder_left = motors.get_speed(M_L);
	encoders_msg.encoder_right = motors.get_speed(M_R);
	encoders_pub.publish(&encoders_msg);

	str_msg.data = "Hello world!";
	chatter.publish(&str_msg);
	nh.spinOnce();
}

void setup()
{
  //pinMode(BRAKE, OUTPUT);
  //digitalWrite(BRAKE, LOW);

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);//LED
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//BRAKE
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);//DIR_A
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);//DIR_B
}

void loop(TIM_HandleTypeDef* a_motorTimHandler, TIM_HandleTypeDef* a_loopTimHandler)
{
	MotorBoard myboard = MotorBoard(a_motorTimHandler);
	HAL_TIM_Base_Start_IT(a_loopTimHandler);
	while(true) {
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		myboard.update();

		HAL_Delay(100);
	}
}
