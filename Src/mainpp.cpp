/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <constants.h>
#include <tf/tf.h>
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", cmd_vel_cb);
ros::Subscriber<krabi_msgs::motors_parameters> parameters_sub("motors_parameters", parameters_cb);
ros::Subscriber<std_msgs::Bool> enable_sub("enable_motor", enable_motor_cb);


void set_odom_cb(const krabi_msgs::SetOdomRequest &req, krabi_msgs::SetOdomResponse &res)
{
	MotorBoard::set_odom(req.x, req.y, req.theta);
}

void parameters_cb(const krabi_msgs::motors_parameters& a_parameters)
{
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current);
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current_left, a_parameters.max_current_right);
}

void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
	MotorBoard::getDCMotor().set_speed_order(twist.linear.x, -twist.angular.z);
	asserv_msg.max_current_left = twist.linear.x;

}

void enable_motor_cb(const std_msgs::Bool& enable)
{
	if(!enable.data) {
		MotorBoard::getDCMotor().resetMotors();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	MotorBoard::getNodeHandle().getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	MotorBoard::getNodeHandle().getHardware()->reset_rbuf();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim->Instance == TIM15) {
		MotorBoard::getDCMotor().update();// should be called every MotorBoard::UPDATE_RATE
	}
}

ros::NodeHandle MotorBoard::nh;
DCMotorHardware MotorBoard::motorsHardware;
DCMotor MotorBoard::motors;
MCP3002 MotorBoard::currentReader;
volatile long long MotorBoard::message_counter = 0;

void MotorBoard::set_odom(float a_x, float a_y, float a_theta)
{
	/*X = a_x;
	Y = a_y;
	int32_t encoder_left = motors.get_encoder_ticks(M_L);
	int32_t encoder_right = motors.get_encoder_ticks(M_R);

	float current_theta = get_orientation_float(encoder_left, encoder_right);
	theta_offset = a_theta - current_theta;*/
}

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler) {
	motorsHardware = DCMotorHardware(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, TIM1, TIM2, a_motorTimHandler, TIM_CHANNEL_4, a_motorTimHandler, TIM_CHANNEL_1);
	currentReader = MCP3002(GPIOC, GPIO_PIN_7, GPIOB, GPIO_PIN_6, GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_7);
	motors = DCMotor(&motorsHardware, &currentReader);
	odometry = new Odometry();

	motors.set_max_acceleration(Odometry::millimetersToTicks(5000));//mm/s/s
	motors.set_max_speed(Odometry::millimetersToTicks(500));//mm/s

	nh.initNode();
	nh.advertise(odom_pub);
	nh.advertise(asserv_pub);
	//nh.advertise(odom_light_pub);
	//nh.advertise(chatter);
	//nh.advertise(encoders_pub);
	//nh.advertise(motors_pub);
	nh.subscribe(twist_sub);
	nh.subscribe(parameters_sub);
	nh.subscribe(enable_sub);
	HAL_Delay(100);
	while (!nh.connected())
	{
	    nh.spinOnce();
	    HAL_Delay(MS_BETWEEN_UPDATES);
	}
	nh.negotiateTopics();
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
	if (!nh.connected()){
		return;
	}

	odometry->update();

	//int32_t right_speed = motors.get_speed(M_R);
	//int32_t left_speed = motors.get_speed(M_L);

	odom_msg.header.frame_id = "odom";
	odom_msg.header.stamp = nh.now();
	odom_msg.header.seq = message_counter++;
	odom_msg.child_frame_id = "base_link";

	for (unsigned int i = 0; i < (sizeof(odom_msg.pose.covariance)/sizeof(*(odom_msg.pose.covariance))); i++){
		odom_msg.pose.covariance[i] = 0;
	}

	odom_msg.pose.pose.position.x = odometry->getX();
	odom_msg.pose.pose.position.y = odometry->getY();;
	odom_msg.pose.pose.position.z = 0;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(odometry->getTheta());
	odom_msg.pose.pose.orientation = odom_quat;

	//odom_msg.pose.pose.orientation.x = 0;
	//odom_msg.pose.pose.orientation.y = 0;
	//odom_msg.pose.pose.orientation.z = current_theta_rad;

	for (unsigned int i = 0; i < (sizeof(odom_msg.twist.covariance)/sizeof(*(odom_msg.twist.covariance))); i++){
		odom_msg.twist.covariance[i] = 0;
	}

	odom_msg.twist.twist.linear.x = odometry->getLinearSpeed();
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.linear.z = 0;

	odom_msg.twist.twist.angular.x = 0;
	odom_msg.twist.twist.angular.y = 0;
	odom_msg.twist.twist.angular.z = odometry->getAngularSpeed();
	odom_pub.publish(&odom_msg);



	/*odom_light_msg.header.frame_id = "odom_light";
	odom_light_msg.pose.position.x = X;
	odom_light_msg.pose.position.y = Y;
	odom_light_msg.pose.position.z = 0;

	odom_light_msg.pose.orientation.x = 0;
	odom_light_msg.pose.orientation.y = 0;
	odom_light_msg.pose.orientation.z = current_theta_rad;

	odom_light_msg.speed.linear.x = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_light_msg.speed.linear.y = 0;
	odom_light_msg.speed.linear.z = 0;

	odom_light_msg.speed.angular.x = 0;
	odom_light_msg.speed.angular.y = 0;
	odom_light_msg.speed.angular.z = ticksToMillimeters((left_speed-right_speed)/2)/1000.f;
	odom_light_msg.current_motor_left = motors.get_accumulated_current(0);
	odom_light_msg.current_motor_right = motors.get_accumulated_current(1);

	odom_light_pub.publish(&odom_light_msg);*/

	motors_msg.current_left = motors.get_current(0);
	motors_msg.current_right = motors.get_current(1);

	motors_msg.current_left_accumulated = motors.get_accumulated_current(0);
	motors_msg.current_right_accumulated = motors.get_accumulated_current(1);

	//motors_pub.publish(&motors_msg);

	str_msg.data = "Hello world!";
	//chatter.publish(&str_msg);
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
		uint32_t before = HAL_GetTick();

		myboard.update();
		uint32_t after = HAL_GetTick();

		HAL_Delay(100 - (after - before));
		/*for(int ii = 0; ii< 10 ; ii++){
			uint32_t before = HAL_GetTick();
			int32_t right_speed = MotorBoard::getDCMotor().get_speed(M_R);
			int32_t left_speed = MotorBoard::getDCMotor().get_speed(M_L);
			float current_speed = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
			asserv_msg.max_current = current_speed;
			asserv_msg.max_current_right = static_cast<float>(MotorBoard::getDCMotor().get_error(M_R))/5000;
			//asserv_msg.max_current_right = static_cast<float>(MotorBoard::getDCMotor().get_voltage(M_R))/500;
			asserv_pub.publish(&asserv_msg);

			uint32_t after = HAL_GetTick();

			HAL_Delay(10 - (after - before));
		}*/
	}
}
