/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <constants.h>

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", cmd_vel_cb);
ros::Subscriber<std_msgs::Bool> enable_sub("enable_motor", enable_motor_cb);

void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
	MotorBoard::getDCMotor().set_speed_order(twist.linear.x, -twist.angular.z);
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
		MotorBoard::getDCMotor().update();
	}
}

ros::NodeHandle MotorBoard::nh;
DCMotorHardware MotorBoard::motorsHardware;
DCMotor MotorBoard::motors;
MCP3002 MotorBoard::currentReader;

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler) {
	motorsHardware = DCMotorHardware(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_5, TIM1, TIM2, a_motorTimHandler, TIM_CHANNEL_4, a_motorTimHandler, TIM_CHANNEL_1);
	currentReader = MCP3002(GPIOC, GPIO_PIN_7, GPIOB, GPIO_PIN_6, GPIOA, GPIO_PIN_9, GPIOA, GPIO_PIN_7);
	motors = DCMotor(&motorsHardware, &currentReader);
	nh.initNode();
	//nh.advertise(odom_pub);
	nh.advertise(odom_light_pub);
	nh.advertise(chatter);
	nh.advertise(encoders_pub);
	nh.advertise(motors_pub);
	nh.subscribe(twist_sub);
	nh.subscribe(enable_sub);
}
MotorBoard::MotorBoard() {}
MotorBoard::~MotorBoard() {}

ros::NodeHandle& MotorBoard::getNodeHandle(void) {
	return nh;
}

DCMotor& MotorBoard::getDCMotor(void) {
	return motors;
}

/*
    Return the Robot's orientation, in degrees, with respect to the last encoder reset.
*/
float get_orientation_float(long encoder1, long encoder2)
{
    int absolute_orientation = fmod((encoder2 - encoder1) / TICKS_PER_DEG, 360);

    if (absolute_orientation >= 0)
        return (absolute_orientation);
    else
        return (360.f + absolute_orientation); // reminder: abs_ori is < 0 here
}

int fixOverflow(long after, long before)
{
    if (after - before > TICKS_half_OVERFLOW)
    {
        // printf("before (%ld) - after (%ld) > TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // before, after, TICKS_half_OVERFLOW, after - before - TICKS_OVERFLOW);
        return after - before - TICKS_OVERFLOW;
    }
    if (after - before < -TICKS_half_OVERFLOW)
    {
        // printf("after (%ld) - before (%ld) < -TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // after, before, -TICKS_half_OVERFLOW, after - before + TICKS_OVERFLOW);
        return after - before + TICKS_OVERFLOW;
    }
    return after - before;
}

float ticksToMillimeters(int32_t ticks)
{
	return (DIST_PER_REVOLUTION * (float)ticks / TICKS_PER_REVOLUTION);
}

/*
        Given current value of both encoders
        return the linear dist by approximating it as the average of both wheels' linear distances.
        Static variables are used to keep last value of encoders.
*/
float MotorBoard::compute_linear_dist(const long encoder1, const long encoder2)
{
    float dist1, dist2, dist;
    int diff_encoder1, diff_encoder2;

    // Compute difference in nb of ticks between last measurements and now
    diff_encoder1 = fixOverflow(encoder1, last_encoder_left);
    diff_encoder2 = fixOverflow(encoder2, last_encoder_right);

    // Compute each wheel's dist and approximate linear dist as their average
    dist1 = ticksToMillimeters(diff_encoder1);
    dist2 = ticksToMillimeters(diff_encoder2);
    dist = (dist1 + dist2) / 2.0f;

    if (fabsf(dist) > 500.)
    {
        //printf("\n/!\\ HIGH SPEED DETECTED: %.2f /!\\\n\n", dist);
        // exit(4);
    }

    // Update static variables' values (current encoder values become old ones)
    last_encoder_left = encoder1;
    last_encoder_right = encoder2;

    // Return the computed linear dist
    return dist / 1000.f; // convert to meters
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

	int32_t encoder_left = motors.get_encoder_ticks(M_L);
	int32_t encoder_right = motors.get_encoder_ticks(M_R);
	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

	float linear_dist = compute_linear_dist(encoder_left, encoder_right);
	float current_theta = get_orientation_float(encoder_left, encoder_right);

	float current_theta_rad = current_theta * M_PI / 180.f;

	X += linear_dist * cos(current_theta_rad);
	Y += linear_dist * sin(current_theta_rad);

	odom_light_msg.header.frame_id = "odom_light";
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

	odom_light_pub.publish(&odom_light_msg);

	encoders_msg.encoder_left = encoder_left;//get_speed(M_L);
	encoders_msg.encoder_right = encoder_right;//get_speed(M_R);
//	encoders_msg.encoder_left = motors.get_speed(M_L);
//	encoders_msg.encoder_right = motors.get_speed(M_R);
	encoders_pub.publish(&encoders_msg);

	motors_msg.encoders = encoders_msg;
	motors_msg.current_left = motors.get_current(0);
	motors_msg.current_right = motors.get_current(1);

	motors_msg.current_left_accumulated = motors.get_accumulated_current(0);
	motors_msg.current_right_accumulated = motors.get_accumulated_current(1);

	motors_pub.publish(&motors_msg);

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
