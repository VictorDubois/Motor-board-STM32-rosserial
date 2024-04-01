/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <constants.h>
extern "C" {
	#include "main.h"
}
#include <string.h>
#include <string>
#include "math.h"

#define TX_LINE_BUFFER_SIZE 100
#define RX_LINE_BUFFER_SIZE 100
#define RX_BUFFER_SIZE 1
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_line_buffer[RX_LINE_BUFFER_SIZE];
uint8_t tx_e_line_buffer[TX_LINE_BUFFER_SIZE];
uint8_t tx_o_line_buffer[TX_LINE_BUFFER_SIZE];
int offset_message_already_received = 0;
int nb_messages_received = 0;
float test_message_received = 0;
unsigned int nb_updates_without_message = 0;
//#include <tf/tf.h>

/*ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", cmd_vel_cb);
ros::Subscriber<krabi_msgs::motors_parameters> parameters_sub("motors_parameters", parameters_cb);
ros::Subscriber<std_msgs::Bool> enable_sub("enable_motor", enable_motor_cb);
ros::Subscriber<krabi_msgs::motors_cmd> motors_cmd_sub("motors_cmd", motors_cmd_cb);*/

float get_float(uint8_t* a_string, int a_beginning)
{
	char hexValue[8];
	strncpy(hexValue, (char*)a_string + a_beginning, 8);
    uint32_t floatHex = strtoul(hexValue, NULL, 16); // Convert hexadecimal string to unsigned long
    float floatValue;
    memcpy(&floatValue, &floatHex, sizeof(floatValue)); // Convert unsigned long to float
    return floatValue;
}

void motors_cmd_hex_cb(uint8_t* a_message)
{
	krabi_msgs::motors_cmd motors_cmd_msg;
	int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;

    motors_cmd_msg.enable_motors = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;
    motors_cmd_msg.override_PWM = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;
    motors_cmd_msg.PWM_override_left = get_float(a_message, offset + (i++)*float_msg_size);
    motors_cmd_msg.PWM_override_right = get_float(a_message, offset + (i++)*float_msg_size);
    motors_cmd_msg.reset_encoders = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;

    motors_cmd_cb(motors_cmd_msg);
}

void parameters_hex_cb(uint8_t* a_message)
{
	krabi_msgs::motors_parameters motors_parameters_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    motors_parameters_msg.max_current_left = get_float(a_message, offset + (i++)*float_msg_size);
    motors_parameters_msg.max_current_right = get_float(a_message, offset + (i++)*float_msg_size);
    //test_message_received = get_float(a_message, offset + (i++)*float_msg_size);
    //motors_parameters_msg.max_current = test_message_received;
    motors_parameters_msg.max_current = get_float(a_message, offset + (i++)*float_msg_size);

    parameters_cb(motors_parameters_msg);
}

void cmd_vel_hex_cb(uint8_t* a_message)
{
	geometry_msgs::Twist twist_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    twist_msg.linear.x = get_float(a_message, offset + (i++)*float_msg_size);
    twist_msg.angular.z = get_float(a_message, offset + (i++)*float_msg_size);

    cmd_vel_cb(twist_msg);
}

void enable_motor_hex_cb(uint8_t* a_message)
{
	std_msgs::Bool enable_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    enable_msg.data = get_float(a_message, offset + (i++)*float_msg_size) > 0.5;

    enable_motor_cb(enable_msg);
}

void read_serial()
{
	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	nb_updates_without_message = 0;


	if(rx_line_buffer[0]=='e')
	{
		nb_messages_received++;
		enable_motor_hex_cb(rx_line_buffer);
	}
	if(rx_line_buffer[0]=='v')
	{
		nb_messages_received++;
		cmd_vel_hex_cb(rx_line_buffer);
	}
	if(rx_line_buffer[0]=='p')
	{
		nb_messages_received++;
		parameters_hex_cb(rx_line_buffer);
	}
	if(rx_line_buffer[0]=='c')
	{
		nb_messages_received++;
		motors_cmd_hex_cb(rx_line_buffer);
	}
}

void motors_cmd_cb(const krabi_msgs::motors_cmd &motors_cmd_msg)
{
	if (motors_cmd_msg.reset_encoders)
	{
		MotorBoard::set_odom(0, 0, 0);
	}

	MotorBoard::getDCMotor().set_enable_motors(motors_cmd_msg.enable_motors);

	if (!motors_cmd_msg.enable_motors) {
		MotorBoard::getDCMotor().resetMotors();
		return;
	}

	if (motors_cmd_msg.override_PWM)
	{
		MotorBoard::getDCMotor().override_PWM(motors_cmd_msg.PWM_override_left, motors_cmd_msg.PWM_override_right);
	}
	else
	{
		MotorBoard::getDCMotor().stop_pwm_override();
	}
}

/*void set_odom_alone_cb(const krabi_msgs::SetOdomRequest &req, krabi_msgs::SetOdomResponse &res)
{
	MotorBoard::set_odom(req.x, req.y, req.theta);
	res.success = true;
}

ros::ServiceServer<krabi_msgs::SetOdomRequest, krabi_msgs::SetOdomResponse> set_odom_srv("set_odom", set_odom_alone_cb);*/

void parameters_cb(const krabi_msgs::motors_parameters& a_parameters)
{
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current);
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current_left, a_parameters.max_current_right);
}

void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
	MotorBoard::getDCMotor().set_speed_order(twist.linear.x, -twist.angular.z);
	//asserv_msg.max_current_left = twist.linear.x;

}

void enable_motor_cb(const std_msgs::Bool& enable)
{
	if(!enable.data) {
		MotorBoard::getDCMotor().resetMotors();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//MotorBoard::getNodeHandle().getHardware()->flush();
	//HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
}



void receiveUART(UART_HandleTypeDef *huart){
	//MotorBoard::getNodeHandle().getHardware()->reset_rbuf();
	if (huart->Instance == USART2) {
		test_message_received++;

		int dma_buffer_offset = 0;

		int i = 0;
		while (i < RX_BUFFER_SIZE)
		{
			rx_line_buffer[offset_message_already_received + i] = rx_buffer[i];

			if (rx_buffer[i] == '\n' || rx_buffer[i] == '\r')
			{
				break;
			}
			else
			{
				i++;
			}
		}

		if (rx_buffer[i] == '\n' || rx_buffer[i] == '\r' ||
				(offset_message_already_received + i > 1 && rx_line_buffer[offset_message_already_received + i-1] == '\\' && (rx_line_buffer[offset_message_already_received + i] == 'n' || rx_line_buffer[offset_message_already_received + i] == 'r' )))
		{
			// Message received !
			read_serial();
			offset_message_already_received = 0;
		}
		else
		{
			offset_message_already_received += i;

			dma_buffer_offset += i;
			// Ensure the buffer does not overflow
			if (offset_message_already_received+RX_BUFFER_SIZE >= sizeof(rx_line_buffer)) {
				// Buffer overflow, handle error or reset the buffer
				offset_message_already_received = 0;
			}
		}



		//MotorBoard::getDCMotor().receiveSerial(huart->Instance->RDR);
		/*rx_buffer[rx_index++] = huart->Instance->RDR; // Read received byte and store in buffer
		if (rx_buffer[rx_index - 1] == "\n") {
			// Process the received line
			// Example: Print received line
			HAL_UART_Transmit(&huart, rx_buffer, rx_index, HAL_MAX_DELAY);
			rx_index = 0; // Reset buffer index
		}*/
		//HAL_UART_Receive_IT(&huart, &rx_buffer[rx_index], 1); // Enable UART receive interrupt again
		//HAL_UART_Receive_DMA(huart, rx_buffer, RX_BUFFER_SIZE); // Start a new DMA reception
		//HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, 1); // Start a new DMA reception
		//HAL_UART_Receive_IT(huart, rx_buffer, 1); // Enable UART receive interrupt again

		//HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED



	}
	//HAL_UART_Receive_DMA(huart, rx_buffer, 10); //works
}

void float_to_hex(float a_value, uint8_t* a_out, int start_pos)
{
    /*//memcpy(a_out + start_pos, &a_value, sizeof(a_value));
	uint8_t before_hex[4];
	char after_hex[8];
    memcpy(&before_hex, &a_value, sizeof(a_value));
    for (int i = 0; i < 4; i++)
    {
    	auto nb_char_written = sprintf(after_hex + i*2, "%02X", before_hex + i);
    }
    memcpy(a_out + start_pos, after_hex, 8);*/

    uint32_t floatHex;
    memcpy(&floatHex, &a_value, sizeof(a_value));
	sprintf((char*)(a_out + start_pos), "%08X", floatHex); // Convert to hexadecimal string
}

void publish_encoders(UART_HandleTypeDef * huart2)
{
	//uint8_t out_msg[TX_BUFFER_SIZE_toto];
	int start_pos = 0;
	const int step = 8;

	tx_e_line_buffer[start_pos] = 'e';
	start_pos += 1;
	tx_e_line_buffer[start_pos] = ':';
	start_pos += 1;
	float_to_hex(encoders_msg.encoder_right, tx_e_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(encoders_msg.encoder_left, tx_e_line_buffer, start_pos);
	start_pos += step;

	tx_e_line_buffer[start_pos] = '\n';
	start_pos += 2;

	HAL_UART_Transmit_DMA(huart2, tx_e_line_buffer, start_pos);
	//HAL_UART_Transmit_IT(huart2, out_msg, start_pos);
}

void publish_odom_lighter(UART_HandleTypeDef * huart2)
{
	int start_pos = 0;
	const int step = 8;

	// @TODO: output what was received

	tx_o_line_buffer[start_pos] = 'o';
	start_pos += 1;
	tx_o_line_buffer[start_pos] = ':';
	start_pos += 1;
	float_to_hex(odom_lighter_msg.poseX, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.poseY, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.angleRz, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.speedVx, tx_o_line_buffer, start_pos);
	start_pos += step;
	float_to_hex(odom_lighter_msg.speedWz, tx_o_line_buffer, start_pos);
	start_pos += step;
	tx_o_line_buffer[start_pos] = '\n';
	start_pos += 1;

	HAL_UART_Transmit_DMA(huart2, tx_o_line_buffer, start_pos);
	//HAL_UART_Transmit_DMA(huart2, (uint8_t*)"\n", 1);
	//HAL_UART_Transmit_IT(huart2, out_msg, start_pos);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart){
	receiveUART(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	receiveUART(huart);
}

//void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
//{
//	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM15) {
		MotorBoard::getDCMotor().update();

	}
	if (htim->Instance == TIM7) {
	}
}



//ros::NodeHandle MotorBoard::nh;
DCMotorHardware MotorBoard::motorsHardware;
DCMotor MotorBoard::motors;
MCP3002 MotorBoard::currentReader;
volatile long long MotorBoard::message_counter = 0;

float MotorBoard::X = 0;
float MotorBoard::Y = 0;
float MotorBoard::theta_offset = 0;

void MotorBoard::set_odom(float a_x, float a_y, float a_theta)
{
	X = a_x;
	Y = a_y;
	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	float current_theta = get_orientation_float(encoder_left, encoder_right);
	theta_offset = a_theta - current_theta;
}

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler//){
	, UART_HandleTypeDef * huart2) :
		huart2(huart2){

	while(false)
	{
		HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, static_cast<GPIO_PinState>(bool(int(HAL_GetTick()/1000)%2))); // Turn On/OFF LED
		HAL_Delay(100);
	}

	HAL_Delay(1);//3000, 4000, 5000 works
	//1, 500, 2000 does not work

	motorsHardware = DCMotorHardware(TIM1, TIM2, a_motorTimHandler, TIM_CHANNEL_4, a_motorTimHandler, TIM_CHANNEL_1);
	currentReader = MCP3002();
	motors = DCMotor(&motorsHardware, &currentReader);

	motors.set_max_acceleration(millimetersToTicks(9810));//mm/s/s
	motors.set_max_speed(millimetersToTicks(2000));//mm/s (=1.9rad/s)

	set_odom(0, 0, 0);

	//nh.initNode();
	HAL_Delay(100);

	//nh.advertise(odom_pub);
	/*nh.advertise(asserv_pub);
	nh.advertise(odom_light_pub);
	nh.advertise(odom_lighter_pub);
	nh.advertise(encoders_pub);
	nh.advertise(motors_pub);
	nh.subscribe(twist_sub);
	nh.subscribe(parameters_sub);
	nh.subscribe(enable_sub);
	nh.subscribe(motors_cmd_sub);

	nh.advertiseService(set_odom_srv);*/

	//HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, RX_BUFFER_SIZE); // Start a new DMA reception
	HAL_Delay(100);
	//HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	uint8_t reinit = 1;
	/*while (!nh.connected())
	{
	    nh.spinOnce();
		if (HAL_GetTick()/1000 > reinit){
			MotorBoard::getNodeHandle().getHardware()->reset_rbuf();
			reinit++;
		}
	    HAL_Delay(MS_BETWEEN_UPDATES);
	}*/


}
MotorBoard::MotorBoard() {}
MotorBoard::~MotorBoard() {}

/*ros::NodeHandle& MotorBoard::getNodeHandle(void) {
	return nh;
}*/

DCMotor& MotorBoard::getDCMotor(void) {
	return motors;
}

/*
    Return the Robot's orientation, in degrees, with respect to the last encoder reset.
*/
float get_orientation_float(int32_t encoder1, int32_t encoder2)
{


    float absolute_orientation = fmod((encoder2 - encoder1) / TICKS_PER_DEG, 360);

    if (absolute_orientation >= 0)
        return (absolute_orientation);
    else
        return (360.f + absolute_orientation); // reminder: abs_ori is < 0 here
}

int32_t fixOverflowAngular(int16_t a_after, int32_t before)
{
	int32_t after = a_after;
    while (after - before > TICKS_half_OVERFLOW)
    {
        // printf("before (%ld) - after (%ld) > TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // before, after, TICKS_half_OVERFLOW, after - before - TICKS_OVERFLOW);
        after -= TICKS_OVERFLOW;
    }
    while (after - before < -TICKS_half_OVERFLOW)
    {
        // printf("after (%ld) - before (%ld) < -TICKS_half_OVERFLOW (%d). Returning %ld\n\n\n",
        // after, before, -TICKS_half_OVERFLOW, after - before + TICKS_OVERFLOW);
        after += TICKS_OVERFLOW;
    }
    return after;
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

constexpr float ticksToMillimeters(int32_t ticks)
{
	return (DIST_PER_REVOLUTION * (float)ticks / TICKS_PER_REVOLUTION);
}


constexpr int32_t millimetersToTicks(float millimeters)
{
	return static_cast<int32_t>(millimeters * TICKS_PER_REVOLUTION/DIST_PER_REVOLUTION);
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

void MotorBoard::update_inputs() {
	// @todo
	//nh.spinOnce();
}

void MotorBoard::update() {
	nb_updates_without_message++;

	if (nb_updates_without_message>10)
	{
		// 1. Disable UART and DMA
		HAL_UART_DMAStop(huart2); // Stop DMA associated with UART2
		HAL_UART_DeInit(huart2); // Deinitialize UART2

		// 2. Reset DMA Configuration
		//HAL_DMA_DeInit(&hdma_usart2_rx); // Deinitialize DMA channel used for UART2 reception

		// 3. Reset UART Configuration
		huart2->Instance = USART2;
		huart2->Init.BaudRate = 115200;
		huart2->Init.WordLength = UART_WORDLENGTH_8B;
		huart2->Init.StopBits = UART_STOPBITS_1;
		huart2->Init.Parity = UART_PARITY_NONE;
		huart2->Init.Mode = UART_MODE_TX_RX;
		huart2->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart2->Init.OverSampling = UART_OVERSAMPLING_16;
		huart2->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart2->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		while (HAL_UART_Init(huart2) != HAL_OK)
		{
			toggleLed();
			HAL_Delay(100);
		}
		//MX_USART2_UART_Init(); // Reinitialize UART2 with default settings. This function should be generated by CubeMX or manually defined in your code.
		while (HAL_UART_Receive_DMA(huart2, rx_buffer, 1) != HAL_OK)
		{
			toggleLed();
			HAL_Delay(300);
		}
	}

	/*if (!nh.connected()){
		return;
	}*/

	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	encoders_msg.encoder_left = encoder_left;
	encoders_msg.encoder_right = encoder_right;
	//encoders_msg.header.stamp = nh.now();

	//encoders_pub.publish(&encoders_msg);//@todo
	//publish_encoders(huart2);

	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

	/*if (left_speed > 0){
		HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	}
	else
	{
		HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET); // Turn Off LED

	}*/


	float linear_dist = compute_linear_dist(encoder_left, encoder_right);

    // Compute difference in nb of ticks between last measurements and now
    int32_t fixed_angular_encoder_left = fixOverflowAngular(encoder_left, last_encoder_left_angular);
    int32_t fixed_angular_encoder_right = fixOverflowAngular(encoder_right, last_encoder_right_angular);
	last_encoder_left_angular = fixed_angular_encoder_left;
	float current_theta = get_orientation_float(fixed_angular_encoder_left, fixed_angular_encoder_right);
	last_encoder_right_angular = fixed_angular_encoder_right;
	current_theta += theta_offset;

	float current_theta_rad = current_theta * M_PI / 180.f;

	X += linear_dist * cos(current_theta_rad);
	Y += linear_dist * sin(current_theta_rad);


	//int32_t right_speed = motors.get_speed(M_R);
	//int32_t left_speed = motors.get_speed(M_L);

	/*odom_msg.header.frame_id = "odom";
	odom_msg.header.stamp = nh.now();
	odom_msg.header.seq = message_counter++;
	odom_msg.child_frame_id = "base_link";

	for (unsigned int i = 0; i < (sizeof(odom_msg.pose.covariance)/sizeof(*(odom_msg.pose.covariance))); i++){
		odom_msg.pose.covariance[i] = 0;
	}

	odom_msg.pose.pose.position.x = X;
	odom_msg.pose.pose.position.y = Y;
	odom_msg.pose.pose.position.z = 0;

	odom_msg.pose.pose.orientation = odom_quat;

	//odom_msg.pose.pose.orientation.x = 0;
	//odom_msg.pose.pose.orientation.y = 0;
	//odom_msg.pose.pose.orientation.z = current_theta_rad;

	for (unsigned int i = 0; i < (sizeof(odom_msg.twist.covariance)/sizeof(*(odom_msg.twist.covariance))); i++){
		odom_msg.twist.covariance[i] = 0;
	}

	odom_msg.twist.twist.linear.x = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.linear.z = 0;

	odom_msg.twist.twist.angular.x = 0;
	odom_msg.twist.twist.angular.y = 0;


	odom_msg.twist.twist.angular.z = odometry->getAngularSpeed();
	odom_pub.publish(&odom_msg);*/

	/*odom_lighter_msg.header.stamp = nh.now();
	odom_lighter_msg.header.seq = message_counter++;
	odom_lighter_msg.header.frame_id = "";*/
	odom_lighter_msg.poseX = MotorBoard::getDCMotor().get_linear_speed_order();//X;
	odom_lighter_msg.poseY = 4.2f;//Y;
	odom_lighter_msg.angleRz = nb_messages_received;//current_theta_rad;
	odom_lighter_msg.speedVx = (float)test_message_received;//ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_lighter_msg.speedWz = encoder_left;//((right_speed - left_speed)/TICKS_PER_DEG)*M_PI/180; // rad/s
	publish_odom_lighter(huart2);
	//odom_lighter_pub.publish(&odom_lighter_msg);//@todo

	if (false && message_counter%100 == 0)
	{
		/*
		//odom_light_msg.header.stamp = nh.now();
		odom_light_msg.header.seq = message_counter++;
		odom_light_msg.header.frame_id = "odom_light_newer";
		odom_light_msg.pose.position.x = X;
		odom_light_msg.pose.position.y = Y;
		odom_light_msg.pose.position.z = MotorBoard::getDCMotor().get_dt();

		odom_light_msg.pose.orientation = tf::createQuaternionFromYaw(current_theta_rad);

		odom_light_msg.speed.linear.x = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
		odom_light_msg.speed.linear.y = ticksToMillimeters(MotorBoard::getDCMotor().get_linear_speed_order())/1000.f;
		odom_light_msg.speed.linear.z = MotorBoard::getDCMotor().get_linear_error_integ()/1000.f;

		odom_light_msg.speed.angular.x = ticksToMillimeters(MotorBoard::getDCMotor().get_voltage(M_L));
		odom_light_msg.speed.angular.y = ticksToMillimeters(MotorBoard::getDCMotor().get_voltage(M_R));
		odom_light_msg.speed.angular.z = MotorBoard::getDCMotor().get_linear_error()/1000.f;//ticksToMillimeters((left_speed-right_speed)/2)/1000.f; // C'est complètement faux, non ?
		odom_light_msg.current_motor_left = motors.get_accumulated_current(M_L);
		odom_light_msg.current_motor_right = motors.get_accumulated_current(M_R);


		odom_light_pub.publish(&odom_light_msg);*/

		motors_msg.current_left = motors.get_current(M_L);
		motors_msg.current_right = motors.get_current(M_R);

		motors_msg.current_left_accumulated = motors.get_accumulated_current(M_L);
		motors_msg.current_right_accumulated = motors.get_accumulated_current(M_R);

		//motors_pub.publish(&motors_msg);


	}

	//nh.spinOnce();
}

void setup()
{
  //pinMode(BRAKE, OUTPUT);
  //digitalWrite(BRAKE, LOW);

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);//LED
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//BRAKE
  HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);//DIR_A
  HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET);//DIR_B
}

void toggleLed()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED on GPIOA Pin 5
}

void loop(TIM_HandleTypeDef* a_motorTimHandler, TIM_HandleTypeDef* a_loopTimHandler, UART_HandleTypeDef * huart2)
{
	//HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
	MotorBoard myboard = MotorBoard(a_motorTimHandler, huart2);

	__HAL_UART_CLEAR_OREFLAG(huart2);

//	while (1)
//	{
//		// Attempt to establish UART connection
//		if (HAL_UART_Transmit(huart2, (uint8_t*)"Hello PC\r\n", strlen("Hello PC\r\n"), HAL_MAX_DELAY) == HAL_OK)
//		{
//			// Connection established, break out of the loop
//			break;
//		}
//		else
//		{
//			// Error occurred, handle the error (e.g., LED blinking, logging, etc.)
//			// For simplicity, let's toggle an LED to indicate error
//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED on GPIOA Pin 5
//
//			// Wait for a short duration before attempting again
//			HAL_Delay(100); // Adjust this delay according to your application's needs
//		}
//	}
	HAL_TIM_Base_Start_IT(a_loopTimHandler);
	int32_t waiting_time = 100;
	//HAL_UARTEx_ReceiveToIdle_DMA(huart2, rx_buffer, RX_BUFFER_SIZE);


	while (HAL_UART_Receive_DMA(huart2, rx_buffer, 1) != HAL_OK)
	{
		toggleLed();
		HAL_Delay(300);
	}

	//HAL_UART_Receive_IT(huart2, rx_buffer, 1);

	//HAL_UARTEx_ReceiveToIdle_DMA(huart2, rx_buffer, 10); // Start a new DMA reception
	while(true) {
		myboard.update();

		for(int ii = 0; ii< 10 ; ii++){
			myboard.update_inputs();


			// Debug messages

			/*int32_t right_speed = MotorBoard::getDCMotor().get_speed(M_R);
			int32_t left_speed = MotorBoard::getDCMotor().get_speed(M_L);
			float current_speed = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
			asserv_msg.max_current = current_speed;
			asserv_msg.max_current_right = static_cast<float>(MotorBoard::getDCMotor().get_voltage(M_R))/500;
			asserv_msg.max_current_left = static_cast<float>(MotorBoard::getDCMotor().get_linear_speed_order())/500;


			asserv_pub.publish(&asserv_msg);*/



			HAL_Delay(waiting_time);
		}
	}
}
