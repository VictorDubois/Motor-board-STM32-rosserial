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
unsigned int offset_message_already_received = 0;
int nb_messages_received = 0;
float test_message_received = 0;
unsigned int nb_updates_without_message = 0;


float get_float(const uint8_t* a_string, const int a_beginning)
{
	char hexValue[8];
	strncpy(hexValue, (char*)a_string + a_beginning, 8);
    uint32_t floatHex = strtoul(hexValue, NULL, 16); // Convert hexadecimal string to unsigned long
    float floatValue;
    memcpy(&floatValue, &floatHex, sizeof(floatValue)); // Convert unsigned long to float
    return floatValue;
}

void motors_cmd_hex_cb(const uint8_t* a_message)
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

void parameters_hex_cb(const uint8_t* a_message)
{
	krabi_msgs::motors_parameters motors_parameters_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    motors_parameters_msg.max_current_left = get_float(a_message, offset + (i++)*float_msg_size);
    motors_parameters_msg.max_current_right = get_float(a_message, offset + (i++)*float_msg_size);
    motors_parameters_msg.max_current = get_float(a_message, offset + (i++)*float_msg_size);

    parameters_cb(motors_parameters_msg);
}

void cmd_vel_hex_cb(const uint8_t* a_message)
{
	geometry_msgs::Twist twist_msg;
    int i = 0;
    const int offset = 1;
    const int float_msg_size = 8;
    twist_msg.linear.x = get_float(a_message, offset + (i++)*float_msg_size);
    twist_msg.angular.z = get_float(a_message, offset + (i++)*float_msg_size);

    cmd_vel_cb(twist_msg);
}

void enable_motor_hex_cb(const uint8_t* a_message)
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
	//HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET); // Turn On LED
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

void parameters_cb(const krabi_msgs::motors_parameters& a_parameters)
{
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current);
	MotorBoard::getDCMotor().set_max_current(a_parameters.max_current_left, a_parameters.max_current_right);
}

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

void receiveUART(UART_HandleTypeDef *huart){
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
	}
}

void float_to_hex(const float a_value, uint8_t* a_out, const int start_pos)
{
    uint32_t floatHex;
    memcpy(&floatHex, &a_value, sizeof(a_value));
	sprintf((char*)(a_out + start_pos), "%08X", floatHex); // Convert to hexadecimal string
}

void publish_encoders(UART_HandleTypeDef * huart2)
{
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
}

void publish_odom_lighter(UART_HandleTypeDef * huart2)
{
	int start_pos = 0;
	const int step = 8;

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
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart){
	receiveUART(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	receiveUART(huart);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM15) {
		MotorBoard::getDCMotor().update();

	}
	if (htim->Instance == TIM7) {
	}
}

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

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler, UART_HandleTypeDef * huart2) :
		huart2(huart2)
{

	while(false)
	{
		HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, static_cast<GPIO_PinState>(bool(int(HAL_GetTick()/1000)%2))); // Turn On/OFF LED
		HAL_Delay(100);
	}

	HAL_Delay(1);

	motorsHardware = DCMotorHardware(TIM1, TIM2, a_motorTimHandler, TIM_CHANNEL_4, a_motorTimHandler, TIM_CHANNEL_1);
	currentReader = MCP3002();
	motors = DCMotor(&motorsHardware, &currentReader);

	motors.set_max_acceleration(millimetersToTicks(9810));//mm/s/s
	motors.set_max_speed(millimetersToTicks(2000));//mm/s (=1.9rad/s)

	set_odom(0, 0, 0);

	HAL_Delay(100);
}
MotorBoard::MotorBoard() {}
MotorBoard::~MotorBoard() {}

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

    // Update static variables' values (current encoder values become old ones)
    last_encoder_left = encoder1;
    last_encoder_right = encoder2;

    // Return the computed linear dist
    return dist / 1000.f; // convert to meters
}

void MotorBoard::update_inputs() {

}

void doResetUart(UART_HandleTypeDef * huart2)
{
	// 1. Disable UART and DMA
	HAL_UART_DMAStop(huart2); // Stop DMA associated with UART2
	HAL_UART_DeInit(huart2); // Deinitialize UART2

	// 2. Reset UART Configuration
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
	while (HAL_UART_Receive_DMA(huart2, rx_buffer, 1) != HAL_OK)
	{
		toggleLed();
		HAL_Delay(300);
	}
}

void MotorBoard::resetUart()
{
	doResetUart(huart2);
}

void MotorBoard::update() {
	nb_updates_without_message++;

	// Check if the heart beat is OK
	if (nb_updates_without_message>100)
	{
		this->resetUart();
	}

	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	encoders_msg.encoder_left = encoder_left;
	encoders_msg.encoder_right = encoder_right;
	//publish_encoders(huart2); // Currently, it is only possible to transmit one message

	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

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

	// Debug communication
	/*odom_lighter_msg.poseX = MotorBoard::getDCMotor().get_linear_speed_order();//X;
	odom_lighter_msg.poseY = 4.2f;//Y;
	odom_lighter_msg.angleRz = nb_messages_received;//current_theta_rad;
	odom_lighter_msg.speedVx = (float)test_message_received;//ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_lighter_msg.speedWz = encoder_left;//((right_speed - left_speed)/TICKS_PER_DEG)*M_PI/180; // rad/s*/

	odom_lighter_msg.poseX = X;
	odom_lighter_msg.poseY = Y;
	odom_lighter_msg.angleRz = current_theta_rad;
	odom_lighter_msg.speedVx = ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_lighter_msg.speedWz = ((right_speed - left_speed)/TICKS_PER_DEG)*M_PI/180; // rad/s
	publish_odom_lighter(huart2);

	if (false && message_counter%100 == 0)
	{
		motors_msg.current_left = motors.get_current(M_L);
		motors_msg.current_right = motors.get_current(M_R);

		motors_msg.current_left_accumulated = motors.get_accumulated_current(M_L);
		motors_msg.current_right_accumulated = motors.get_accumulated_current(M_R);

		//motors_pub.publish(&motors_msg);
	}
}

void setup()
{
  HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);//DIR_A
  HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET);//DIR_B
}

void toggleLed()
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED on GPIOA Pin 5
}

void loop(TIM_HandleTypeDef* a_motorTimHandler, TIM_HandleTypeDef* a_loopTimHandler, UART_HandleTypeDef * huart2)
{
	MotorBoard myboard = MotorBoard(a_motorTimHandler, huart2);

	__HAL_UART_CLEAR_OREFLAG(huart2); // Not sure if actually needed

	HAL_TIM_Base_Start_IT(a_loopTimHandler);
	uint32_t waiting_time = 100;

	// Make sure that the Uart/DMA is correctly initialized, or signal it
	while (HAL_UART_Receive_DMA(huart2, rx_buffer, 1) != HAL_OK)
	{
		toggleLed();
		HAL_Delay(300);
	}

	while(true) {
		myboard.update();

		HAL_Delay(waiting_time);
	}
}
