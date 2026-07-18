/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include "canManager.h"

#include <constants.h>
extern "C" {
	#include "main.h"
}

#include "math.h"

#include <uartBroker.h>

#include <cstring>

#include "callbacks.h"



uartBroker s_uart_broker;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart){
	s_uart_broker.receiveUART(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	s_uart_broker.receiveUART(huart);
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
CurrentReaderCan MotorBoard::currentReader;
volatile long long MotorBoard::message_counter = 0;
volatile long MotorBoard::last_encoder_left = 0;
volatile long MotorBoard::last_encoder_right = 0;
volatile long MotorBoard::int32_t_encoder_left = 0;
volatile long MotorBoard::int32_t_encoder_right = 0;
float MotorBoard::X = 0;
float MotorBoard::Y = 0;
float MotorBoard::theta_offset = 0;

void MotorBoard::set_odom(float a_x, float a_y, float a_theta)
{
	motors.resetEncodersCounter();
	X = a_x;
	Y = a_y;
	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	float current_theta = get_orientation_float(encoder_left, encoder_right, 0);
	theta_offset = a_theta - current_theta;

	last_encoder_left = motors.get_encoder_ticks(M_L);
	last_encoder_right = motors.get_encoder_ticks(M_R);

	int32_t_encoder_left = motors.get_encoder_ticks(M_L);
	int32_t_encoder_right = motors.get_encoder_ticks(M_R);
}

MotorBoard::MotorBoard(TIM_HandleTypeDef* a_motorTimHandler, UART_HandleTypeDef * huart2, FDCAN_HandleTypeDef* hcan, ADC_HandleTypeDef* hadc2) :
	huart2(huart2),
	hcan(hcan)
{

	while(false)
	{
		HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, static_cast<GPIO_PinState>(bool(int(HAL_GetTick()/1000)%2))); // Turn On/OFF LED
		HAL_Delay(100);
	}

	HAL_Delay(1);

	canBroker = CanBroker();

	motorsHardware = DCMotorHardware(TIM2, TIM1, a_motorTimHandler, TIM_CHANNEL_2, a_motorTimHandler, TIM_CHANNEL_1, hcan);

#ifdef USE_MCP3002
	currentReader = CurrentReaderMCP3002();
#else
	//currentReader = CurrentReaderAdc(hadc2);
	currentReader = CurrentReaderCan();
#endif
	motors = DCMotor(&motorsHardware, &currentReader);

	motors.set_max_acceleration(millimetersToTicks(9810));//mm/s/s
	motors.set_max_speed(millimetersToTicks(2000));//mm/s (=1.9rad/s) // @TODO try increasing this

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
float get_orientation_float(int32_t encoder1, int32_t encoder2, float offset)
{


    float absolute_orientation = fmod(ticksToDegrees((encoder2 - encoder1)/2) + offset, 360);

    if (absolute_orientation >= 0)
        return (absolute_orientation);
    else
        return (360.f + absolute_orientation); // reminder: abs_ori is < 0 here
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
    diff_encoder1 = diffWithFixOverflow(encoder1, last_encoder_left);
    diff_encoder2 = diffWithFixOverflow(encoder2, last_encoder_right);

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

void MotorBoard::updateCurrent()
{
	int16_t left_current_mA = motors.get_accumulated_current(M_L) / currentReader.getOneMilliAmp(); // 2 bytes
	int16_t right_current_mA = motors.get_accumulated_current(M_R) / currentReader.getOneMilliAmp(); // 2 bytes
	uint16_t left_wheel_unstalled_in_ms = motors.get_remaining_time_stopped(M_L); // 2 bytes Nb of ms until the robot's left wheel is allowed to move again
	uint16_t right_wheel_unstalled_in_ms = motors.get_remaining_time_stopped(M_R);

	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

	float speedVx = ticksToMeters(left_speed + right_speed)/2;
	float speedWz = ticksToRads(right_speed - left_speed)/2; // rad/s

	canBroker.publishCurrent(left_current_mA, right_current_mA, left_wheel_unstalled_in_ms, right_wheel_unstalled_in_ms, speedVx, speedWz);
}

void MotorBoard::update() {
	s_uart_broker.checkHeartBeat(huart2);

	int16_t encoder_left = motors.get_encoder_ticks(M_L);
	int16_t encoder_right = motors.get_encoder_ticks(M_R);

	s_uart_broker.setEncodersMsg(encoder_left, encoder_right);

	int32_t_encoder_left = fixOverflow(encoder_left, int32_t_encoder_left);
	int32_t_encoder_right = fixOverflow(encoder_right, int32_t_encoder_right);
	//publish_encoders(huart2); // Currently, it is only possible to transmit one message

	int32_t right_speed = motors.get_speed(M_R);
	int32_t left_speed = motors.get_speed(M_L);

	float linear_dist = compute_linear_dist(encoder_left, encoder_right);
	float current_theta = get_orientation_float(int32_t_encoder_left, int32_t_encoder_right, theta_offset);
	//current_theta += theta_offset;

	float current_theta_rad = current_theta * M_PI / 180.f;

	X += linear_dist * cos(current_theta_rad);
	Y += linear_dist * sin(current_theta_rad);

	// Debug communication
	/*odom_lighter_msg.poseX = MotorBoard::getDCMotor().get_linear_speed_order();//X;
	odom_lighter_msg.poseY = 4.2f;//Y;
	odom_lighter_msg.angleRz = nb_messages_received;//current_theta_rad;
	odom_lighter_msg.speedVx = (float)test_message_received;//ticksToMillimeters((left_speed+right_speed)/2)/1000.f;
	odom_lighter_msg.speedWz = encoder_left;//((right_speed - left_speed)/TICKS_PER_DEG)*M_PI/180; // rad/s*/


	float speedVx = ticksToMeters(left_speed + right_speed)/2;
	float speedWz = ticksToRads(right_speed - left_speed)/2; // rad/s

	s_uart_broker.setOdomLighterMsg(X, Y, current_theta_rad, speedVx, speedWz);

	s_uart_broker.publish_odom_lighter(huart2);

	int16_t currentLeft  = motors.get_accumulated_current(M_L);
	int16_t currentRight = motors.get_accumulated_current(M_R);

	canBroker.publishOdometry(X, Y, current_theta_rad, currentLeft, currentRight);


	if (false && message_counter%100 == 0)
	{
		s_uart_broker.setMotorsMsg(s_uart_broker.getEncodersMsg(), motors.get_current(M_L), motors.get_current(M_R), motors.get_accumulated_current(M_L), motors.get_accumulated_current(M_R));

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




void loop(TIM_HandleTypeDef* a_motorTimHandler, TIM_HandleTypeDef* a_loopTimHandler, UART_HandleTypeDef * huart2, FDCAN_HandleTypeDef* hcan, ADC_HandleTypeDef* hadc2)
{
	MotorBoard myboard = MotorBoard(a_motorTimHandler, huart2, hcan, hadc2);

	__HAL_UART_CLEAR_OREFLAG(huart2); // Not sure if actually needed


	if (!FDCAN_Config(hcan))
	{
		Error_Handler();
	}


	HAL_TIM_Base_Start_IT(a_loopTimHandler);
	uint32_t waiting_time = 5; // ms

	s_uart_broker.initDMA(huart2);

	CAN::DigitalOutputs init_digital_outputs;
	init_digital_outputs.enable_outputs = 0;
	init_digital_outputs.enable_power= 0;
	digital_outputs_cb(init_digital_outputs);

	while(true) {

		myboard.update();
		//HAL_Delay(1); // ms

		myboard.updateCurrent();
		CAN_ProcessTxQueue(hcan);

		HAL_Delay(waiting_time - 1); // ms
	}
}
