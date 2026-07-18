/*
 * callbacks.h
 *
 *  Created on: Jun 8, 2026
 *      Author: victor.dubois@navya.tech
 */

#ifndef INC_CALLBACKS_H_
#define INC_CALLBACKS_H_

#include "msgs.h"


void motors_cmd_hex_cb(const uint8_t* a_message);
void parameters_hex_cb(const uint8_t* a_message);

void cmd_vel_hex_cb(const uint8_t* a_message);
void enable_motor_hex_cb(const uint8_t* a_message);

void motors_cmd_cb(const CAN::MotorBoardCmdInput &motors_cmd_msg);
void motors_cmd_cb(const krabi_msgs::motors_cmd &motors_cmd_msg);
void digital_outputs_cb(const CAN::DigitalOutputs &digital_outputs_msg);
void cmd_vel_cb(const geometry_msgs::Twist& twist);
void parameters_cb(const krabi_msgs::motors_parameters& parameters);
void enable_motor_cb(const std_msgs::Bool& enable);




#endif /* INC_CALLBACKS_H_ */
