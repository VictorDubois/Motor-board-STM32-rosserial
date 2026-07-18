// tx_queue.c

#include "canManager.h"
#include <string.h> // memcpy
#include "mainpp.h"
#include "constants.h"
#include "callbacks.h"

CAN_TxQueue_t canTxQueue = {0};

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

// Call this from your ISRs - just enqueues, never touches HAL
bool CAN_Enqueue(FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    if (canTxQueue.count >= SW_TX_QUEUE_SIZE)
        return false;  // Queue full!

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    canTxQueue.buffer[canTxQueue.tail].header = *header;
   memcpy(canTxQueue.buffer[canTxQueue.tail].data, data, 8);
    canTxQueue.tail = (canTxQueue.tail + 1) % SW_TX_QUEUE_SIZE;
    canTxQueue.count++;

    __set_PRIMASK(primask);
    return true;
}

// Call this from main loop ONLY - safe HAL access
void CAN_ProcessTxQueue(FDCAN_HandleTypeDef* hcan)
{
    while (canTxQueue.count > 0)
    {
        // Check HW FIFO not full (max 3 slots)
        if ((hcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U)
            continue;  // HW full, try later

        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        CAN_TxMessage_t msg = canTxQueue.buffer[canTxQueue.head];
        canTxQueue.head = (canTxQueue.head + 1) % SW_TX_QUEUE_SIZE;
        canTxQueue.count--;

        __set_PRIMASK(primask);

        // Only called from main loop = safe!
        HAL_FDCAN_AddMessageToTxFifoQ(hcan, &msg.header, msg.data);
    }
}

void CanBroker::publishOdometry(float X, float Y, float current_theta_rad, int16_t currentLeft, int16_t currentRight)
{
	/* Set the data to be transmitted */
	TxHeader.Identifier = CAN::can_ids::ODOMETRY_XYum;
	int32_t poseX_um = X * 1000000;
	int32_t poseY_um = Y * 1000000;

	TxData[0] = (poseX_um >> 24) & 0xFF;
	TxData[1] = (poseX_um >> 16) & 0xFF;
	TxData[2] = (poseX_um >> 8) & 0xFF;
	TxData[3] = (poseX_um) & 0xFF;
	TxData[4] = (poseY_um >>24) & 0xFF;
	TxData[5] = (poseY_um >> 16) & 0xFF;
	TxData[6] = (poseY_um >> 8) & 0xFF;
	TxData[7] = (poseY_um) & 0xFF;
	CAN_Enqueue(&TxHeader, TxData);


	TxHeader.Identifier = CAN::can_ids::ODOMETRY_THETA;
	int32_t angleRz_centi_deg = current_theta_rad * (100.0f * 180.f / M_PI);

	TxData[0] = (angleRz_centi_deg >> 24) & 0xFF;
	TxData[1] = (angleRz_centi_deg >> 16) & 0xFF;
	TxData[2] = (angleRz_centi_deg >> 8) & 0xFF;
	TxData[3] = (angleRz_centi_deg) & 0xFF;
	TxData[4] = (currentLeft >>8) & 0xFF;
	TxData[5] = (currentLeft) & 0xFF;
	TxData[6] = (currentRight >> 8) & 0xFF;
	TxData[7] = (currentRight) & 0xFF;
	CAN_Enqueue(&TxHeader, TxData);
}

void CanBroker::publishCurrent(int16_t left_current_mA, int16_t right_current_mA, uint16_t left_wheel_unstalled_in_ms, uint16_t right_wheel_unstalled_in_ms, float speedVx, float speedWz)
{
	TxHeader.Identifier = CAN::can_ids::CURRENT_LIMIT;

	TxData[0] = (left_current_mA >> 8) & 0xFF;
	TxData[1] = (left_current_mA ) & 0xFF;
	TxData[2] = (right_current_mA >> 8) & 0xFF;
	TxData[3] = (right_current_mA ) & 0xFF;
	TxData[4] = (left_wheel_unstalled_in_ms >> 8) & 0xFF;
	TxData[5] = (left_wheel_unstalled_in_ms ) & 0xFF;
	TxData[6] = (right_wheel_unstalled_in_ms >> 8) & 0xFF;
	TxData[7] = (right_wheel_unstalled_in_ms) & 0xFF;

	CAN_Enqueue(&TxHeader, TxData);

	TxHeader.Identifier = CAN::can_ids::ODOMETRY_SPEED;
	int32_t speedVx_µm_s = speedVx * 1000000.f;   // 4 bytes
	int32_t speedWz_mrad_s = speedWz * 1000.f; // 4 bytes

	TxData[0] = (speedVx_µm_s >> 24) & 0xFF;
	TxData[1] = (speedVx_µm_s >> 16) & 0xFF;
	TxData[2] = (speedVx_µm_s >> 8) & 0xFF;
	TxData[3] = (speedVx_µm_s ) & 0xFF;
	TxData[4] = (speedWz_mrad_s >> 24) & 0xFF;
	TxData[5] = (speedWz_mrad_s >> 16) & 0xFF;
	TxData[6] = (speedWz_mrad_s >> 8) & 0xFF;
	TxData[7] = (speedWz_mrad_s) & 0xFF;

	CAN_Enqueue(&TxHeader, TxData);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	  {
	    /* Retrieve Rx messages from RX FIFO0 */
	    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	    {
	        //Error_Handler();
	    	return;
	    }

	    if ((RxHeader.Identifier == CAN::can_ids::CMD_VEL_FLOAT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	    {
	    	CAN::CmdVelFloat l_cmd_vel;
	        memcpy(&(l_cmd_vel.linear_x_m), RxData, sizeof(float));
	        memcpy(&(l_cmd_vel.angular_z_rad), RxData + sizeof(float), sizeof(float));

	        MotorBoard::getDCMotor().set_speed_order(metersToTicks(l_cmd_vel.linear_x_m), radsToTicks(-l_cmd_vel.angular_z_rad));
	    }

	    else if ((RxHeader.Identifier == CAN::can_ids::C620_OUTPUT_1) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
		{
			uint16_t l_mechanical_angle_8192_ticks = 0;
			int16_t l_speed_rpm = 0;
			int16_t l_torque = 0;

			l_mechanical_angle_8192_ticks |= RxData[0] << 8;
			l_mechanical_angle_8192_ticks |= RxData[1];
			l_speed_rpm |= RxData[2] << 8;
			l_speed_rpm |= RxData[3] ;
			l_torque |= RxData[4] << 8;
			l_torque |= RxData[5] ;


	#ifdef USE_CAN_SPEED_ODOMETRY
			float l_speed_meter_s = l_speed_rpm * MOTOR_RPM_TO_WHEEL_M_S;
			MotorBoard::getDCMotor().set_speed(M_L, metersToTicks(l_speed_meter_s));
			MotorBoard::getDCMotor().set_ticks(M_L, l_mechanical_angle_8192_ticks);
	#endif

	#ifdef USE_C620_CURRENT
			MotorBoard::getDCMotor().set_current(M_L, l_torque);
	#endif
		}

	    else if ((RxHeader.Identifier == CAN::can_ids::C620_OUTPUT_2) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
		{
			uint16_t l_mechanical_angle_8192_ticks = 0;
			int16_t l_speed_rpm = 0;
			int16_t l_torque = 0;

			l_mechanical_angle_8192_ticks |= RxData[0] << 8;
			l_mechanical_angle_8192_ticks |= RxData[1];
			l_speed_rpm |= RxData[2] << 8;
			l_speed_rpm |= RxData[3] ;
			l_torque |= RxData[4] << 8;
			l_torque |= RxData[5] ;


	#ifdef USE_CAN_SPEED_ODOMETRY
			float l_speed_meter_s = l_speed_rpm * MOTOR_RPM_TO_WHEEL_M_S;
			MotorBoard::getDCMotor().set_speed(M_R, metersToTicks(l_speed_meter_s));
			MotorBoard::getDCMotor().set_ticks(M_R, l_mechanical_angle_8192_ticks);
	#endif

	#ifdef USE_C620_CURRENT
			MotorBoard::getDCMotor().set_current(M_R, l_torque);
	#endif
		}

	    else if ((RxHeader.Identifier == CAN::can_ids::CMD_VEL) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
		{
			CAN::CmdVelFloat l_cmd_vel;

			int32_t linear_x_µm_s = 0;
			int32_t angular_z_µrad_s = 0;
			linear_x_µm_s |= RxData[0] << 24;
			linear_x_µm_s |= RxData[1] << 16;
			linear_x_µm_s |= RxData[2] << 8;
			linear_x_µm_s |= RxData[3] ;

			angular_z_µrad_s |= RxData[4] << 24;
			angular_z_µrad_s |= RxData[5] << 16;
			angular_z_µrad_s |= RxData[6] << 8;
			angular_z_µrad_s |= RxData[7] ;

			l_cmd_vel.linear_x_m = linear_x_µm_s/(1000000.0f);
			l_cmd_vel.angular_z_rad = angular_z_µrad_s/(1000000.0f);

			MotorBoard::getDCMotor().set_speed_order(metersToTicks(l_cmd_vel.linear_x_m), radsToTicks(-l_cmd_vel.angular_z_rad));
		}

	    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_CMD_INPUT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
		{
			CAN::MotorBoardCmdInput l_cmd_inputs;

			//memcpy(&(l_cmd_inputs), RxData, 8*sizeof(uint8_t));

			l_cmd_inputs.enable_motors = RxData[0];
			l_cmd_inputs.override_PWM = RxData[1];
			l_cmd_inputs.PWM_override_left = RxData[3] | (RxData[2] << 8);
			l_cmd_inputs.PWM_override_right = RxData[5] | (RxData[4] << 8);
			l_cmd_inputs.reset_encoders = RxData[6];

			motors_cmd_cb(l_cmd_inputs);
		}
	    else if ((RxHeader.Identifier == CAN::can_ids::DIGITAL_OUTPUTS) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	    {
			CAN::DigitalOutputs l_digital_outputs;

			l_digital_outputs.enable_outputs = RxData[1] | (RxData[0] << 8);
			l_digital_outputs.enable_power = RxData[2];

			digital_outputs_cb(l_digital_outputs);
		}
	    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_CURRENT_INPUT) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
		{
			CAN::MotorBoardCurrentInput l_cmd_current_inputs;

			//memcpy(&(l_cmd_current_inputs), RxData, 8*sizeof(uint8_t));
			l_cmd_current_inputs.max_current_left_mA = RxData[1] | (RxData[0] << 8);
			l_cmd_current_inputs.max_current_right_mA = RxData[3] | (RxData[2] << 8);
			l_cmd_current_inputs.max_current_mA = RxData[5] | (RxData[4] << 8);

			MotorBoard::getDCMotor().set_max_current(l_cmd_current_inputs.max_current_mA/1000.0f);
			MotorBoard::getDCMotor().set_max_current(l_cmd_current_inputs.max_current_left_mA/1000.0f, l_cmd_current_inputs.max_current_right_mA/1000.0f);
		}

	    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_ENABLE) && (RxHeader.IdType == FDCAN_STANDARD_ID))
		{
			bool l_motor_enable = RxData[0];

			if (!l_motor_enable)
			{
				MotorBoard::getDCMotor().resetMotor(M_L);
				MotorBoard::getDCMotor().resetMotor(M_R);
			}

		}
	    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_LINEAR_PI_SET) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	    {
	        CAN::MotorBoardPiSet msg;
	        memcpy(&msg, RxData, sizeof(msg));
	        MotorBoard::getDCMotor().set_linear_pi(msg.p, msg.i);
	    }
	    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_ANGULAR_PI_SET) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	    {
	        CAN::MotorBoardPiSet msg;
	        memcpy(&msg, RxData, sizeof(msg));
	        MotorBoard::getDCMotor().set_angular_pi(msg.p, msg.i);
	    }
	    else if ((RxHeader.Identifier == CAN::can_ids::MOTOR_BOARD_DERIVATIVE_SET) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
	    {
	        CAN::MotorBoardDerivativeSet msg;
	        memcpy(&msg, RxData, sizeof(msg));
	        MotorBoard::getDCMotor().set_derivative(msg.linear_d, msg.angular_d);
	    }
	  }
}

/**
  * @brief  Configures the FDCAN.
  *   None
  * @retval None
  */
bool FDCAN_Config(FDCAN_HandleTypeDef* hcan)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 42;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    return false;
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(hcan) != HAL_OK)
  {
	return false;
  }

  if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
	return false;
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x321;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

	/* Start the Transmission process */
	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, TxData) != HAL_OK)
	{
		/* Transmission request Error */
		MotorBoard::getDCMotor().resetMotors();
	}
	return true;
}
