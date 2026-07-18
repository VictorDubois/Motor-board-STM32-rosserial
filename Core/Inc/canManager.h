/*
 * canManager.h
 *
 */

#ifndef INC_CANMANAGER_H_
#define INC_CANMANAGER_H_

#include "stm32g4xx_hal.h"
#include <CanStruct/can_structs.h>
#include "math.h"

// Inspired by https://community.st.com/t5/stm32-mcus/how-to-use-fdcan-to-create-a-simple-communication-with-a-basic/ta-p/671766


// tx_queue.h
#define SW_TX_QUEUE_SIZE 32  // Adjust as needed

typedef struct {
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CAN_TxMessage_t;

typedef struct {
    CAN_TxMessage_t buffer[SW_TX_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} CAN_TxQueue_t;

extern CAN_TxQueue_t canTxQueue;

bool CAN_Enqueue(FDCAN_TxHeaderTypeDef *header, uint8_t *data);

void CAN_ProcessTxQueue(FDCAN_HandleTypeDef* hcan);

bool FDCAN_Config(FDCAN_HandleTypeDef* hcan);

class CanBroker
{

public:
	void RxFifoCallback();
	void publishCurrent(int16_t left_current_mA, int16_t right_current_mA, uint16_t left_wheel_unstalled_in_ms, uint16_t right_wheel_unstalled_in_ms, float speedVx, float speedWz);
	void publishOdometry(float X, float Y, float current_theta_rad, int16_t currentLeft, int16_t currentRight);

private:
};

#endif /* INC_CANMANAGER_H_ */
