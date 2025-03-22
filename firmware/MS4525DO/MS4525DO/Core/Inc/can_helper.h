/*
 * can_helper.h
 *
 *  Created on: Mar 21, 2025
 *      Author: ellay
 */

#ifndef INC_CAN_HELPER_H_
#define INC_CAN_HELPER_H_

#include "main.h"

extern volatile uint8_t can_message_ready;
extern uint8_t last_rx_data[8];
extern uint32_t last_rx_id;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void send_can_message(uint8_t txData[8]);
void print_last_rx(void);

#endif /* INC_CAN_HELPER_H_ */
