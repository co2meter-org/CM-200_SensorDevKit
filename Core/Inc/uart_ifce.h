/*
 * uart_ifce.h
 *
 *  Created on: Jun 10, 2024
 *      Author: robert.miller
 */

#ifndef INC_UART_IFCE_H_
#define INC_UART_IFCE_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "custom_app.h"

void ComPort_Config(USBD_CDC_LineCodingTypeDef LineCoding);
void uart_ifce_init();
 void uart_ifce_deinit();
 void uart_ifce_transmit_dma(const uint8_t *pData, uint16_t Size);
 void BLE_to_UART(uint8_t * bleBuf, uint16_t Len);

#endif /* INC_UART_IFCE_H_ */
