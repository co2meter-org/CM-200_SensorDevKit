/*
 * uart_ifce.c
 *
 *  Created on: Jun 10, 2024
 *      Author: robert.miller
 */

#include "uart_ifce.h"

#define BUFF_LENGTH  2048
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
//uint8_t UserRxBuffer[BuffLength];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBuffer[BUFF_LENGTH] = { 0 };

uint32_t UserTxBufPtrIn;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
uint32_t UserTxBufPtrOut; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */
uint8_t UserRxBufferBLE[32];
uint8_t UserRxBufferPtrBLE = 0;
uint16_t UserRxBufferLengthBLE = 0;

extern UART_HandleTypeDef huart1;

__IO uint32_t uwPrescalerValue;

void uart_ifce_init()
{
	  /* USER CODE BEGIN 3 */
	  /*##-1- Configure the UART peripheral ######################################*/
	  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	  /* USART configured as follow:
	      - Word Length = 8 Bits
	      - Stop Bit    = One Stop bit
	      - Parity      = No parity
	      - BaudRate    = 115200 baud
	      - Hardware flow control disabled (RTS and CTS signals) */
	  huart1.Instance          = USART1;
	  huart1.Init.BaudRate     = 9600;
	  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
	  huart1.Init.StopBits     = UART_STOPBITS_1;
	  huart1.Init.Parity       = UART_PARITY_NONE;
	  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	  huart1.Init.Mode         = UART_MODE_TX_RX;

	  if(HAL_UART_Init(&huart1) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

	  /*##-2- Put UART peripheral in IT reception process ########################*/
	  /* Any data received will be stored in "UserTxBufferFS" buffer  */
	  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)UserTxBuffer, 1) != HAL_OK)
	  {
	    /* Transfer error in reception process */
	    Error_Handler();
	  }
}

void uart_ifce_deinit()
{
	  if(HAL_UART_DeInit(&huart1) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
	USB_Rcv();
}

/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None.
  * @note   When a configuration is not supported, a default value is used.
  */
void ComPort_Config(USBD_CDC_LineCodingTypeDef LineCoding)
{
  if(HAL_UART_DeInit(&huart1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* set the Stop bit */
  switch (LineCoding.format)
  {
  case 0:
	  huart1.Init.StopBits = UART_STOPBITS_1;
    break;
  case 2:
	  huart1.Init.StopBits = UART_STOPBITS_2;
    break;
  default :
	  huart1.Init.StopBits = UART_STOPBITS_1;
    break;
  }

  /* set the parity bit*/
  switch (LineCoding.paritytype)
  {
  case 0:
	  huart1.Init.Parity = UART_PARITY_NONE;
    break;
  case 1:
	  huart1.Init.Parity = UART_PARITY_ODD;
    break;
  case 2:
	  huart1.Init.Parity = UART_PARITY_EVEN;
    break;
  default :
	  huart1.Init.Parity = UART_PARITY_NONE;
    break;
  }

  /*set the data type : only 8bits and 9bits is supported */
  switch (LineCoding.datatype)
  {
  case 0x07:
    /* With this configuration a parity (Even or Odd) must be set */
	  huart1.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  case 0x08:
    if(huart1.Init.Parity == UART_PARITY_NONE)
    {
    	huart1.Init.WordLength = UART_WORDLENGTH_8B;
    }
    else
    {
    	huart1.Init.WordLength = UART_WORDLENGTH_9B;
    }

    break;
  default :
	  huart1.Init.WordLength = UART_WORDLENGTH_8B;
    break;
  }

  huart1.Init.BaudRate     = LineCoding.bitrate;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&huart1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Start reception: provide the buffer pointer with offset and the buffer size */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)(UserTxBuffer + UserTxBufPtrIn), 1);
}


/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Transfer error occurred in reception and/or transmission process */
  Error_Handler();
}


// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(const unsigned char * buf, int len)
{
	uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
	uint16_t nbyte = (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
    crc ^= nbyte;

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

void uart_ifce_transmit_dma(const uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

static uint8_t check_crc()
{
	if (UserRxBufferLengthBLE < 7)
		return 0;

	unsigned char a_SzString_CPY[UserRxBufferLengthBLE - 2];
	memcpy(a_SzString_CPY, UserRxBufferBLE, UserRxBufferLengthBLE - 2);
	uint16_t modbus_crc = ModRTU_CRC(a_SzString_CPY, UserRxBufferLengthBLE - 2);
	uint8_t modbus_crc_lo = (uint8_t)((modbus_crc & 0xff00) >> 8);
	uint8_t modbus_crc_hi = (uint8_t)(modbus_crc & 0xff);

	if (UserRxBufferBLE[UserRxBufferLengthBLE - 2] == modbus_crc_hi && UserRxBufferBLE[UserRxBufferLengthBLE - 1] == modbus_crc_lo)
		return 1;

	return 0;
}

void txToUSB()
{
  uint32_t buffptr;
  uint32_t buffsize;
  uint8_t status = USBD_OK;

  if(UserTxBufPtrOut != UserTxBufPtrIn)
  {
	if(UserTxBufPtrOut > UserTxBufPtrIn) /* Rollback */
	{
	  buffsize = APP_RX_DATA_SIZE - UserTxBufPtrOut;
	}
	else
	{
	  buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
	}

	buffptr = UserTxBufPtrOut;

	if (UserTxBuffer[buffptr] != 0)
		status = CDC_Transmit_FS((uint8_t*)&UserTxBuffer[buffptr], buffsize);

	if(status == USBD_OK)
	{

	}
	UserRxBufferBLE[UserRxBufferLengthBLE++] = UserTxBuffer[buffptr];
	if(UserTxBuffer[buffptr] == '\n' && UserRxBufferBLE[0] != 0xFE)
	{
		Write_UART_To_BLE(UserRxBufferBLE, UserRxBufferLengthBLE);
		UserRxBufferLengthBLE = 0;
	}
	else if (check_crc() == 1)
	{
		Write_UART_To_BLE(UserRxBufferBLE, UserRxBufferLengthBLE);
		UserRxBufferLengthBLE = 0;
	}
	else
	{
		if(UserRxBufferLengthBLE >= 32 - 1)
		{
			Write_UART_To_BLE(UserRxBufferBLE, UserRxBufferLengthBLE);
			UserRxBufferLengthBLE = 0;
		}
	}
	//uint8_t bleTXBuffer[buffsize];
	//strncpy((char *)bleTXBuffer, (char *)UserTxBufferFS[buffptr], buffsize);


	UserTxBufPtrOut += buffsize;
	if (UserTxBufPtrOut == APP_RX_DATA_SIZE)
	{
		UserTxBufPtrOut = 0;
	}
  }
}

void BLE_to_UART(uint8_t * bleBuf, uint16_t Len)
{
	  HAL_UART_Transmit_DMA(&huart1, bleBuf, Len);
}


/**
  * @brief  Rx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Increment Index for buffer writing */
  UserTxBufPtrIn++;

  /* To avoid buffer overflow */
  if(UserTxBufPtrIn == APP_RX_DATA_SIZE)
  {
    UserTxBufPtrIn = 0;
  }
  txToUSB();

  /* Start another reception: provide the buffer pointer with offset and the buffer size */
  HAL_UART_Receive_IT(huart, (uint8_t *)(UserTxBuffer + UserTxBufPtrIn), 1);
}
