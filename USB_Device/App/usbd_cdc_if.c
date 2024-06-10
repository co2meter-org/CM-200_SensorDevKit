/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           USB_Device/CDC_Standalone/USB_Device/App/usbd_cdc_if.c
  * @author         MCD Application Team
  * @brief          This file implements the USB device descriptors.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"
//#include "custom_stm.h"
#include "custom_app.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};
uint32_t BuffLength;
uint32_t UserTxBufPtrIn;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
uint32_t UserTxBufPtrOut; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */
uint8_t UserRxBufferBLE[32];
uint8_t UserRxBufferPtrBLE = 0;
uint16_t UserRxBufferLengthBLE = 0;

extern UART_HandleTypeDef huart1;

__IO uint32_t uwPrescalerValue;
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USB handler declaration */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
 void TIM_Config(void);
 static void ComPort_Config(void);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
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
  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)UserTxBufferFS, 1) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }

  /*##-5- Set Application Buffers ############################################*/
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  /* DeInitialize the UART peripheral */
  if(HAL_UART_DeInit(&huart1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

    case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

    case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

    case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  	case CDC_SET_LINE_CODING:
		LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
								(pbuf[2] << 16) | (pbuf[3] << 24));
		LineCoding.format     = pbuf[4];
		LineCoding.paritytype = pbuf[5];
		LineCoding.datatype   = pbuf[6];

		/* Set the new configuration */
		ComPort_Config();
    break;

    case CDC_GET_LINE_CODING:
		pbuf[0] = (uint8_t)(LineCoding.bitrate);
		pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
		pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
		pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
		pbuf[4] = LineCoding.format;
		pbuf[5] = LineCoding.paritytype;
		pbuf[6] = LineCoding.datatype;
    break;

    case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */

    break;

    case CDC_SEND_BREAK:
    /* Add your code here */
    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  HAL_UART_Transmit_DMA(&huart1, Buf, *Len);
  //HAL_UART_Transmit(&UartHandle, Buf, *Len, 1000);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);//hUsbDeviceFS
}

/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None.
  * @note   When a configuration is not supported, a default value is used.
  */
static void ComPort_Config(void)
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
  HAL_UART_Receive_IT(&huart1, (uint8_t *)(UserTxBufferFS + UserTxBufPtrIn), 1);
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

	if (UserTxBufferFS[buffptr] != 0)
		USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBufferFS[buffptr], buffsize);

	if(USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK)
	{
		UserRxBufferBLE[UserRxBufferLengthBLE++] = UserTxBufferFS[buffptr];
		if(UserTxBufferFS[buffptr] == '\n' && UserRxBufferBLE[0] != 0xFE)
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
  HAL_UART_Receive_IT(huart, (uint8_t *)(UserTxBufferFS + UserTxBufPtrIn), 1);
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
