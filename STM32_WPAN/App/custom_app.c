/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* CM200_SensorDevKit */
  uint8_t               Read_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
uint8_t bConnected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* CM200_SensorDevKit */
static void Custom_Read_Update_Char(void);
static void Custom_Read_Send_Notification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */
	  case 32:
		Custom_Read_Update_Char();
		break;

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* CM200_SensorDevKit */
    case CUSTOM_STM_WRITE_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_WRITE_READ_EVT */

      /* USER CODE END CUSTOM_STM_WRITE_READ_EVT */
      break;

    case CUSTOM_STM_WRITE_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_WRITE_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_WRITE_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_READ_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_READ_READ_EVT */

      /* USER CODE END CUSTOM_STM_READ_READ_EVT */
      break;

    case CUSTOM_STM_READ_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_READ_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_READ_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_READ_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_READ_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_READ_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
    	bConnected = 1;
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
    	bConnected = 0;
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	UTIL_SEQ_RegTask(1<<CUSTOM_STM_READ_NOTIFY_ENABLED_EVT, UTIL_SEQ_RFU, Custom_STM_App_Notification);

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* CM200_SensorDevKit */
void Custom_Read_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Read_UC_1*/
  Custom_STM_App_Update_Char(CUSTOM_STM_READ, (uint8_t *)UpdateCharData);

  /* USER CODE END Read_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_READ, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Read_UC_Last*/

  /* USER CODE END Read_UC_Last*/
  return;
}

void Custom_Read_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Read_NS_1*/

  /* USER CODE END Read_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_READ, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Read_NS_Last*/

  /* USER CODE END Read_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void Write_UART_To_BLE(uint8_t * bBuff, size_t bBuffSize)
{
	strncpy(UpdateCharData, bBuff, bBuffSize);
	// CUSTOM_STM_WRITE_WRITE_NO_RESP_EVT
	UTIL_SEQ_SetTask( 1<<CUSTOM_STM_READ_NOTIFY_ENABLED_EVT, CFG_SCH_PRIO_0);
	//Custom_Read_Update_Char();
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
