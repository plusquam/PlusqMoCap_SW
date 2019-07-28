/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : p2p_server_app.c
 * Description        : P2P Server Application
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "scheduler.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef struct{
    uint8_t             Device_Led_Selection;
    uint8_t             Led1;
 }P2P_LedCharValue_t;

 typedef struct{
    uint8_t             Device_Button_Selection;
    uint8_t             ButtonStatus;
 }P2P_ButtonCharValue_t;

typedef struct
{
  uint8_t               Notification_Status; /* used to chek if P2P Server is enabled to Notify */
  P2P_LedCharValue_t    LedControl;
  P2P_ButtonCharValue_t ButtonControl;
  uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_Server_App_Context_t P2P_Server_App_Context;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */

/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */

/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;
      
    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
    	if(pNotification->DataTransfered.pPayload[0] == 0x01) {
			if(pNotification->DataTransfered.pPayload[1] == 0x01)
			{
//				if 2 nd byte of P2P_WRITE characteristic value is 0x00
				/* Turn the blue LED ON */
				HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
				P2P_Server_App_Context.LedControl.Led1=0x01; //LED1 ON
			}
			else if(pNotification->DataTransfered.pPayload[1] == 0x00)
			{
//				if 2 nd byte of P2P_WRITE characteristic value is 0x01
				/* Turn the blue LED OFF */
				HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
				P2P_Server_App_Context.LedControl.Led1=0x00; //LED1 OFF
			}
    	}

/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */

/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */

/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;
    
    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */

/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
