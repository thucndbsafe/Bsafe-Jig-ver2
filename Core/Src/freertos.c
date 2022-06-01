/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "app_ethernet.h"
#include "app_debug.h"
#include "lwip.h"
#include "stdio.h"
//#include "app_http.h"
#include "lwip/dns.h"
//#include "mqtt_client.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


/*********************       NET VAR    **********************************/
	struct netif g_netif;
	static TaskHandle_t m_task_handle_protocol = NULL; //NET APP TASK
	osThreadId DHCP_id;
	SemaphoreHandle_t hHttpStart;
	xQueueHandle httpQueue;
	bool send_offline_file = false;
	SemaphoreHandle_t sent_an_offline_file;
/*****************************************************************/


/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
//  MX_LWIP_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  HAL_GPIO_WritePin (LED1_G_GPIO_Port,LED1_G_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin (LED1_R_GPIO_Port,LED1_R_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin (LED2_G_GPIO_Port,LED2_G_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin (LED2_R_GPIO_Port,LED2_R_Pin,GPIO_PIN_SET);
  DEBUG_INFO ("TEST DEBUG OK \r\n");
#if LWIP_DHCP
	  /* Start DHCPClient */
	  osThreadDef(DHCP, DHCP_Thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
	  DHCP_id = osThreadCreate (osThread(DHCP), &g_netif);
#endif
  for(;;)
  {
	  if (HAL_GPIO_ReadPin(BT_IN_GPIO_Port, BT_IN_Pin) == 0)
	  {
		  HAL_GPIO_TogglePin (LED1_G_GPIO_Port, LED1_G_Pin);
		  HAL_GPIO_TogglePin (LED2_G_GPIO_Port, LED2_G_Pin);
		  HAL_GPIO_TogglePin (LED1_R_GPIO_Port, LED1_R_Pin);
		  HAL_GPIO_TogglePin (LED2_R_GPIO_Port, LED2_R_Pin);
//		  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
//		vTaskDelay(1000);
//		HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	  }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Netif_Config (bool restart)
{
	ip4_addr_t ipaddr;
	ip4_addr_t netmask;
	ip4_addr_t gw;
	  /* IP addresses initialization with DHCP (IPv4) */
	  ipaddr.addr = 0;
	  netmask.addr = 0;
	  gw.addr = 0;
	  if (restart)
	  {
			netif_remove (&g_netif);
			DEBUG_INFO ("NET IF REMOVE \r\n");
	  /* Start DHCP negotiation for a network interface (IPv4) */

//#if LWIP_DHCP
//	  /* Start DHCPClient */
//	  osThreadDef(DHCP, DHCP_Thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
//	  DHCP_id = osThreadCreate (osThread(DHCP), &g_netif);
//#endif
	  }
	  /* add the network interface (IPv4/IPv6) with RTOS */
	  netif_add(&g_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);//&tcpip_input =>null

	  /* Registers the default network interface */
	  netif_set_default(&g_netif);
	  netif_set_link_callback(&g_netif, ethernet_link_status_updated);
	  if (netif_is_link_up(&g_netif))
	  {
	    /* When the netif is fully configured this function must be called */
	    netif_set_up(&g_netif);
	  }
	  else
	  {
	    /* When the netif link is down this function must be called */
	    netif_set_down(&g_netif);
	  }
	  app_ethernet_notification(&g_netif);
	  /* Set the link callback function, this function is called on change of link status*/


	  DEBUG_INFO ("SET LINK CALLBACK \r\n");

}
/* USER CODE END Application */

