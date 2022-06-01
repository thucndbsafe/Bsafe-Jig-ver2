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
#include "usbd_storage_if.h"
#include "tusb.h"
#include "app_debug.h"
#include "fatfs.h"
#include "lwrb.h"
#include <stdbool.h>
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "esp_loader.h"
#include "example_common.h"
#include "md5_hash.h"
#include "app_btn.h"
#include "utilities.h"
#include "usart.h"
#include "stm32_port.h"
#include "app_ethernet.h"
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip.h"
#include "stdio.h"
#include "app_http.h"
#include "lwip/dns.h"
#include "mqtt_client.h"
#include "min.h"
#include "min_id.h"
#include "ringBuffer.h"
#include "adc.h"
#include "sntp.h"
#include "rtc.h"
#include "time.h"
#include "app_cli.h"
#include "iwdg.h"
#include "ff.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} date_time_t;

typedef enum
{
	NOT_CONNECTED,
	CONNECT,
	ETHERNET_CONNECTED
} ETHERNET_STATE;
typedef union
{
	struct
	{
		uint32_t rs485: 1;
		uint32_t rs232: 1;
		uint32_t test_wd_ok: 1;
		uint32_t sim_ok :1;
		uint32_t temper_ok: 1;
//		uint32_t vin_ok: 1;
		uint32_t vbat_ok: 1;
		uint32_t v1v8_ok: 1;
		uint32_t v3v3_ok: 1;
		uint32_t v5v_ok: 1;
		uint32_t vgsm4v2_ok: 1;
		uint32_t charge_ok: 1;
		uint32_t alarm_ok:1;
		uint32_t fault_ok:1;
		uint32_t relay0_ok: 1;
		uint32_t relay1_ok: 1;
		uint32_t buttonTest_ok: 1;
		uint32_t sosButton_ok: 1;
		uint32_t vsys_ok:1;
		uint32_t reserved : 13;
	} result;
	uint32_t value;
}__attribute__((packed)) func_test_t;
typedef union
{

	struct

	{

		uint16_t eth : 1;

		uint16_t wifi : 1;

		uint16_t gsm : 1;

		uint16_t server : 1;

		uint16_t input0_pass : 1;

		uint16_t input1_pass : 1;

		uint16_t input2_pass : 1;

		uint16_t input3_pass : 1;

		uint16_t button_pass : 1;

		uint16_t main_power_pass : 1;

		uint16_t backup_power_pass : 1;

		uint16_t reserve : 5;

	} name;

	uint16_t value;

} __attribute__((packed)) jig_peripheral_t;
typedef struct

{

	char gsm_imei[16];

	char sim_imei[16];

	uint8_t mac[6];

	uint16_t gsm_voltage;

	jig_peripheral_t peripheral;

    uint8_t temperature;

    uint8_t device_type;  // 0 = "B01", 1 = "B02"

    uint8_t fw_version[3];  // Major.minor.build

    uint8_t hw_version[3];  // Major.minor.build

    func_test_t test_result;

    uint32_t timestamp;
} __attribute__((packed)) jig_value_t;

typedef struct
{
	uint16_t vin_max;
	uint16_t vin_min;
	uint16_t vbat_max;
	uint16_t vbat_min;
	uint16_t v1v8_max;
	uint16_t v1v8_min;
	uint16_t v3v3_max;
	uint16_t v3v3_min;
	uint16_t v5v_max;
	uint16_t v5v_min;
	uint16_t v4v2_max;
	uint16_t v4v2_min;
	uint16_t vsys_max;
	uint16_t vsys_min;
} __attribute__((packed)) test_info;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_CDC_TX_RING_BUFFER_SIZE		1024
// BUTTON DEFINE
#define BIT_EVENT_GROUP_BT_IN_PRESSED        (1 << 0)
#define HW_BTN_CONFIG                                       \
{                                                           \
    /* PinName       Last state   Idle level*/              \
	{0,				1,				1						}}

#define TIMEOUT 8000 // TEST TIMEOUT
static const uint8_t day_in_month[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
#define FIRSTYEAR 2000 // start year
#define FIRSTDAY 6     // 0 = Sunday

// WDT DEFINE
EventGroupHandle_t m_wdg_event_group = NULL;
#define defaultTaskB 	(1 << 0)
#define cdcTaskB 		(1 << 1)
#define usbTaskB 		(1 << 2)
#define flashTaskB 		(1 << 3)
#define netTaskB 		(1 << 4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static EventBits_t uxBits;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/**************        BUTTON VAR               ***********/
	static app_btn_hw_config_t m_button_cfg[] = HW_BTN_CONFIG;
	static EventGroupHandle_t m_button_event_group = NULL;
/**********************************************************/

/*********************      flash task var             ***********************/
	static TaskHandle_t m_task_connect_handle = NULL;

	static example_binaries_t m_binary;
	static char m_file_address[128];
	volatile uint32_t led_busy_toggle = 0;

	static const char *info_file = "info.txt";
	static const char *bootloader_file = "bootloader.bin";
	static const char *firmware_file = "fire_safe_normal.bin";
	static const char *ota_file = "ota_data_initial.bin";
	static const char *partition_file = "partition-table.bin";

	static const char *chip_des[] = {"ESP8266", "ESP32", "ESP32S2", "ESP32C3", "ESP32S3", "ESP32C2", "ESP32H2", "UNKNOWN"};
/***********************************************************************************************************/

//************************** MD5 CRC CONFIG VAR **********************************************************//
	static uint8_t m_buffer[4096];
	static MD5Context_t md5_context;
	static esp_loader_config_t m_loader_cfg =
	{
		.baud_rate = 115200,
		.reset_trigger_pin = ESP_EN_Pin,
		.gpio0_trigger_pin = ESP_IO0_Pin,
		.buffer = m_buffer,
		.buffer_size = 4096,
		.sync_timeout = 100,
		.trials = 10,
		.md5_context = &md5_context
	};
//***********************************************************************************************//

/*************************** 	  FLASH DISK VARIABLE        *************************************************************/
	BYTE gFSWork[_MAX_SS];
	UINT br, bw;  // File read/write count
	UINT fbr, fbw;  // File read/write count
	FRESULT flash_res;
	bool m_disk_is_mounted = false;
/***********************************************************************************************************************/

/********************************  	 CDC TASK VAR     *****************************************************************/
	StackType_t  cdc_stack[configMINIMAL_STACK_SIZE];
	StaticTask_t cdc_taskdef;
/**********************************************************************************************************************/

//********************************    tiny USB TASK VAR     ********************************************************//
	static TaskHandle_t m_USB_handle = NULL;
	bool tusb_init_flag = false;

//********************************************************************************************************************//

/*********************8*************       NET VAR    ******************************************************************/
	struct netif g_netif;
	static TaskHandle_t m_task_handle_protocol = NULL; //NET APP TASK
	osThreadId DHCP_id;
	SemaphoreHandle_t hHttpStart;
	xQueueHandle httpQueue;
	bool send_offline_file = false;
	SemaphoreHandle_t sent_an_offline_file;
/*********************************************************************************************************************/

//***************************  TESTING TASK VAR   *********************************//
	static TaskHandle_t mTest_Handle_t = NULL;
	char gsm_imei[16];
	char sim_imei[16];
	uint8_t MAC[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t lastMAC[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static jig_value_t *rx_value;
	static jig_value_t *to_send_value;
	jig_peripheral_t jig_result;
	bool get_jig_info = false;
	static func_test_t test_res;
	bool allPassed = false;
	static const char *test_info_file = "test_info.txt";
	char json_send_to_sever[1024];
	test_info voltage_info;
	uint8_t V_info_buff[512];
	uint8_t relay_toggle_0;
	uint8_t relay_toggle_1;
	bool ready_send_to_sever = false;
	// adc read var
	uint16_t ADCScan[6];
	uint8_t idle_detect;
//*****************************************************************************//

//************************* TIME PROTOCOL PRO AND VAR***********************************//
	void lwip_sntp_recv_cb (uint32_t time);
//*******************************************************************************************//

//************************** RTC VAR*******************************//

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_TimeTypeDef sTimeToSend = {0};
  RTC_DateTypeDef sDateToSend = {0};
//  RTC_HandleTypeDef hrtc1;
  static date_time_t date_time;
//********************************************************************//

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//***************************************   TASK PFT *****************************************************************//
void cdc_task(void* params);

void usb_task (void* params);
//***************************************************************************************************************//

void reInitRTC ( RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate);

static void initialize_stnp(void);
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
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	/********  DISCONNECT USB TO MAKE RENUM EVENT  *********/
	MX_FATFS_Init();

	vTaskDelay(500);		// time for usb renum

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/*            RECONNECT USB         */
	//************************* INIT FUNCTIONS **************************//
	MX_USB_DEVICE_Init();
//***********************     MOUNT FLASH DISK	*************************//
	flash_res = f_mount(&USERFatFS, USERPath, 1);
	  if (flash_res != FR_OK)
	  {
		DEBUG_WARN("Mount flash fail\r\n");
		flash_res = f_mkfs(USERPath, FM_ANY, 0, gFSWork, sizeof gFSWork);
		flash_res = f_mount(&USERFatFS, USERPath, 1);
		if (flash_res == FR_OK)
		{
			m_disk_is_mounted = true;
			DEBUG_INFO ("format disk and mount again\r\n");
		}
		else
		{
			DEBUG_ERROR("Mount flash error\r\n");
		}
	  }
	  else
	  {
		m_disk_is_mounted = true;
		DEBUG_INFO ("Mount flash ok\r\n");
	  }
	  TCHAR label[32];
	  f_getlabel(USERPath, label, 0);
	  DEBUG_INFO("Label %s\r\n", label);
	  if (strcmp(label, "BSAFE JIG"))
	  {
		DEBUG_INFO("Set label\r\n");
		f_setlabel("BSAFE JIG");
	  }
//***********************************************************************//
  /* Infinite loop */


#if LWIP_DHCP
	  /* Start DHCPClient */
	  osThreadDef(DHCP, DHCP_Thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
	  DHCP_id = osThreadCreate (osThread(DHCP), &g_netif);
#endif
	  if (m_USB_handle == NULL)
	  {
		  xTaskCreate(usb_task, "usb_task", 512, NULL, 4, &m_USB_handle);// pio =1
	  }
	  osDelay (200);
	  initialize_stnp();
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
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//***************************** USB MSC AND CDC TASK***************************************************************//
static bool m_cdc_debug_register = false;
static lwrb_t m_ringbuffer_usb_cdc_tx;
static uint8_t m_lwrb_tx_raw_buffer[USB_CDC_TX_RING_BUFFER_SIZE];
void tusb_read_callback (void)
{
//	while (1)
//	{
//		uint8_t usb_ch;
//		uint32_t count = tud_cdc_read(&usb_ch, 1);
//		if (count)
//		{
//	//				DEBUG_WARN ("THERE IS DATA FROM USB \r\n");
//	//		lwrb_write (&m_ringbuffer_cli_rx, &usb_ch, 1);
//			app_cli_poll(usb_ch);
//		}
//		else
//		{
//			break;
//		}
//	}
	return;
}
uint32_t cdc_tx(const void *buffer, uint32_t size)
{
	lwrb_write(&m_ringbuffer_usb_cdc_tx, buffer, size);
	return size;
}
void usb_task (void* params)
{
	DEBUG_INFO ("USB TASK\r\n");
	tusb_init_flag = tusb_init ();
	// Create CDC task
	(void) xTaskCreateStatic(cdc_task, "cdc", 256, NULL, 1, cdc_stack, &cdc_taskdef);// pio =2

	while (1)
	{
		tud_task();
	}
}

void cdc_task(void* params)
{
	DEBUG_INFO("ENTER CDC TASK\r\n");
	lwrb_init(&m_ringbuffer_usb_cdc_tx, m_lwrb_tx_raw_buffer, USB_CDC_TX_RING_BUFFER_SIZE);
	for (;;)
	{
//	    // connected() check for DTR bit
//	    // Most but not all terminal client set this when making connection
//		tusb_read_callback();
		if (tud_cdc_connected())
		{
			if (m_cdc_debug_register == false)
			{
				m_cdc_debug_register = true;
				app_debug_register_callback_print(cdc_tx);
			}
			if (tud_cdc_available())
			{
				tusb_read_callback();
			}
		}
		else
		{
			if (m_cdc_debug_register)
			{
				m_cdc_debug_register = false;
				app_debug_unregister_callback_print(cdc_tx);
				// Flush all cdc tx buffer
				char tmp[1];
				while (lwrb_read(&m_ringbuffer_usb_cdc_tx, tmp, 1))
				{

				}
			}
		}

		char buffer[ (TUD_OPT_HIGH_SPEED ? 512 : 64)];
		uint32_t size;
		while (1)
		{
			uint32_t avai = tud_cdc_write_available();
			if (avai >= sizeof(buffer))
			{
				avai = sizeof(buffer);
			}
			size = lwrb_read(&m_ringbuffer_usb_cdc_tx, buffer, avai);
			if (size)
			{
				tud_cdc_write(buffer, size);
				tud_cdc_write_flush();
			}
			else
			{
				break;
			}
		}
//		xEventGroupSetBits(m_wdg_event_group, cdcTaskB);
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
/**********************************************************************/
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
// *********************** interrupt callback ************************//
void rs232_rx_callback (void)
{
//	uint8_t ch = getChar (USART3);
//	lwrb_write (&m_ringbuffer_host_rx, &ch, 1);
}

void rs485_rx_callback (void)
{
//	idle_detect = 5;
//	uint8_t ch =getChar(UART5);
//	lwrb_write (&m_ringbuffer_rs485_rx, &ch, 1);
////	DEBUG_ISR ("%c",ch);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
//  if (GPIO_Pin == RELAY_NO_Pin)
//  {
//	  relay_toggle_0 ++;
//	  if (relay_toggle_0 > 5)
//	  {
//		  test_res.result.relay0_ok = 1;
//		  relay_toggle_0 = 0;
//	  }
//  }
//  else if (GPIO_Pin == RELAY_NC_Pin)
//  {
//	  relay_toggle_0 ++;
//	  if (relay_toggle_1 > 5)
//	  {
//		  relay_toggle_1 = 0;
//		  test_res.result.relay1_ok = 1;
//	  }
//  }
}
//********************************************************************//
static void convert_second_to_date_time(uint32_t sec, date_time_t *t, uint8_t Calyear)
{
    uint16_t day;
    uint8_t year;
    uint16_t days_of_year;
    uint8_t leap400;
    uint8_t month;

    t->second = sec % 60;
    sec /= 60;
    t->minute = sec % 60;
    sec /= 60;
    t->hour = sec % 24;

    if (Calyear == 0)
        return;

    day = (uint16_t)(sec / 24);

    year = FIRSTYEAR % 100;                   // 0..99
    leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3); // 4, 3, 2, 1

    for (;;)
    {
        days_of_year = 365;
        if ((year & 3) == 0)
        {
            days_of_year = 366; // leap year
            if (year == 0 || year == 100 || year == 200)
            { // 100 year exception
                if (--leap400)
                { // 400 year exception
                    days_of_year = 365;
                }
            }
        }
        if (day < days_of_year)
        {
            break;
        }
        day -= days_of_year;
        year++; // 00..136 / 99..235
    }
    t->year = year + FIRSTYEAR / 100 * 100 - 2000; // + century
    if (days_of_year & 1 && day > 58)
    {          // no leap year and after 28.2.
        day++; // skip 29.2.
    }

    for (month = 1; day >= day_in_month[month - 1]; month++)
    {
        day -= day_in_month[month - 1];
    }

    t->month = month; // 1..12
    t->day = day + 1; // 1..31
}

static uint32_t convert_date_time_to_second(date_time_t *t)
{
    uint8_t i;
    uint32_t result = 0;
    uint16_t idx, year;

    year = t->year + 2000;

    /* Calculate days of years before */
    result = (uint32_t)year * 365;
    if (t->year >= 1)
    {
        result += (year + 3) / 4;
        result -= (year - 1) / 100;
        result += (year - 1) / 400;
    }

    /* Start with 2000 a.d. */
    result -= 730485UL;

    /* Make month an array index */
    idx = t->month - 1;

    /* Loop thru each month, adding the days */
    for (i = 0; i < idx; i++)
    {
        result += day_in_month[i];
    }

    /* Leap year? adjust February */
    if (year % 400 == 0 || (year % 4 == 0 && year % 100 != 0))
    {
        ;
    }
    else
    {
        if (t->month > 2)
        {
            result--;
        }
    }

    /* Add remaining days */
    result += t->day;

    /* Convert to seconds, add all the other stuff */
//    if (year < 2000)
//    {
//	    result = (result - 1) * 86400L + (uint32_t)t->hour * 3600 +
//	             (uint32_t)t->minute * 60 + t->second;
//	}
//	else
//	{
//		result = (result-2) * 86400L + (uint32_t)t->hour * 3600 +
//             (uint32_t)t->minute * 60 + t->second;
//	}
    result = (result - 1) * 86400L + (uint32_t)t->hour * 3600 +
    	             (uint32_t)t->minute * 60 + t->second;
    return result;
}

void lwip_sntp_recv_cb (uint32_t time)
{

	if (time == 0)
	{
		DEBUG_INFO ("NTP ERROR \r\n");
	}
	else
	{
//		reInitRTC()
		DEBUG_INFO (" GOT TIME : it's been %u second from 1970\r\n", time);

		time_t rawtime = time;
		struct tm ts;

		// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
		ts = *localtime(&rawtime);

		DEBUG_INFO("Time now: %02d:%02d:%02d  %02d-%02d-%d\r\n", ts.tm_hour, ts.tm_min, ts.tm_sec, ts.tm_mday, ts.tm_mon + 1, ts.tm_year + 1900);

		date_time.year = (ts.tm_year + 1900) % 2000; //year - 1900
		date_time.month = ts.tm_mon + 1;             // month, where 0 = jan
		date_time.day = ts.tm_mday;                  // day of the month
		date_time.hour = ts.tm_hour;
		date_time.minute = ts.tm_min;
		date_time.second = ts.tm_sec;

		uint32_t time_buff = convert_date_time_to_second (&date_time) + 25200;// time for gmt +7

		convert_second_to_date_time (time_buff, &date_time, 1);

		DEBUG_INFO ("TIME NOW IS: %d:%d:%d %d-%d-%d\r\n", date_time.hour, date_time.minute, date_time.second, date_time.day,date_time.month, (date_time.year + 2000));
		sTime.Hours = date_time.hour;
		sTime.Minutes = date_time.minute;
		sTime.Seconds = date_time.second;
		sDate.Year = date_time.year;
		sDate.Month = date_time.month;
		sDate.Date = date_time.day;
		reInitRTC (sTime,sDate);
		HAL_RTC_GetTime (&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate (&hrtc, &sDate, RTC_FORMAT_BIN);
		DEBUG_INFO ("GET TIME: %d: %d: %d \r\n", (uint8_t)(sTime.Hours), (uint8_t)(sTime.Minutes),(uint8_t)(sTime.Seconds));
		DEBUG_INFO ("GET date: %d: %d: %d \r\n", (uint8_t)(sDate.Date), (uint8_t)(sDate.Month),(uint8_t)(sDate.Year));
//		uint32_t timenew = convert_date_time_to_second (&date_time);
//		DEBUG_INFO ("TIME CALCULATE: %u", timenew);
	}
}

static void initialize_stnp(void)
{
    static bool sntp_start = false;
    if (sntp_start == false)
    {
        sntp_start = true;

        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
        sntp_init();
        DEBUG_INFO("Initialize stnp\r\n");
    }
}

//************************************************************************//
// ************************* TIME RTC*********************************//
void reInitRTC ( RTC_TimeTypeDef sTime, RTC_DateTypeDef sDate)
{
	  /** Initialize RTC Only
	  */
	  /* USER CODE BEGIN Check_RTC_BKUP */

	  /* USER CODE END Check_RTC_BKUP */

	  /** Initialize RTC and set the Time and Date
	  */
//	  sTime.Hours = 0x0;
//	  sTime.Minutes = 0x0;
//	  sTime.Seconds = 0x0;
//	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  DEBUG_ERROR ("CAN'T SET TIME \r\n");
	    Error_Handler();
	  }
//	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//	  sDate.Month = RTC_MONTH_JANUARY;
//	  sDate.Date = 0x12;
//	  sDate.Year = 0x0;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  DEBUG_ERROR ("CAN'T SET date \r\n");
	    Error_Handler();
	  }
}
//*****************************************************************//
/* USER CODE END Application */

