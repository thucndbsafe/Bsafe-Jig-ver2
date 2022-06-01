/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

#include "stm32f2xx_ll_dma.h"
#include "stm32f2xx_ll_usart.h"
#include "stm32f2xx_ll_rcc.h"
#include "stm32f2xx_ll_bus.h"
#include "stm32f2xx_ll_cortex.h"
#include "stm32f2xx_ll_system.h"
#include "stm32f2xx_ll_utils.h"
#include "stm32f2xx_ll_pwr.h"
#include "stm32f2xx_ll_gpio.h"

#include "stm32f2xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_G_Pin GPIO_PIN_2
#define LED2_G_GPIO_Port GPIOE
#define ESP_EN_Pin GPIO_PIN_13
#define ESP_EN_GPIO_Port GPIOC
#define ESP_IO0_Pin GPIO_PIN_14
#define ESP_IO0_GPIO_Port GPIOC
#define LED1_R_Pin GPIO_PIN_15
#define LED1_R_GPIO_Port GPIOC
#define LED2_R_Pin GPIO_PIN_0
#define LED2_R_GPIO_Port GPIOF
#define GD32_SDIO_Pin GPIO_PIN_3
#define GD32_SDIO_GPIO_Port GPIOA
#define BT_IN_Pin GPIO_PIN_4
#define BT_IN_GPIO_Port GPIOA
#define GD32_CLK_Pin GPIO_PIN_5
#define GD32_CLK_GPIO_Port GPIOA
#define ADC_1V8_Pin GPIO_PIN_11
#define ADC_1V8_GPIO_Port GPIOF
#define ADC_VBATRF_Pin GPIO_PIN_15
#define ADC_VBATRF_GPIO_Port GPIOF
#define IN_LR4_Pin GPIO_PIN_0
#define IN_LR4_GPIO_Port GPIOG
#define IN_LB3_Pin GPIO_PIN_1
#define IN_LB3_GPIO_Port GPIOG
#define LED1_G_Pin GPIO_PIN_10
#define LED1_G_GPIO_Port GPIOE
#define STATUS_Pin GPIO_PIN_13
#define STATUS_GPIO_Port GPIOE
#define EN_STWD_Pin GPIO_PIN_14
#define EN_STWD_GPIO_Port GPIOE
#define ALARM_IN_Pin GPIO_PIN_10
#define ALARM_IN_GPIO_Port GPIOB
#define BOTTON__Pin GPIO_PIN_11
#define BOTTON__GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define RELAY_NC_Pin GPIO_PIN_8
#define RELAY_NC_GPIO_Port GPIOD
#define RELAY_NO_Pin GPIO_PIN_9
#define RELAY_NO_GPIO_Port GPIOD
#define PRE_PWR__Pin GPIO_PIN_10
#define PRE_PWR__GPIO_Port GPIOD
#define MAIN_PW_Pin GPIO_PIN_11
#define MAIN_PW_GPIO_Port GPIOD
#define MODE1_Pin GPIO_PIN_12
#define MODE1_GPIO_Port GPIOD
#define MODE2_Pin GPIO_PIN_13
#define MODE2_GPIO_Port GPIOD
#define IN_LR3_Pin GPIO_PIN_2
#define IN_LR3_GPIO_Port GPIOG
#define IN_LB2_Pin GPIO_PIN_3
#define IN_LB2_GPIO_Port GPIOG
#define IN_LR2_Pin GPIO_PIN_4
#define IN_LR2_GPIO_Port GPIOG
#define IN_LB1_Pin GPIO_PIN_5
#define IN_LB1_GPIO_Port GPIOG
#define IN_LR1_Pin GPIO_PIN_6
#define IN_LR1_GPIO_Port GPIOG
#define IN_LB4_Pin GPIO_PIN_7
#define IN_LB4_GPIO_Port GPIOG
#define IN_LB5_Pin GPIO_PIN_8
#define IN_LB5_GPIO_Port GPIOG
#define IN_LB6_Pin GPIO_PIN_6
#define IN_LB6_GPIO_Port GPIOC
#define IN_LR6_Pin GPIO_PIN_7
#define IN_LR6_GPIO_Port GPIOC
#define GSM_PWRKEY_Pin GPIO_PIN_8
#define GSM_PWRKEY_GPIO_Port GPIOC
#define GSM_RESET_Pin GPIO_PIN_9
#define GSM_RESET_GPIO_Port GPIOC
#define IN_LR5_Pin GPIO_PIN_8
#define IN_LR5_GPIO_Port GPIOA
#define GMS_RX_Pin GPIO_PIN_9
#define GMS_RX_GPIO_Port GPIOA
#define GMS_TX_Pin GPIO_PIN_10
#define GMS_TX_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define TX3_Pin GPIO_PIN_10
#define TX3_GPIO_Port GPIOC
#define RX3_Pin GPIO_PIN_11
#define RX3_GPIO_Port GPIOC
#define TX5_Pin GPIO_PIN_12
#define TX5_GPIO_Port GPIOC
#define RS485_Pin GPIO_PIN_0
#define RS485_GPIO_Port GPIOD
#define RX5_Pin GPIO_PIN_2
#define RX5_GPIO_Port GPIOD
#define BUZZ_Pin GPIO_PIN_3
#define BUZZ_GPIO_Port GPIOD
#define ESP_RX_Pin GPIO_PIN_5
#define ESP_RX_GPIO_Port GPIOD
#define ESP_TX_Pin GPIO_PIN_6
#define ESP_TX_GPIO_Port GPIOD
#define W_ENET_RST_Pin GPIO_PIN_12
#define W_ENET_RST_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
