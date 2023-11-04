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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "vt100.h"
#include "src4193.h"
#include "pcm9211.h"
#include "bd34301.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define DEVICE_NAME             "MDAC220822"
#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_FW               "231104_1436"
#define DEVICE_FW_SIZE          sizeof(DEVICDSD_PCME_FW)
#define DEVICE_PCB              "00"
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_MNF              "KONVOLUCIO"
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)


#define PCM9211_DEVICE_ADDRESS  0x86
#define BD34_DEVICE_ADDRESS     0x38

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A0_USB_ISO_Pin GPIO_PIN_13
#define A0_USB_ISO_GPIO_Port GPIOC
#define TIMEBASE_OUT_Pin GPIO_PIN_14
#define TIMEBASE_OUT_GPIO_Port GPIOC
#define DAC_MUTE_COM_Pin GPIO_PIN_15
#define DAC_MUTE_COM_GPIO_Port GPIOC
#define FREQ_MEAS_LRCK_Pin GPIO_PIN_0
#define FREQ_MEAS_LRCK_GPIO_Port GPIOA
#define RESET_Pin GPIO_PIN_1
#define RESET_GPIO_Port GPIOA
#define EN_USB_ISO_Pin GPIO_PIN_2
#define EN_USB_ISO_GPIO_Port GPIOA
#define RDY_SRC_Pin GPIO_PIN_3
#define RDY_SRC_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define USART1_DIR_Pin GPIO_PIN_6
#define USART1_DIR_GPIO_Port GPIOA
#define RATIO_SRC_Pin GPIO_PIN_0
#define RATIO_SRC_GPIO_Port GPIOB
#define EN_I2S_I2C_ISO_Pin GPIO_PIN_1
#define EN_I2S_I2C_ISO_GPIO_Port GPIOB
#define RESET_SPDIF_Pin GPIO_PIN_2
#define RESET_SPDIF_GPIO_Port GPIOB
#define NON_PCM_Pin GPIO_PIN_12
#define NON_PCM_GPIO_Port GPIOB
#define ERROR_SPDIF_Pin GPIO_PIN_13
#define ERROR_SPDIF_GPIO_Port GPIOB
#define LIVE_LED_Pin GPIO_PIN_14
#define LIVE_LED_GPIO_Port GPIOB
#define PCM_DET_Pin GPIO_PIN_15
#define PCM_DET_GPIO_Port GPIOB
#define A1_USB_ISO_Pin GPIO_PIN_8
#define A1_USB_ISO_GPIO_Port GPIOA
#define A2_USB_ISO_Pin GPIO_PIN_11
#define A2_USB_ISO_GPIO_Port GPIOA
#define FREQ_MEAS_BCLK_Pin GPIO_PIN_12
#define FREQ_MEAS_BCLK_GPIO_Port GPIOA
#define EN_SPDIF_ISO_Pin GPIO_PIN_15
#define EN_SPDIF_ISO_GPIO_Port GPIOA
#define MUX_PCM_Pin GPIO_PIN_4
#define MUX_PCM_GPIO_Port GPIOB
#define DSD_PCM_USB_ISO_Pin GPIO_PIN_5
#define DSD_PCM_USB_ISO_GPIO_Port GPIOB
#define SCL_I2C1_DAC_Pin GPIO_PIN_6
#define SCL_I2C1_DAC_GPIO_Port GPIOB
#define SDA_I2C1_DAC_Pin GPIO_PIN_7
#define SDA_I2C1_DAC_GPIO_Port GPIOB
#define MCLK_SEL_ISO_Pin GPIO_PIN_8
#define MCLK_SEL_ISO_GPIO_Port GPIOB
#define MUTE_USB_ISO_Pin GPIO_PIN_9
#define MUTE_USB_ISO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
