/*
 * scr41.c
 *
 *  Created on: Feb 16, 2022
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "src4193.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/


SPI_HandleTypeDef *_Spi;

uint8_t SRC41Init(SPI_HandleTypeDef *hspi)
{
  _Spi = hspi;
  return SRC_OK;
}

uint8_t SCR41Update(SRC41_t* src)
{
  uint8_t size = sizeof(src->Data);

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit( _Spi, src->Data, size, 100);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
  return SRC_OK;
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
