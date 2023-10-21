/*
 * pcm9211.h
 *
 *  Created on: Sep 19, 2023
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_PCM9211_H_
#define INC_PCM9211_H_
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define PCM9211_RXIN0   0x00
#define PCM9211_RXIN1   0x01
#define PCM9211_RXIN2   0x02
#define PCM9211_RXIN3   0x03

typedef enum _PCM9211_Frequencies_t
{
  PCM9211_OUT_OF_RANGE = 0x00,
  PCM9211_8KHz,         //0x01
  PCM9211_11_025KHz,    //0x02
  PCM9211_12KHz,        //0x03
  PCM9211_16KHz,        //0x04
  PCM9211_22_05KHz,     //0x05
  PCM9211_24KHz,        //0x06
  PCM9211_32KHz,        //0x07
  PCM9211_44_1KHz,      //0x08
  PCM9211_48KHz,        //0x09
  PCM9211_64KHz,        //0x0A
  PCM9211_88_2KHz,      //0x0B
  PCM9211_96KHz,        //0x0C
  PCM9211_128KHz,       //0x0D
  PCM9211_176_4KHz,     //0x0E
  PCM9211_192KHz        //0x0F
}PCM9211_Frequencies_t;

/* Exported functions ------------------------------------------------------- */
uint8_t PCM9211_Init(I2C_HandleTypeDef *i2c, uint8_t address);
void PCM9211_SelectSource(uint8_t source);
uint32_t PCM9211_DataInfromation();
uint8_t PCM9211_SamplingFreq(void);
void PCM9211_Reset(void);



#endif /* INC_PCM9211_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
