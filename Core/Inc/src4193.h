/*
 * scr41.h
 *
 *  Created on: Feb 16, 2022
 *      Author: Margit Robert
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_SCR4193_H_
#define INC_SCR4193_H_
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/*** SPI: MSB First Out ***/
typedef struct _SRC4193_t
{
  uint8_t RdyFlag;
  uint8_t RatioFlag;
  union
  {
    struct
    {/*** First Byte ***/
      struct _Header
      {
        uint8_t RegAddr;  //Address of first data byet...
        uint8_t DontCare;
      }Header;
      /*** All Zero Register Address is 0x00 ***/
      uint8_t AllZeroRegister; //
      struct _SYS
      { /***  Address is 0x01 ***/
        uint8_t MODE:3;   //LSB bits
        uint8_t BYASS:1;
        uint8_t MUTE:1;
        uint8_t ZERO:1;
        uint8_t TRACK:1;
        uint8_t PDN:1;    //MSB bit
      }System;
      struct _Filter
      { /***  Address is 0x02 ***/
        uint8_t LGRP:1;   //LSB
        uint8_t DFLT:1;
        uint8_t ZERO:6;
      }Filter;
      struct _Format
      {
        /***  Address is 0x03 ***/
        uint8_t IFMT:3;   //LSB
        uint8_t ZERO:1;
        uint8_t OFMT:2;
        uint8_t OWL:2;
      }Format;
      uint8_t AttLeft;
      uint8_t AttRight;

    };
    uint8_t Data[8];
  };
}SRC41_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define SRC_OK    0
#define SRC_FAIL  1

/* Exported functions ------------------------------------------------------- */
uint8_t SRC41Init(SPI_HandleTypeDef *hspi);
uint8_t SCR41Update(SRC41_t *data);

#endif /* INC_SCR41_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
