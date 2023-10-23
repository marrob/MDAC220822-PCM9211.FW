/*
 * bd34301.h
 *
 *  Created on: Feb 19, 2022
 *      Author: Margit Robert
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_BD34301_H_
#define INC_BD34301_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/

typedef struct _BD34301_Mode{
  uint8_t Clock1;       /* 0x04 - MclkDiv[1:0] */
  uint8_t Clock2;       /* 0x06 - PhaseAdj */
  uint8_t AudioIf1;     /* 0x10 - DSD, MUTE, FMT, WLEN */
  uint8_t AudioIf3;     /* 0x13 - LrSwap */
  uint8_t DsdFilter;    /* 0x16 - DSD Filter */
  uint8_t FirFilter1;   /* 0x30 - FirAlgo */
  uint8_t FirFilter2;   /* 0x31 - FirCoef */
  uint8_t DeEmph1;      /* 0x33 - DempFs[1:0]*/
  uint8_t DeEmph2;      /* 0x34 - Demp1, Demp2*/
  uint8_t DeltaSigma;   /* 0x40 - DsSettings, DsOsr[1:0]*/
  uint8_t Settings5;    /* 0x60 - Magic*/
  uint8_t Settings6;    /* 0x61 - Maigc*/
}BD34301_Mode_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define BD34_OK     0
#define BD34_FAIL   1

#define BD34_ADDR_SIZE          0x01
#define BD34_CHIP_VERSION       0x01
#define BD34_DIGITAL_POWER      0x02
#define BD34_ANALOG_POWER       0x03
#define BD34_TIMEOUT            100


/* Exported variables ------------------------------------------------------- */
extern BD34301_Mode_t BD34301_ModeList[];



/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef BD34301_RegWrite(uint8_t address, uint8_t data);

uint8_t BD34301_Init(I2C_HandleTypeDef *i2c, uint8_t address);
void BD34301_ModeSwitching(BD34301_Mode_t *mode);
void BD34301_PrintMode(char *buffer, BD34301_Mode_t *mode);
void BD34301_RamClear(void);
void BD34301_SoftwareResetOn(void);
void BD34301_SoftwareResetOff(void);
void BD34301_DigitalPowerOn(void);
void BD34301_DigitalPowerOff(void);
void BD34301_MuteOn(void);
void BD34301_MuteOff(void);

#endif /* INC_BD34301_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
