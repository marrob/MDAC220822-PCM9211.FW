/*
 * pcm9211.c
 *
 *  Created on: Sep 19, 2023
 *      Author: marrob
 * DIT: Digital Transmitter, ez mehetne TOSLINK-re, de nem használjuk
 * DIR: Digital Receiver, ebből több csatornát is használunk
 * ADC: Az Line-in ADC-je nem fogjuk használni
 * RXIN0: RCA bement (Gábornál és az EVAL boardon is)
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "pcm9211.h"


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define PCM9211_TIMOUT_TICK       100
#define PCM9211_MEM_ADDRESS_SIZE   1

#define PCM9211_OK                 0
#define PCM9211_IO_ERROR           1

/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef *_i2c;
static uint8_t  _devAddress;
static uint32_t _errorCounter;

/* Private function prototypes -----------------------------------------------*/
static uint8_t Read(uint8_t reg);
static uint32_t ReadU32(uint8_t reg);
static uint8_t Write(uint8_t reg, uint8_t value);

/* Private user code ---------------------------------------------------------*/
uint8_t PCM9211_Init(I2C_HandleTypeDef *i2c, uint8_t address)
{
  PCM9211_Reset();
  _i2c = i2c;
  _devAddress = address;
  uint8_t readValue = 0;

  /*** IO Self Test ***/
  Write(0x2E,0xAA);
   readValue = Read(0x2E);
  if(readValue != 0xAA)
    return PCM9211_IO_ERROR;
  Write(0x2E,0x00);

  /*** Soft Reset ***/
  Write(0x40, 0x00);
  HAL_Delay(100);
  readValue = Read(0x40);
  if(readValue != 0xC0)
    return PCM9211_IO_ERROR;

  /*** Disable: ADC, DIT ***/
  Write(0x40, 0xE2);
  readValue = Read(0x40);
  if(readValue != 0xE2)
    return PCM9211_IO_ERROR;

  PCM9211_SelectSource(PCM9211_RXIN0);

  /*** Main Output Source: DIR ***/
  Write(0x6B, 0x11);

  return PCM9211_OK;
}


/*
 * 0x00: RXIN0 - SPDIF0
 * 0x01: RXIN1 - SPDIF1
 * 0x02: RXIN2 - AES/EBU
 * 0x03: RXIN3 - TOSLink
 */

void PCM9211_SelectSource(uint8_t source)
{
  /*** Eanble RXIN0(SPDIF), RXIN1(TOSlink) amplifier and select source***/
  source &= 0x0F;
  Write(0x34, source);
}

/*
0000: Out of range
0001: 8 kHz
0010: 11.025 kHz
0011: 12 kHz
0100: 16 kHz
0101: 22.05 kHz
0110: 24 kHz
0111: 32 kHz
1000: 44.1 kHz
1001: 48 kHz
1010: 64 kHz
1011: 88.2 kHz
1100: 96 kHz
1101: 128 kHz
1110: 176.4 kHz
1111: 192 kHz
*/
PCM9211_Frequencies_t PCM9211_SamplingFreq(void)
{
  uint8_t readValue = 0;
  uint8_t freq = 0;
  readValue = Read(0x38);
  if((readValue & 0x80) == 0)//Calculated
  {
    if((readValue & 0x70)==0) //DIR - Port
    {
      freq = readValue & 0x0F;
    }
  }
  return freq;
}




uint32_t PCM9211_DataInfromation()
{

  return Read(0x3A);
 // return ReadU32(0x3A);
}



static uint8_t Read(uint8_t reg)
{
  uint8_t value = 0;
  if(HAL_I2C_Mem_Read(_i2c, _devAddress, reg, PCM9211_MEM_ADDRESS_SIZE, &value, sizeof(value), PCM9211_TIMOUT_TICK)!= HAL_OK)
    _errorCounter++;
  return value;
}

static uint32_t ReadU32(uint8_t reg)
{
  uint32_t value = 0;
  uint8_t temp[4];

  if(HAL_I2C_Mem_Read(_i2c, _devAddress, reg, PCM9211_MEM_ADDRESS_SIZE, temp, sizeof(temp), PCM9211_TIMOUT_TICK)!= HAL_OK)
    _errorCounter++;
  value = *((uint32_t*)temp);
  return value;
}

static uint8_t Write(uint8_t reg, uint8_t value)
{
  if(HAL_I2C_Mem_Write(_i2c, _devAddress, reg, PCM9211_MEM_ADDRESS_SIZE, &value, sizeof(value), PCM9211_TIMOUT_TICK)!= HAL_OK)
    _errorCounter++;
  return HAL_OK;
}

void PCM9211_Reset(void)
{
  HAL_GPIO_WritePin(RESET_SPDIF_GPIO_Port, RESET_SPDIF_Pin, GPIO_PIN_SET); // igy van resetben
  HAL_Delay(100);
  HAL_GPIO_WritePin(RESET_SPDIF_GPIO_Port, RESET_SPDIF_Pin, GPIO_PIN_RESET); // reset vége
  HAL_Delay(100);
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
