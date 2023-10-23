/*
 * bd34301.c
 *
 *  Created on: Feb 19, 2022
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "bd34301.h"
#include <stdio.h>


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static I2C_HandleTypeDef *_i2c;
static uint8_t  _devAddress;
static uint32_t _errorCounter;

BD34301_Mode_t BD34301_ModeList[] = {
  //Clock1 | Clock2 | AudioIf1 | AudioIf3 | DsdFilter | FirFilter1 | FirFilter2 | DeEmph1 | DeEmph2 | DeltaSigma | Settigns5 | Settings6|
  {  0x03,    0x00,     0x0B,      0x00,        0x00,       0x01,      0x80,       0x00,     0x00,       0x02,        0x16,       0x16 }, //DAC_PCM_32_0KHZ
  {  0x02,    0x00,     0x0B,      0x00,        0x00,       0x01,      0x80,       0x00,     0x00,       0x02,        0x16,       0x16 }, //DAC_PCM_44_1KHZ
  {  0x02,    0x00,     0x0B,      0x00,        0x00,       0x01,      0x80,       0x00,     0x00,       0x02,        0x16,       0x16 }, //DAC_PCM_48_0KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x02,      0x01,       0x00,     0x00,       0x11,        0x16,       0x16 }, //DAC_PCM_88_2KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x02,      0x01,       0x00,     0x00,       0x11,        0x16,       0x16 }, //DAC_PCM_96_0KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x04,      0x02,       0x00,     0x00,       0x11,        0x16,       0x16 }, //DAC_PCM_176_4KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x04,      0x02,       0x00,     0x00,       0x11,        0x16,       0x16 }, //DAC_PCM_192_KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x08,      0x80,       0x00,     0x00,       0x11,        0x16,       0x16 }, //DAC_PCM_362_8KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x08,      0x80,       0x00,     0x00,       0x11,        0x16,       0x16 }, //DAC_PCM_384_0KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x08,      0x80,       0x00,     0x00,       0x01,        0x16,       0x16 }, //DAC_PCM_705_6KHZ
  {  0x00,    0x00,     0x0B,      0x00,        0x00,       0x08,      0x80,       0x00,     0x00,       0x01,        0x16,       0x16 }, //DAC_PCM_768_0KHZ
  {  0x00,    0x00,     0x8B,      0x00,        0x02,       0x00,      0x00,       0x00,     0x00,       0x02,        0x9E,       0x1E }, //DAC_DSD_64
  {  0x00,    0x00,     0x8B,      0x00,        0x01,       0x00,      0x00,       0x00,     0x00,       0x02,        0x9E,       0x1E }, //DAC_DSD_128
  {  0x00,    0x00,     0x8B,      0x00,        0x00,       0x00,      0x00,       0x00,     0x00,       0x02,        0x9E,       0x1E }, //DAC_DSD_256
  {  0x00,    0x00,     0x8B,      0x00,        0x00,       0x00,      0x00,       0x00,     0x00,       0x02,        0x9E,       0x1E }, //DAC_DSD_512
};


/* Private function prototypes -----------------------------------------------*/
void BD34301_PowerOnSequence(void);

/* Private user code ---------------------------------------------------------*/

uint8_t BD34301_Init(I2C_HandleTypeDef *i2c, uint8_t address)
{
  /*
   * RHOM BD34301
   * Max CLK: 400KHz
   * Device Address:0x38
   * MCLK: 22.5792MHz
   * Be Careful: Alwayas be signal on the PCM inputs before RAM Clear... Fiugre 32...
   */
  _i2c=i2c;
  _devAddress = address;


  BD34301_RegWrite(0x04, 0x02); /*Clock 1*/
  BD34301_RegWrite(0x06, 0x00); /*Clock 2*/

  BD34301_RegWrite(0x10, 0x08); /*AuidoIf1*/
  BD34301_RegWrite(0x13, 0x00); /*AuidoIf3*/

  /*** Audio Input Polarity ***/
  BD34301_RegWrite(0x17, 0x00);

  /*** Volume Transition Time ***/
  BD34301_RegWrite(0x20, 0x48);

  /*** FIR Filter ***/
  BD34301_RegWrite(0x30, 0x01);
  BD34301_RegWrite(0x31, 0x80);

  /*** Delta Sigma ***/
  BD34301_RegWrite(0x40, 0x00);

  /*** Settings ***/
  BD34301_RegWrite(0x41, 0x00);
  BD34301_RegWrite(0x42, 0x34);
  BD34301_RegWrite(0x43, 0xB8);
  BD34301_RegWrite(0x48, 0x0D);

  /*** Settings 5. & 6. ***/
  BD34301_RegWrite(0x60, 0x16);
  BD34301_RegWrite(0x61, 0x16);


  BD34301_PowerOnSequence();

  return BD34_OK;
}

void BD34301_PowerOnSequence(void)
{
  BD34301_SoftwareResetOff();
  BD34301_DigitalPowerOn();

  /*** Pop Noise Prevention ***/
  BD34301_RegWrite(0xD0, 0x6A);
  BD34301_RegWrite(0xD3, 0x10);
  BD34301_RegWrite(0xD3, 0x00);
  BD34301_RegWrite(0xD0, 0x00);

  /*** Analog Power On ***/
  BD34301_RegWrite(0x03, 0x01);

  BD34301_RamClear();

  BD34301_MuteOff();

}

void BD34301_RamClear(void)
{
  BD34301_RegWrite(0x2F, 0x80);
  BD34301_RegWrite(0x2F, 0x00);
}

void BD34301_SoftwareResetOn(void)
{
  BD34301_RegWrite(0x00, 0x00);
}

void BD34301_SoftwareResetOff(void)
{
  BD34301_RegWrite(0x00, 0x01);
}

void BD34301_DigitalPowerOn(void)
{
  BD34301_RegWrite(0x02, 0x01);
}

void BD34301_DigitalPowerOff(void)
{
  BD34301_RegWrite(0x02, 0x00);
}

void BD34301_MuteOn(void)
{
  BD34301_RegWrite(0x2A, 0x00);
}

void BD34301_MuteOff(void)
{
  BD34301_RegWrite(0x2A, 0x03);
}

void BD34301_ModeSwitching(BD34301_Mode_t *mode)
{
  BD34301_RegWrite(0x04, mode->Clock1);
  BD34301_RegWrite(0x06, mode->Clock2);
  BD34301_RegWrite(0x10, mode->AudioIf1);
  BD34301_RegWrite(0x13, mode->AudioIf3);
  BD34301_RegWrite(0x16, mode->DsdFilter);
  BD34301_RegWrite(0x30, mode->FirFilter1);
  BD34301_RegWrite(0x31, mode->FirFilter2);
  BD34301_RegWrite(0x33, mode->DeEmph1);
  BD34301_RegWrite(0x34, mode->DeEmph1);
  BD34301_RegWrite(0x40, mode->DeltaSigma);
  BD34301_RegWrite(0x60, mode->Settings5);
  BD34301_RegWrite(0x61, mode->Settings6);
}

void BD34301_PrintMode(char *buffer, BD34301_Mode_t *mode)
{
  sprintf(buffer, "0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
      mode->Clock1,
      mode->Clock2,
      mode->AudioIf1,
      mode->AudioIf3,
      mode->DsdFilter,
      mode->FirFilter1,
      mode->FirFilter2,
      mode->DeEmph1,
      mode->DeEmph2,
      mode->DeltaSigma,
      mode->Settings5,
      mode->Settings6
  );
}

HAL_StatusTypeDef BD34301_RegWrite(uint8_t address, uint8_t data)
{
  HAL_StatusTypeDef status = 0;
  if((status = HAL_I2C_Mem_Write(_i2c, _devAddress, address, BD34_ADDR_SIZE, (uint8_t[]){data}, 1, BD34_TIMEOUT))!= HAL_OK)
    _errorCounter++;
  return status;
}



/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
