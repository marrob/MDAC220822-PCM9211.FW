/*
 * bd34301.c
 *
 *  Created on: Feb 19, 2022
 *      Author: Margit Robert
 */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "dac_bd34.h"
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
I2C_HandleTypeDef *_I2C;

/* Private user code ---------------------------------------------------------*/

uint8_t DacBD34Init(I2C_HandleTypeDef *i2c)
{
  /*
   * RHOM BD34301
   * Max CLK: 400KHz
   * Device Address:0x38
   * MCLK: 22.5792MHz
   * Be Careful: Alwayas be signal on the PCM inputs before RAM Clear... Fiugre 32...
   */
  _I2C=i2c;


  DacBD34RegWrite(0x04, 0x02); /*Clock 1*/
  DacBD34RegWrite(0x06, 0x00); /*Clock 2*/

  DacBD34RegWrite(0x10, 0x08); /*AuidoIf1*/
  DacBD34RegWrite(0x13, 0x00); /*AuidoIf3*/

  /*** Audio Input Polarity ***/
  DacBD34RegWrite(0x17, 0x00);

  /*** Volume Transition Time ***/
  DacBD34RegWrite(0x20, 0x48);

  /*** FIR Filter ***/
  DacBD34RegWrite(0x30, 0x01);
  DacBD34RegWrite(0x31, 0x80);

  /*** Delta Sigma ***/
  DacBD34RegWrite(0x40, 0x00);

  /*** Settings ***/
  DacBD34RegWrite(0x41, 0x00);
  DacBD34RegWrite(0x42, 0x34);
  DacBD34RegWrite(0x43, 0xB8);
  DacBD34RegWrite(0x48, 0x0D);

  /*** Settings 5. & 6. ***/
  DacBD34RegWrite(0x60, 0x16);
  DacBD34RegWrite(0x61, 0x16);

  return BD34_OK;
}


HAL_StatusTypeDef DacBD34RegWrite(uint8_t address, uint8_t data)
{
  return HAL_I2C_Mem_Write(_I2C, BD34_DEV_ADDRESS, address, BD34_ADDR_SIZE, (uint8_t[]){data}, 1, 100);
}



/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/