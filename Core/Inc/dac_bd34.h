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

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define BD34_OK     0
#define BD34_FAIL   1

#define BD34_ADDR_SIZE          0x01
#define BD34_CHIP_VERSION       0x01
#define BD34_DIGITAL_POWER      0x02
#define BD34_ANALOG_POWER       0x03
#define BD34_TIMEOUT            100

HAL_StatusTypeDef DacBD34RegWrite(uint8_t address, uint8_t data);

/* Exported functions ------------------------------------------------------- */

uint8_t DacBD34Init(I2C_HandleTypeDef *i2c, uint8_t address);
#endif /* INC_BD34301_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
