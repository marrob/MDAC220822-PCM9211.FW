/*
 * wm8805.h
 *
 *  Created on: Feb 12, 2022
 *      Author: Margit Robert
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_WM8805_H_
#define INC_WM8805_H_
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/





/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define WM_OK     0x00
#define WM_FAIL   0x01


/* Exported functions ------------------------------------------------------- */
uint8_t WmInit(void);
uint8_t WmSelfTest(void);
uint8_t WmSelectInput(uint8_t input);
void WmDebug(void);
void WmTask(void);

#endif /* INC_WM8805_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
