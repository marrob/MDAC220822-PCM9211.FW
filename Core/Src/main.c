/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  *221121_0841:
  * User Manual:
  * - A route-ot a felhasználó állitja
  * - A DAC paramétereit a routról jövő jel alapján a uC határozza meg... (többnyire frekiméréssel)
  *
  * 230430_1732:
  * - A frekvencia mérés átkerült az SRC elé, igy közvetett módon lehet meghatározni a DacAudioFormat-ot.
  * - A DSD - PCM közötti váltások problémásak, valószinüleg az XMOS jelzi rosszul a váltásokat
  * - A robothang továbbra is probléma...
  * - Bizonyos körülmények között elnémul a DAC, a paraméterek ujraküldése újra felébreszti.
  * - A USB-DSD ben eléletileg meg kellene fordulnia a csatornáknak... de mégsem történik meg...
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include "LiveLed.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _Xtatus_t{
  XMOS_UNKNOWN = 0xFF,
  XMOS_PCM_44_1KHZ = 0x17,
  XMOS_PCM_48_0KHZ = 0x16,
  XMOS_PCM_88_2KHZ = 0x15,
  XMOS_PCM_96_0KHZ = 0x14,
  XMOS_PCM_176_4KHZ = 0x13,
  XMOS_PCM_192_KHZ = 0x12,
  XMOS_PCM_352_8KHZ = 0x11,
  XMOS_PCM_384_KHZ = 0x10,
  XMOS_DSD_64 = 0x02,
  XMOS_DSD_128 = 0x00,
  XMOS_DSD_256 = 0x04
}XmosStatus_t;

typedef enum _DacAudioFormat_t{
  DAC_PCM_32_0KHZ = 0,
  DAC_PCM_44_1KHZ,  //1
  DAC_PCM_48_0KHZ,  //2
  DAC_PCM_88_2KHZ,  //3
  DAC_PCM_96_0KHZ,  //4
  DAC_PCM_176_4KHZ, //5
  DAC_PCM_192_KHZ,  //6
  DAC_PCM_352_8KHZ, //7
  DAC_PCM_384_0KHZ, //8
  DAC_PCM_705_6KHZ, //9
  DAC_PCM_768_0KHZ, //10
  DAC_DSD_64,
  DAC_DSD_128,
  DAC_DSD_256,
  DAC_DSD_512,
}DacAudioFormat_t;

typedef enum _Route_t
{
  ROUTE_NONE_DAC = 0,
  ROUTE_MUTE_DAC,
  ROUTE_USB_DAC,
  ROUTE_HDMI_DAC,
  ROUTE_BNC_DAC,
  ROUTE_RCA_DAC,
  ROUTE_XLR_DAC,
}Route_t;

/* 1->176.4, 2->44.1KHz, 3->88.2KHz*/

typedef enum _MasterClock_t
{
  CLK_22_5792MHZ,
  CLK_24_575MHZ
}MasterClocks_t;


/*
 * FIGYELEM!
 * AudioTypes_t és AudioFreqArray-nak fedniük kell egymást!
 */
typedef enum _AudioTypes
{
  AUDIO_PCM_32_0KHZ,    //0
  AUDIO_PCM_44_1KHZ,    //1
  AUDIO_PCM_48_0KHZ,    //2
  AUDIO_PCM_88_2KHZ,    //3
  AUDIO_PCM_96_0KHZ,    //4
  AUDIO_PCM_176_4KHZ,   //5
  AUDIO_PCM_192_KHZ,    //6
  AUDIO_PCM_352_8KHZ,   //7
  AUDIO_PCM_384_0KHZ,
  AUDIO_PCM_705_6KHZ,
  AUDIO_DSD_64,
  AUDIO_DSD_128,
  AUDIO_DSD_256,
  AUDIO_DSD_512,
  AUDIO_UNKNOWN,
}AudioTypes_t;

uint32_t AudioFreqArray[] ={
  32000,
  44100,  // -> PCM -> LRCK jel
  48000,  // -> PCM -> LRCK jel
  88200,  // -> PCM -> LRCK jel
  96000,  // -> PCM -> LRCK jel
  176400,
  192000,
  352800,
  384000,
  705600,  // -> PCM -> LRCK jel
  2822400, // -> DSD64 -> BCLK jel
  5644800, // -> DSD128 -> BCLK jel
  11289600,// -> DSD256 -> BCLK jel
};

typedef enum _DebugState_Type
{
  SDBG_IDLE,             // 00
  SDBG_MAKE_HARDFAULT,   // 01
  SDBG_HARD_RESET,       // 02
  SDBG_DAC_MUTE_ON,      // 03
  SDBG_DAC_MUTE_OFF,     // 04
  SDBG_DAC_RECONFIG,     // 05
  SDBG_LAST
}DebugState_t;


typedef struct _AppTypeDef
{
  struct _Meas
  {
    __IO uint32_t FreqLRCK_MHz;
    __IO uint32_t FreqBCLK_MHz;
  }Meas;

  struct _AudiType
  {
    AudioTypes_t Pre;
    AudioTypes_t Curr;
  }AudioType;

  MasterClocks_t MasterClock;

  SRC41_t SRC;

  struct _SrcConfig
  {
    uint8_t Pre;
    uint8_t Curr;
  }SrcConfig;


  struct _XStatus
  {
    XmosStatus_t Pre;
    XmosStatus_t Curr;
  }XmosStatus;

  struct _Route
  {
    Route_t Pre;
    Route_t Curr;
  }Route;

  struct _Volume
  {
    uint32_t Pre;
    uint32_t Curr;
  }Volume;

  uint32_t UpTimeSec;

  uint8_t MuteByUser;


  DacAudioFormat_t DacAudioFormat;
  DacAudioFormat_t DacAudioFormatToBeSet;

  struct _CustomDacConfig{
    uint32_t Pre;
    uint32_t Curr;
  }CustomDacConfig;


  struct _Diag
  {
    uint8_t  WakeUpFromWdtReset;
    uint32_t RS485ResponseCnt;
    uint32_t RS485RequestCnt;
    uint32_t RS485UnknwonCnt;
    uint32_t RS485NotMyCmdCnt;
    uint32_t UartErrorCnt;
    uint32_t UartDmaDataEmptyErrorCallbackCnt;
    uint32_t AuidoTypeCorrectionCnt;
    uint32_t XmosFormatUnknownCnt;
    uint32_t XmosStatusChangedCnt;
    uint8_t XmosMuteSignaledCnt;
    uint8_t DacReConfgiurationCnt;
    uint32_t SpdifAuidoTypeChangedCnt;
    PCM9211_Frequencies_t PCM9211SamplingFreq;
  }Diag;

  DebugState_t DebugState;
}Device_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*** RS485 ***/
#define RS485_TX_HOLD_MS      1
#define RS485_CMD_LENGTH      35
#define RS485_ARG1_LENGTH     35
#define RS485_ARG2_LENGTH     35

#define UART_BUFFER_SIZE       128
#define UART_DMA_BUFFER_SIZE   128

/*** Address ***/
#define CLIENT_TX_ADDR        0x30
#define CLIENT_RX_ADDR        0x03


/*** FrMeter ***/
#define FRMETER_TIM_TIMEBASE      TIM3
#define FRMETER_TIM_LRCK_COUNTER  TIM2
#define FRMETER_TIM_BCLK_COUNTER  TIM1

#define FrMeterTimebaseItEnable()     __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE)
#define FreqMeterTimebaseStart()      __HAL_TIM_ENABLE(&htim3)
#define FreqMeterTimebaseValue        FRMETER_TIM_TIMEBASE->CNT

#define FreqMeterLrckCoutnerStart()    __HAL_TIM_ENABLE(&htim2)
#define FreqMeterLrckCounterValue     FRMETER_TIM_LRCK_COUNTER->CNT

#define FreqMeterBclkCoutnerStart()    __HAL_TIM_ENABLE(&htim1)
#define FreqMeterBclkCounterValue     FRMETER_TIM_BCLK_COUNTER->CNT


/*** DIO ***/
#define DO_EN_I2S_I2C_ISO         ((uint8_t)1<<1)
#define DO_EN_USB_ISO             ((uint8_t)1<<2)
#define DO_EN_SPDIF_ISO           ((uint8_t)1<<3)
#define DO_MUX_PCM                ((uint8_t)1<<4)

#define DI_A0_USB                 ((uint8_t)1<<0)
#define DI_A1_USB                 ((uint8_t)1<<1)
#define DI_A2_USB                 ((uint8_t)1<<2)
#define DI_A3_USB                 ((uint8_t)1<<3)
#define DI_DSD_PCM_USB            ((uint8_t)1<<4)
#define DI_XMOS_MUTE              ((uint8_t)1<<5)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

Device_t Device;
LiveLED_HnadleTypeDef hLiveLed;

/*** RS485 ***/
char    UartRxBuffer[UART_BUFFER_SIZE];
char    UartDmaBuffer[UART_DMA_BUFFER_SIZE];
char    UartTxBuffer[UART_BUFFER_SIZE];
__IO uint8_t UartRxBufferPtr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/*** LiveLed ***/
void LiveLedOff(void);
void LiveLedOn(void);

/*** FrMeter ***/
void FrMeterStart(void);
void FreqMeasSourceBCLK(void);
void FreqMeasSourceLRCK(void);
AudioTypes_t GetAudioType();
void SetMasterClock(MasterClocks_t clk);

/*** UART/RS485 ***/
char* UartParser(char *line);
void RS485DirRx(void);
void RS485DirTx(void);
void UartTxTask(void);
void Reset(void);
XmosStatus_t ReadXmosStaus(void);
uint8_t XmosIsMute(void);
void SetRoute (Route_t route);

/*** Tasks ***/
void UpTimeTask(void);

void RelayMuteOff(void);
void RelayMuteOn(void);

void DebugTask(DebugState_t dbg);

DacAudioFormat_t SrcAudioFormatCorrection(DacAudioFormat_t currAudioFormat, uint8_t srcEenabled, uint8_t srcMode);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /*** Check if the system has resumed from WWDG reset ***/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    Device.Diag.WakeUpFromWdtReset = 1;
  }
  /*** Reset ***/
  Reset();

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

  /*** FrMeter ***/
  FrMeterStart();

  PCM9211_Init(&hi2c1, PCM9211_DEVICE_ADDRESS);
  BD34301_Init(&hi2c1, BD34_DEVICE_ADDRESS);

  /*** SRC ***/
  /*
   *  Van egy SDIN-n bemente
   *  Van input és Output port
   *
   *  OUTPUT MASTER Módban kell üzemelnie
   *  OUTPUT Master módban
   *    - a INPUT PORT-> BCKI pin5 - LRCKI pin 6 bemenet
   *    - a OUTPUT PORT -> BCKO és LRCKO - kimenet az fs(LRCK) 44.1KHz = 24.575M/512(MODE:3)
   *
   * A RATIO magas ha OUTPUT > INPUT: 44.1KHz és fs(amit RCKI-ből jön) az 88.2KHz
   *
   *
   *  A bementi freki RCKI = 22.5792/24.575M
   *
   *  MODE: 1 -> OUTPORT RCKI = 128*fs -> 176.4KHz
   *  MODE: 2 -> OUTPORT RCKI = 512*fs -> 44.1KHz
   *  MODE: 3 -> OUTPORT RCKI = 256*fs -> 88.2KHz
   *
   *
   */
  Device.SRC.System.PDN = 1;    /* When 1 device is ON */
  Device.SRC.System.BYASS = 0;  /* When 0 bypass is OFF */
  Device.SRC.System.MODE = 1;   /* 1->176.4, 2->44.1KHz, 3->88.2KHz*/
  Device.SRC.System.MUTE = 0;   /* When 0 mute is disabled */
  Device.SRC.Format.OFMT = 1;   /* I2S */
  Device.SRC.Format.IFMT = 1;   /* I2S */
  Device.SRC.Format.OWL = 0;    /*24bit*/

  SRC41Init(&hspi1);
  SCR41Update(&Device.SRC);

  /*** Defualt - OFF ***/
  Device.Route.Pre = ROUTE_NONE_DAC;
  Device.Route.Curr = ROUTE_MUTE_DAC;
  Device.DacAudioFormat = DAC_PCM_44_1KHZ;
  Device.MasterClock = CLK_22_5792MHZ;
  Device.XmosStatus.Pre = XMOS_UNKNOWN;
  Device.Volume.Curr = 100;
  Device.MuteByUser = 0;

  HAL_GPIO_WritePin(EN_I2S_I2C_ISO_GPIO_Port, EN_I2S_I2C_ISO_Pin, GPIO_PIN_SET); //HDMI I2C Off
  HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // USB Input Off
  HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); // SPDIF Input Off
  HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin, GPIO_PIN_SET); //SRC Off - SRC Bypass ON
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_RESET); // Mute Off


#if WAKEUP
  Device.Route.Pre = ROUTE_NONE_DAC;
  Device.Route.Curr = ROUTE_USB_DAC;
  Device.DacAudioFormat = DAC_PCM_44_1KHZ;
  Device.MasterClock = CLK_22_5792MHZ;
  Device.XmosStatus.Pre = XMOS_UNKNOWN;
  Device.Volume.Curr = 100;
  /*** Nincs némitva és USB-XMOS a bement van kiválaszva SRC bypass ***/
  HAL_GPIO_WritePin(EN_I2S_I2C_ISO_GPIO_Port, EN_I2S_I2C_ISO_Pin, GPIO_PIN_SET); //HDMI I2C Off
  HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // USB Input Off
  HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); // SPDIF Input Off
  HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_SET); //U121
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_SET); // Mute Off
  HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin, GPIO_PIN_SET); //HW SRC Bypass ON
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    //HAL_WWDG_Refresh(&hwwdg);

    // 22.5792MHz - 24.575MHz
    //         LRCK
    // 352.8KHz -   384KHz    |
    // 176.4KHz -   192KHz    |
    // 88.2KHz  -   96KHz     |
    // 44.1KHz  -   48.0KHz   |   64xLRCK

    /*
     * - Az XLR, HDMI, RCA-ról érkező jelek vezérlik a DAC-ot
     * - A LRCK-BCK mért frekvenciák alapján meghatározom audió jel tipusát pl: AUDIO_PCM_44_1KHZ
     * - Ha pl AUDIO_PCM_44_1KHZ - jön akkor 22.5792MHz a Master Clock kell neki
     * - Ha DSD jön az nem mehet keresztül a SRC, a routot kell változtatni (HDMI vagy XMOS)-n úgy hogy ne menejen keresztül rajta
     * - Ha nem USB-röl jön a jel akkor kiválasszam az OP
     **/

    static uint8_t flag;
    static uint32_t timestamp;

    Device.AudioType.Curr = GetAudioType();
    if( Device.Route.Curr == ROUTE_HDMI_DAC ||
        Device.Route.Curr == ROUTE_BNC_DAC ||
        Device.Route.Curr == ROUTE_RCA_DAC ||
        Device.Route.Curr == ROUTE_XLR_DAC)
    {
      if(Device.AudioType.Pre != Device.AudioType.Curr)
      {
        if(flag == 0)
        {
          RelayMuteOn();
          BD34301_MuteOn();
          Device.Diag.SpdifAuidoTypeChangedCnt++;
          flag = 1;
          timestamp = HAL_GetTick();
          Device.AudioType.Pre = AUDIO_UNKNOWN; //ez kikényszerití a némítás-visszakapcsolást, abban az esetben is ha pattanás/rövid hiba törétn a stream-ben
        }
        if(flag == 1)
        {
          if(HAL_GetTick() - timestamp > 500)
          {
            flag = 0;
            switch(Device.AudioType.Curr)
            {
               case AUDIO_PCM_32_0KHZ:{
                 Device.DacAudioFormat = DAC_PCM_32_0KHZ;
                 Device.MasterClock = CLK_22_5792MHZ;
                 break;
               }
               case AUDIO_PCM_44_1KHZ:{
                 Device.DacAudioFormat = DAC_PCM_44_1KHZ;
                 Device.MasterClock = CLK_22_5792MHZ;
                 break;
               }
               case AUDIO_PCM_48_0KHZ:{
                 Device.DacAudioFormat = DAC_PCM_48_0KHZ;
                 Device.MasterClock = CLK_24_575MHZ;
                 break;
               }
               case AUDIO_PCM_88_2KHZ:{
                 Device.DacAudioFormat = DAC_PCM_88_2KHZ;
                 Device.MasterClock = CLK_22_5792MHZ;
                 break;
               }
               case AUDIO_PCM_96_0KHZ:{
                 Device.DacAudioFormat = DAC_PCM_96_0KHZ;
                 Device.MasterClock = CLK_24_575MHZ;
                 break;
               }
               case AUDIO_PCM_176_4KHZ:{
                 Device.DacAudioFormat = DAC_PCM_176_4KHZ;
                 Device.MasterClock = CLK_22_5792MHZ;
                 break;
               }
               case AUDIO_PCM_192_KHZ:{
                 Device.DacAudioFormat = DAC_PCM_192_KHZ;
                 Device.MasterClock = CLK_24_575MHZ;
                 break;
               }
               case AUDIO_PCM_352_8KHZ:{
                   Device.DacAudioFormat = DAC_PCM_352_8KHZ;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
               }
               case AUDIO_PCM_384_0KHZ:{
                   Device.DacAudioFormat = DAC_PCM_384_0KHZ;
                   Device.MasterClock = CLK_24_575MHZ;
                 break;
               }
               case AUDIO_PCM_705_6KHZ:{
                   Device.DacAudioFormat = DAC_PCM_705_6KHZ;
                   Device.MasterClock = CLK_24_575MHZ;
                   break;
               }
               case AUDIO_DSD_64:{
                   Device.DacAudioFormat = DAC_DSD_64;
                   Device.MasterClock = CLK_22_5792MHZ;
                   break;
                 }
               case AUDIO_DSD_128:{
                 Device.DacAudioFormat = DAC_DSD_128;
                 Device.MasterClock = CLK_22_5792MHZ;
                 break;
               }
               case AUDIO_DSD_256:{
                 Device.DacAudioFormat = DAC_DSD_256;
                 Device.MasterClock = CLK_22_5792MHZ;
                 break;
               }
               case AUDIO_DSD_512:
               {
                 Device.DacAudioFormat = DAC_DSD_512;
                 Device.MasterClock = CLK_22_5792MHZ;
                 break;
               }
               case AUDIO_UNKNOWN: {
                 break;
               }
            }

            BD34301_DigitalPowerOff();
            BD34301_SoftwareResetOn();

            Device.DacAudioFormatToBeSet = SrcAudioFormatCorrection(Device.DacAudioFormat, Device.SrcConfig.Curr & 0x80, Device.SRC.System.MODE );
            Device.Diag.DacReConfgiurationCnt++;
            SetMasterClock(Device.MasterClock);
            DelayMs(15); //Kritikus pl 88.2 és 96 váltás között
            BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormat]);

            BD34301_SoftwareResetOff();
            BD34301_DigitalPowerOn();
            BD34301_RamClear(); //Kritkus, nem szól a PCM ha nincs
            RelayMuteOff();
            BD34301_MuteOff();
            Device.AudioType.Pre = Device.AudioType.Curr;
          }
        }
      }
    }

    /*
     * Az USB XMOS statuszta mondja meg mit csináljon a DAC
     */
    Device.XmosStatus.Curr = ReadXmosStaus();
    if(Device.Route.Curr == ROUTE_USB_DAC)
    {
      if(Device.Route.Pre != ROUTE_USB_DAC)
      {
        BD34301_MuteOn();
      }

      /*
       * Taktikai DAC némitás, az új DacAudioFormat visszakpcsolja
       * Ha az USB-n jön digitális jel, ott már nem tudok ilyen gyorsan némitani
       */
      if(Device.XmosStatus.Pre != Device.XmosStatus.Curr)
      {
        Device.Diag.XmosStatusChangedCnt++;
        switch(Device.XmosStatus.Curr)
        {
          case XMOS_PCM_44_1KHZ:{
              Device.DacAudioFormat = DAC_PCM_44_1KHZ;
              Device.MasterClock = CLK_22_5792MHZ;
              break;
            }
          case XMOS_PCM_48_0KHZ:{
            Device.DacAudioFormat = DAC_PCM_48_0KHZ;
            Device.MasterClock = CLK_24_575MHZ;
            break;
          }
          case XMOS_PCM_88_2KHZ:{
            Device.DacAudioFormat = DAC_PCM_88_2KHZ;
            Device.MasterClock = CLK_22_5792MHZ;
            break;
          }
          case XMOS_PCM_96_0KHZ:{
            Device.DacAudioFormat = DAC_PCM_96_0KHZ;
            Device.MasterClock = CLK_24_575MHZ;
            break;
          }
          case XMOS_PCM_176_4KHZ:{
            Device.DacAudioFormat = DAC_PCM_176_4KHZ;
            Device.MasterClock = CLK_22_5792MHZ;
            break;
          }
          case XMOS_PCM_192_KHZ:{
            Device.DacAudioFormat = DAC_PCM_192_KHZ;
            Device.MasterClock = CLK_24_575MHZ;
            break;
          }
          case XMOS_PCM_352_8KHZ:{
            Device.DacAudioFormat = DAC_PCM_352_8KHZ;
            Device.MasterClock = CLK_22_5792MHZ;
            break;
          }
          case XMOS_PCM_384_KHZ:{
            Device.DacAudioFormat = DAC_PCM_384_0KHZ;
            Device.MasterClock = CLK_24_575MHZ;
            break;
          }
          case XMOS_DSD_64:{
            Device.DacAudioFormat = DAC_DSD_64;
            Device.MasterClock = CLK_22_5792MHZ;
            break;
          }
          case XMOS_DSD_128:{
            Device.DacAudioFormat = DAC_DSD_128;
            Device.MasterClock = CLK_22_5792MHZ;
            break;
          }
          case XMOS_DSD_256:{
            Device.DacAudioFormat = DAC_DSD_256;
            Device.MasterClock = CLK_22_5792MHZ;
            break;
          }
          default:
          {
            Device.Diag.XmosFormatUnknownCnt++;
          };
        }

        BD34301_DigitalPowerOff();
        BD34301_SoftwareResetOn();

        Device.DacAudioFormatToBeSet = SrcAudioFormatCorrection(Device.DacAudioFormat, Device.SrcConfig.Curr & 0x80, Device.SRC.System.MODE );
        Device.Diag.DacReConfgiurationCnt++;
        SetMasterClock(Device.MasterClock);
        DelayMs(15); //Kritikus pl 88.2 és 96 váltás között
        BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormat]);

        BD34301_SoftwareResetOff();
        BD34301_DigitalPowerOn();
        BD34301_RamClear(); //Kritkus, nem szól a PCM ha nincs
        BD34301_MuteOff();
        Device.XmosStatus.Pre = Device.XmosStatus.Curr;
      }
    }

    /*
     *
     * Ezt látszólag mindig használja az XMOS szám váltáskor.
     *
     * de DSD->PCM váltáskor a sistergést nem nyomja el.
     */
    if(Device.Route.Curr == ROUTE_USB_DAC)
    {
      static uint8_t preXmosMute;
      if(preXmosMute != XmosIsMute())
      {
        uint8_t currXmosMute = XmosIsMute();
        Device.Diag.XmosMuteSignaledCnt++;
        if(currXmosMute)
          BD34301_RegWrite(0x2A, 0x00); //Mute On
        else
          BD34301_RegWrite(0x2A, 0x03); //Mute Off
        preXmosMute = currXmosMute;
      }
    }

    if(Device.Route.Pre != Device.Route.Curr){
      SetRoute(Device.Route.Curr);
      Device.AudioType.Pre = AUDIO_UNKNOWN;
      Device.XmosStatus.Pre = XMOS_UNKNOWN;
      Device.Route.Pre = Device.Route.Curr;
    }


    /*
     * A Volume 0..100 között értelmezett
     * 0: -110dB Minimum Volume
     * 100: 0dB Maximum Volume
     */
    if(Device.Volume.Pre != Device.Volume.Curr){
      double y = Device.Volume.Curr/100.0;
      double x = (log(y)-log(1E-5))/11.51;
      if(x < 0.1)
        x = 0;
      uint8_t reg = (uint8_t)(255 - x * 255);
      BD34301_RegWrite(0x21, reg);
      BD34301_RegWrite(0x22, reg);
      Device.Volume.Pre = Device.Volume.Curr;
    }

    LiveLedTask(&hLiveLed);
    UartTxTask();
    UpTimeTask();

    /*** SRC Resampler ***/
    /*
     * Ha DSD mértünk a SRC előtt, akkor
     * byassolom az SRC-t, hogy ne szóljon DSD az SRC keresztül, mert max
     * hangerőn sistereg.
     *
     * A státuszban pedig azt jelezni, hogy ki van kapcsolva.
     *
     * Pre állpotot küldöm fel a PC-nek mint státusz
     *
     *
     * ***************** SrcConfig Byte ****************************
     *   7  |  6   |   5   |    4    |   3    |    2   |  1  |  0  |
     *  EN  | OWL1 |  OWL0 |  MODE2  | MODE1  |  MODE0 |  X  |  X  |
     * b1xxx xxxx: Enabled
     * b0xxx xxxx: Disabled / bypass
     * bx11x xxxx: Output Port Data Word Length 16 bits
     * bx00x xxxx: Output Port Data Word Length 24 bits
     * bxxx0 01xx: Output Port is Master mode with RCKI = 128 fs
     * bxxx0 10xx: Output Port is Master mode with RCKI = 512 fs
     * bxxx0 11xx: Output Port is Master mode with RCKI = 256 fs
     * 1->176.4, 2->44.1KHz, 3->88.2KHz
     */

    if( Device.AudioType.Curr == AUDIO_DSD_64 ||
        Device.AudioType.Curr == AUDIO_DSD_128 ||
        Device.AudioType.Curr == AUDIO_DSD_256 ||
        Device.AudioType.Curr == AUDIO_DSD_512)
    {
      HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin, GPIO_PIN_SET);//SRC IC bypass ON
      HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin,GPIO_PIN_SET); //HW SRC Bypass ON
      Device.SrcConfig.Pre &= ~0x80;
    }
    else if(Device.SrcConfig.Pre != Device.SrcConfig.Curr)
    {
      /*
       * Konfigurálás idejére kikapcsolom a DAC-ot
       */

      BD34301_MuteOn();
      BD34301_SoftwareResetOn();

      DelayMs(5);

      if(Device.SrcConfig.Curr & 0x80)
      {
        HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin, GPIO_PIN_RESET); //SRC IC bypass OFF
        HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin,GPIO_PIN_RESET); //HW SRC Bypass OFF
      }
      else
      {
        HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin, GPIO_PIN_SET);//SRC IC bypass ON
        HAL_GPIO_WritePin(MUX_PCM_GPIO_Port, MUX_PCM_Pin,GPIO_PIN_SET); //HW SRC Bypass ON
      }
      /*** Output Port Data Word Length ***/
      Device.SRC.Format.OWL = (Device.SrcConfig.Curr >> 5) & 0x03;
      /*** Setting the Output Port Modes ***/
      Device.SRC.System.MODE = (Device.SrcConfig.Curr >> 2) & 0x07;

      SCR41Update(&Device.SRC);

      /*** Ha az SRC-t bekapcsolta, akkor újra kell konfgiurálni a DAC-ot is ***/
      Device.DacAudioFormatToBeSet = SrcAudioFormatCorrection(Device.DacAudioFormat, Device.SrcConfig.Curr & 0x80, Device.SRC.System.MODE);
      Device.Diag.DacReConfgiurationCnt++;
      SetMasterClock(Device.MasterClock);
      BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormat]);

      BD34301_SoftwareResetOff();
      BD34301_MuteOff();

      DelayMs(5);

      Device.SrcConfig.Pre = Device.SrcConfig.Curr;
    }

    /*
     * ***************** CustomDacConfig  ********************************
     * 31:     LR Swap 1:LR Swap, 0:No Swap
     * 30:     Phase Adjust: 1:Adjust,  0:No Adjust
     * 29,28:  DSD Cut Off Freq
     * 27:     High precision Mode, 1: On, 0:Off
     * 26,25:  0:Sharp Roll-Off, 1: Slow Roll-Off
     * 24,23:  De-Emphisis Filter: O:Off, 1:32KHz, 2:44.1KHz, 3:48KHz
     * 22,21:  Delta Sigma Settings: 0:8x, 1:16x, 2:32x
     */
    if(Device.CustomDacConfig.Pre != Device.CustomDacConfig.Curr)
    {
      /*
       * Konfigurálás idejére kikapcsolom a DAC-ot
       */
      BD34301_MuteOn();
      BD34301_SoftwareResetOn();
      DelayMs(5);

      for(DacAudioFormat_t i = DAC_PCM_32_0KHZ; i <= DAC_DSD_512; i++)
      {

        /*** 31:LR Swap 1:LR Swap, 0:No Swap ***/
        if(Device.CustomDacConfig.Curr & 0x80000000)
          BD34301_ModeList[i].AudioIf3 |= 0x01;
        else
          BD34301_ModeList[i].AudioIf3 &= ~0x01;

        /*** Phase Adjust: 1:Adjust,  0:No Adjust ***/
        BD34301_ModeList[i].Clock2 &= ~0x01;
        if( DAC_PCM_32_0KHZ == i ||
            DAC_PCM_44_1KHZ == i ||
            DAC_PCM_48_0KHZ == i ||
            DAC_PCM_88_2KHZ == i ||
            DAC_PCM_96_0KHZ == i ||
            DAC_PCM_176_4KHZ == i ||
            DAC_PCM_192_KHZ == i ||
            DAC_PCM_352_8KHZ == i ||
            DAC_PCM_384_0KHZ == i ||
            DAC_PCM_705_6KHZ == i ||
            DAC_PCM_768_0KHZ == i ||
            DAC_DSD_64 == i ||
            DAC_DSD_128 == i
            //DAC_DSD_256 == i || update: 230710: recseg ezért tiltom
            //DAC_DSD_512 == i
            )
        if(Device.CustomDacConfig.Curr & 0x40000000)
          BD34301_ModeList[i].Clock2 |= 0x01;


        /*** 29,28:  DSD Cut Off Freq ***/
        if(i == DAC_DSD_64 || i == DAC_DSD_128 || i == DAC_DSD_256 || i == DAC_DSD_512){
          BD34301_ModeList[i].DsdFilter &= ~0x03;
          BD34301_ModeList[i].DsdFilter |= (Device.CustomDacConfig.Curr >> 28) & 0x03;
        }

        if(DAC_PCM_32_0KHZ <=i && i <= DAC_PCM_768_0KHZ)
        {
          /*** 27:PCM High precision Mode Off, 0: On, 1:Off ***/
          /*
           * Bizonyos frekvenciák esetén alkalmazható kapcsolható be, (ez a 0)
           */
          if(Device.CustomDacConfig.Curr & 0x08000000){
            if(DAC_PCM_32_0KHZ == i || DAC_PCM_44_1KHZ == i || DAC_PCM_48_0KHZ == i ||
                DAC_PCM_88_2KHZ == i || DAC_PCM_96_0KHZ == i || DAC_PCM_176_4KHZ == i || DAC_PCM_192_KHZ)
              BD34301_ModeList[i].FirFilter2 &= ~(1<<7); //On
          }
          else{
            BD34301_ModeList[i].FirFilter2 |= 1<<7; //Off
          }

          /*** FIR Filter Settings ***/
          uint8_t fir = (Device.CustomDacConfig.Curr >> 25) & 0x03;
          BD34301_ModeList[i].FirFilter1 &= ~0x0F; //FirAlgo
          BD34301_ModeList[i].FirFilter2 &= ~0x07; //FirCoef

          /*** Filter Settings: Sharp Roll-Off ***/
          if(fir == 0)
          {
            if(DAC_PCM_32_0KHZ == i || DAC_PCM_44_1KHZ == i || DAC_PCM_48_0KHZ == i)
            {
              BD34301_ModeList[i].FirFilter1 |= 0x01;  //FirAlgo
            }
            else if(DAC_PCM_88_2KHZ == i || DAC_PCM_96_0KHZ == i)
            {
              BD34301_ModeList[i].FirFilter1 |= 0x02;  //FirAlgo
              BD34301_ModeList[i].FirFilter2 |= 0x01;  //FirCoef
            }
            else if(DAC_PCM_176_4KHZ == i || DAC_PCM_192_KHZ == i)
            {
              BD34301_ModeList[i].FirFilter1 |= 0x04;  //FirAlgo
              BD34301_ModeList[i].FirFilter2 |= 0x02;  //FirCoef
            }
            else if(DAC_PCM_352_8KHZ == i || DAC_PCM_384_0KHZ == i || DAC_PCM_705_6KHZ || DAC_PCM_768_0KHZ == i)
            {
              BD34301_ModeList[i].FirFilter1 |= 0x08;  //FirAlgo
              BD34301_ModeList[i].FirFilter2 |= 0x80;  //FirCoef
            }
          }
          /*** Filter Settings: Slow Roll-Off ***/
          else if(fir == 1)
          {
            if(DAC_PCM_32_0KHZ == i || DAC_PCM_44_1KHZ == i || DAC_PCM_48_0KHZ == i){
              BD34301_ModeList[i].FirFilter1 |= 0x01;  //FirAlgo
              BD34301_ModeList[i].FirFilter2 |= 0x03;  //FirCoef
            }
            else if(DAC_PCM_88_2KHZ == i || DAC_PCM_96_0KHZ == i){
              BD34301_ModeList[i].FirFilter1 |= 0x02;  //FirAlgo
              BD34301_ModeList[i].FirFilter2 |= 0x04;  //FirCoef
            }
            else if(DAC_PCM_176_4KHZ == i || DAC_PCM_192_KHZ == i){
              BD34301_ModeList[i].FirFilter1 |= 0x04;  //FirAlgo
              BD34301_ModeList[i].FirFilter2 |= 0x05;  //FirCoef
            }
            else if(DAC_PCM_352_8KHZ == i || DAC_PCM_384_0KHZ == i || DAC_PCM_705_6KHZ || DAC_PCM_768_0KHZ == i)
            {
              BD34301_ModeList[i].FirFilter1 |= 0x08;  //FirAlgo
              BD34301_ModeList[i].FirFilter2 |= 0x80;  //FirCoef
            }
          }

          /** 24,23:  De-Emphisis Filter: O:Off, 1:32KHz, 2:44.1KHz, 3:48KHz ***/
          uint8_t de = (Device.CustomDacConfig.Curr >> 23) & 0x03;
          /*** De-Emphisis Filter: O:Off ***/
          BD34301_ModeList[i].DeEmph1 = 0x00; //FirAlgo
          BD34301_ModeList[i].DeEmph2 = 0x00; //FirCoef
          /*** De-Emphisis Filter: 1:32KHz ***/
          if(de == 1){
            BD34301_ModeList[i].DeEmph1 = 0x01; //DempFs
            BD34301_ModeList[i].DeEmph2 = 0x03; //Demp2:Demp1
          }
          /*** De-Emphisis Filter: 2:44.1KHz ***/
          else if(de == 2){
            BD34301_ModeList[i].DeEmph1 = 0x02; //DempFs
            BD34301_ModeList[i].DeEmph2 = 0x03; //Demp2:Demp1
          }
          /*** De-Emphisis Filter: 3:48KHz ***/
          else if(de == 3){
            BD34301_ModeList[i].DeEmph1 = 0x03; //DempFs
            BD34301_ModeList[i].DeEmph2 = 0x03; //Demp2:Demp1
          }

          /*** 22,21:  Delta Sigma Settings: 0:8x, 1:16x, 2:32x ***/
          uint8_t ds = (Device.CustomDacConfig.Curr >> 21) & 0x03;
          /*** Delta Sigma Settings: 0:8x ***/
          if(ds == 0){
            /* Csak bizonyos frekenciákon van x8  Table 15. System Clock Frequency Settings in PCM Mode  */
            if(i == DAC_PCM_32_0KHZ || i == DAC_PCM_44_1KHZ ||  i == DAC_PCM_48_0KHZ || i == DAC_PCM_705_6KHZ || i == DAC_PCM_768_0KHZ){
              BD34301_ModeList[i].DeltaSigma = 0x00;
            }
          }
          /*** Delta Sigma Settings: 1:16x ***/
          else if(ds == 1){
            BD34301_ModeList[i].DeltaSigma = 0x10;
          }
          /*** Delta Sigma Settings: 1:32x ***/
          else if(ds == 2){
            BD34301_ModeList[i].DeltaSigma = 0x11;
          }
        }
      }

      /*
       * Leküldöm az új értékekt majd visszakapcsolom a DAC-ot és várok picit
       */
      Device.DacAudioFormatToBeSet = SrcAudioFormatCorrection(Device.DacAudioFormat, Device.SrcConfig.Curr & 0x80, Device.SRC.System.MODE );
      BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormatToBeSet]);

      BD34301_SoftwareResetOff();
      BD34301_MuteOff();
      DelayMs(5);

      Device.CustomDacConfig.Pre = Device.CustomDacConfig.Curr;
    }

    DebugTask(Device.DebugState);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 48000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  if(HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)UartDmaBuffer, UART_BUFFER_SIZE)!= HAL_OK)
    Device.Diag.UartErrorCnt++;
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TIMEBASE_OUT_GPIO_Port, TIMEBASE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_USB_ISO_Pin|USART1_DIR_Pin|EN_SPDIF_ISO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_I2S_I2C_ISO_Pin|LIVE_LED_Pin|PCM_DET_Pin|MUX_PCM_Pin
                          |MCLK_SEL_ISO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESET_SPDIF_Pin|MUTE_USB_ISO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : A0_USB_ISO_Pin */
  GPIO_InitStruct.Pin = A0_USB_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A0_USB_ISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TIMEBASE_OUT_Pin */
  GPIO_InitStruct.Pin = TIMEBASE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TIMEBASE_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_MUTE_COM_Pin */
  GPIO_InitStruct.Pin = DAC_MUTE_COM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_MUTE_COM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_USB_ISO_Pin USART1_DIR_Pin EN_SPDIF_ISO_Pin */
  GPIO_InitStruct.Pin = EN_USB_ISO_Pin|USART1_DIR_Pin|EN_SPDIF_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RDY_SRC_Pin */
  GPIO_InitStruct.Pin = RDY_SRC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RDY_SRC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RATIO_SRC_Pin */
  GPIO_InitStruct.Pin = RATIO_SRC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RATIO_SRC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_I2S_I2C_ISO_Pin LIVE_LED_Pin MUX_PCM_Pin MCLK_SEL_ISO_Pin */
  GPIO_InitStruct.Pin = EN_I2S_I2C_ISO_Pin|LIVE_LED_Pin|MUX_PCM_Pin|MCLK_SEL_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_SPDIF_Pin */
  GPIO_InitStruct.Pin = RESET_SPDIF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_SPDIF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NON_PCM_Pin ERROR_SPDIF_Pin DSD_PCM_USB_ISO_Pin */
  GPIO_InitStruct.Pin = NON_PCM_Pin|ERROR_SPDIF_Pin|DSD_PCM_USB_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PCM_DET_Pin */
  GPIO_InitStruct.Pin = PCM_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PCM_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_USB_ISO_Pin A2_USB_ISO_Pin */
  GPIO_InitStruct.Pin = A1_USB_ISO_Pin|A2_USB_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUTE_USB_ISO_Pin */
  GPIO_InitStruct.Pin = MUTE_USB_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUTE_USB_ISO_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* GetAudioType --------------------------------------------------------------*/
/*
 * - Az Audio jel tipusat a jel LRCK BCLK mérésvel határozom meg
 * - Az ismert frekvenciak a RLCKfreqArray definialja
 * - BLCK = 64xLRCK
 */
AudioTypes_t GetAudioType()
{
  float tol = 0.04;

  uint32_t lrck = Device.Meas.FreqLRCK_MHz;
  uint32_t bclk  = Device.Meas.FreqBCLK_MHz;

  uint32_t lLrck = 0, hLrck = 0;
  uint32_t lbclk = 0, hbclk = 0;

  AudioTypes_t result = AUDIO_UNKNOWN;

  for(uint8_t i = AUDIO_PCM_32_0KHZ; i < sizeof(AudioFreqArray)/sizeof(uint32_t); i++)
  {
    uint32_t templrck = AudioFreqArray[i];
    hLrck = templrck + templrck * tol;
    lLrck = templrck - templrck * tol;
    if((lLrck < lrck) && (lrck < hLrck))
    {
      uint32_t tempbclk = AudioFreqArray[i] * 64;
      hbclk = tempbclk + tempbclk * tol;
      lbclk = tempbclk - tempbclk * tol;
      if((lbclk < bclk) && (bclk < hbclk)){
        result = i;
        break;
      }
    }
  }

  if(result == AUDIO_UNKNOWN)
  {
    /* Itt már csak DSD lehet most BLCK vonalat kell figyelni csak
     * A táblázatban mos BCLK jelentenek a sorosk
     * */
    for(uint8_t i = AUDIO_DSD_64; i < sizeof(AudioFreqArray)/sizeof(uint32_t); i++){
      uint32_t tempblck = AudioFreqArray[i];
      hbclk = tempblck + tempblck * tol;
      lbclk = tempblck - tempblck * tol;
      if((lbclk < bclk) && (bclk < hbclk)){
        result = i;
        break;
      }
    }
  }

 // DelayMs(50);
  return result;
}
/* DAC -----------------------------------------------------------------------*/

/*
 * SRC előtt méréssel határozom meg, hogy a DAC-nak milyen DacAudioFormat -ot válasszak.
 * Ha az SRC aktiv, akkor ennek a mérésnek az eredényét korrigálni kell az SRC módjával.
 * Ha a mérés DAC_PCM_44_1KHZ mért és az SRC enegdélyezve van az Device.SRC.System.Mode 1,
 * akkor az új AudioFormat az DAC_PCM_176_4KHZ lesz.
 */
DacAudioFormat_t SrcAudioFormatCorrection(DacAudioFormat_t currAudioFormat, uint8_t srcEenabled, uint8_t srcMode)
{
  DacAudioFormat_t retval =  currAudioFormat;
  if(srcEenabled)
  {
    //1->176.4, 2->44.1KHz, 3->88.2KHz
    switch(currAudioFormat)
    {
      case DAC_PCM_32_0KHZ:
      case DAC_PCM_44_1KHZ:
      case DAC_PCM_88_2KHZ:
      case DAC_PCM_176_4KHZ:
      {
        if(srcMode == 1)
          retval = DAC_PCM_176_4KHZ;
        else if(srcMode == 2)
          retval = DAC_PCM_44_1KHZ;
        else if(srcMode == 3)
          retval = DAC_PCM_88_2KHZ;
        break;
      }
      case DAC_PCM_48_0KHZ:
      case DAC_PCM_96_0KHZ:
      case DAC_PCM_192_KHZ:
      {
        if(srcMode == 1)
          retval = DAC_PCM_176_4KHZ;
        else if(srcMode == 2)
          retval = DAC_PCM_48_0KHZ;
        else if(srcMode == 3)
          retval = DAC_PCM_96_0KHZ;
        break;
      }
      default:{}
    }
  }
  return retval;
}

/* FrMeter -------------------------------------------------------------------*/
void FrMeterStart(void)
{
  FreqMeterLrckCounterValue = 0;
  FreqMeterBclkCounterValue = 0;

  FreqMeterTimebaseValue = 0;
  FreqMeterTimebaseStart();
  FreqMeterLrckCoutnerStart();
  FreqMeterBclkCoutnerStart();
  FrMeterTimebaseItEnable();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim->Instance == FRMETER_TIM_TIMEBASE)
   {
     Device.Meas.FreqBCLK_MHz = (FreqMeterBclkCounterValue) * 1000 * 10;
     Device.Meas.FreqLRCK_MHz = (FreqMeterLrckCounterValue) * 1000;

     FreqMeterBclkCounterValue = 0;
     FreqMeterLrckCounterValue = 0;

     /*
      * Ezeket Timebase teszthez használd, ez mérési tartomány 100Hz-től 4MHz-ig használható
      * 10ms-es időalap estén pl a LIVE_LED-nél 20ms-es periódusidőt kell mérned
      * 2022.02.02 by marrob
      */
     //HAL_GPIO_TogglePin(TIMEBASE_OUT_GPIO_Port, TIMEBASE_OUT_Pin);

   }
   else if(htim->Instance == FRMETER_TIM_LRCK_COUNTER)
   {

   }else if(htim->Instance == FRMETER_TIM_BCLK_COUNTER)
   {

   }
}

/* Inputs & Outputs ----------------------------------------------------------*/
void RelayMuteOn(void)
{
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_SET);
}

void RelayMuteOff(void)
{
  HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin, GPIO_PIN_RESET);
}

void SetRoute (Route_t route)
{
  switch(route)
  {
    case ROUTE_NONE_DAC:{
      break;
    }
    case ROUTE_MUTE_DAC:{
      HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_SET);
      break;
    }
    case ROUTE_USB_DAC:{
      HAL_GPIO_WritePin(EN_I2S_I2C_ISO_GPIO_Port, EN_I2S_I2C_ISO_Pin, GPIO_PIN_SET); //HDMI I2C Off
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // USB Input Off
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); // SPDIF Input Off
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_SET); //U121

      HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_RESET); // Mute Off
      break;
    }
    case ROUTE_HDMI_DAC:{
      HAL_GPIO_WritePin(EN_I2S_I2C_ISO_GPIO_Port, EN_I2S_I2C_ISO_Pin, GPIO_PIN_RESET); //HDMI I2C-I2S On
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // USB Input Off
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_RESET); //SPDIF Input Off
      HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_RESET); //Mute Off
      break;
    }
    case ROUTE_BNC_DAC:{
      HAL_GPIO_WritePin(EN_I2S_I2C_ISO_GPIO_Port, EN_I2S_I2C_ISO_Pin, GPIO_PIN_SET); //HDMI I2C Off
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // USB Input Off
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_SET); // SPDIF Input On
      PCM9211_SelectSource(PCM9211_RXIN1);
      HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_RESET); //Mute Off
      break;
    }
    case ROUTE_RCA_DAC:{
      HAL_GPIO_WritePin(EN_I2S_I2C_ISO_GPIO_Port, EN_I2S_I2C_ISO_Pin, GPIO_PIN_SET); //HDMI I2C Off
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); //USB Input Off
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_SET);//SPDIF Input On
      PCM9211_SelectSource(PCM9211_RXIN0);
       HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_RESET); // Mute Off
      break;
    }
    case ROUTE_XLR_DAC:{
      HAL_GPIO_WritePin(EN_I2S_I2C_ISO_GPIO_Port, EN_I2S_I2C_ISO_Pin, GPIO_PIN_SET); //HDMI I2C Off
      HAL_GPIO_WritePin(EN_USB_ISO_GPIO_Port, EN_USB_ISO_Pin, GPIO_PIN_RESET); // USB Input Off
      HAL_GPIO_WritePin(EN_SPDIF_ISO_GPIO_Port, EN_SPDIF_ISO_Pin, GPIO_PIN_SET);// SPDIF Input On
      PCM9211_SelectSource(PCM9211_RXIN2);
      HAL_GPIO_WritePin(DAC_MUTE_COM_GPIO_Port, DAC_MUTE_COM_Pin,GPIO_PIN_RESET); // Mute Off
      break;
    }
  }
}

XmosStatus_t ReadXmosStaus(void)
{
  uint8_t status = 0;

  if(HAL_GPIO_ReadPin(A0_USB_ISO_GPIO_Port, A0_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_A0_USB;

  if(HAL_GPIO_ReadPin(A1_USB_ISO_GPIO_Port, A1_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_A1_USB;

  if(HAL_GPIO_ReadPin(A2_USB_ISO_GPIO_Port, A2_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_A2_USB;

  if(HAL_GPIO_ReadPin(DSD_PCM_USB_ISO_GPIO_Port, DSD_PCM_USB_ISO_Pin) == GPIO_PIN_SET)
    status |= DI_DSD_PCM_USB;
  else
    status &= ~DI_A0_USB;

  return (XmosStatus_t)status;
}

uint8_t XmosIsMute(void){
  uint8_t status = HAL_GPIO_ReadPin(MUTE_USB_ISO_GPIO_Port, MUTE_USB_ISO_Pin) == GPIO_PIN_SET;
  return status;
}


void SetMasterClock(MasterClocks_t clk )
{
  if(clk == CLK_24_575MHZ)
    HAL_GPIO_WritePin(MCLK_SEL_ISO_GPIO_Port, MCLK_SEL_ISO_Pin, GPIO_PIN_SET);

  if(clk == CLK_22_5792MHZ)
    HAL_GPIO_WritePin(MCLK_SEL_ISO_GPIO_Port, MCLK_SEL_ISO_Pin, GPIO_PIN_RESET);
}

/* LEDs ----------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

/* SRC ----------------------------------------------------------------------*/
uint8_t SCR41IsReady(void)
{
  if(HAL_GPIO_ReadPin(RDY_SRC_GPIO_Port, RDY_SRC_Pin) == GPIO_PIN_RESET)
    return 1;
      else
    return 0;
}

/* UART-RS485-----------------------------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size )
{
  if (huart->Instance == USART1)
  {
    /*
     * 1. Ha a bejövő bájtok és a már korábban bejöttek együtt nagyobb mint a buffer, akkro restart.
     * 2. Ha jött zárókarakter, akkor mindent átmásolok a DMA bufferből az RxBufferbe és vége.
     * 3. Ha nem jött zárókarakter, akkor hozzáfüzöm az új bájtokat a már bejöttekhez és
     *    várom a következő csomagban a zárókarkatert.
     *
     * MJ: DMA_IT_HT - A Half Transfer Interruptra nincs szükség.
     *
     *
     */
    if(Size + UartRxBufferPtr > UART_BUFFER_SIZE)
    {
      UartRxBufferPtr = 0;
      memset(UartDmaBuffer, 0xBB, UART_DMA_BUFFER_SIZE);
      memset(UartRxBuffer, 0xCC, UART_BUFFER_SIZE);
      Device.Diag.UartErrorCnt++;
    }
    else
    {
      uint8_t isTerminated = 0;
      for(uint8_t i=0; i < Size; i++)
      {
        if(UartDmaBuffer[i]=='\n')
        {
          memcpy(UartRxBuffer + UartRxBufferPtr, UartDmaBuffer, Size);
          UartRxBufferPtr = 0;
          isTerminated = 1;
          strcpy(UartTxBuffer, UartParser(UartRxBuffer));
        }
      }
      if(!isTerminated)
      {
        if(strlen(UartDmaBuffer) !=0 )
        {
          memcpy(UartRxBuffer + UartRxBufferPtr, UartDmaBuffer, Size);
          UartRxBufferPtr += Size;
        }
        else
        {
          Device.Diag.UartDmaDataEmptyErrorCallbackCnt++;
        }
      }
    }

    if(HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)UartDmaBuffer, UART_DMA_BUFFER_SIZE)!= HAL_OK)
      Device.Diag.UartErrorCnt++;
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  Device.Diag.UartErrorCnt++;
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);

  if(HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)UartDmaBuffer, UART_BUFFER_SIZE)!= HAL_OK)
    Device.Diag.UartErrorCnt++;
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void UART_DMAError(UART_HandleTypeDef *huart)
{
    Device.Diag.UartErrorCnt++;
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    if(HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*)UartDmaBuffer, UART_BUFFER_SIZE)!= HAL_OK)
      Device.Diag.UartErrorCnt++;
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}


char* UartParser(char *line)
{
  unsigned int addr = 0;
  char buffer[UART_BUFFER_SIZE];
  char cmd[RS485_CMD_LENGTH];
  char arg1[RS485_ARG1_LENGTH];
  char arg2[RS485_ARG2_LENGTH];

  int intarg;

  memset(buffer, 0x00, UART_BUFFER_SIZE);
  memset(cmd,0x00, RS485_CMD_LENGTH);
  memset(arg1,0x00, RS485_ARG1_LENGTH);
  memset(arg2,0x00, RS485_ARG2_LENGTH);

  sscanf(line, "#%x %s",&addr, cmd);
  if(addr != CLIENT_RX_ADDR)
  {
    Device.Diag.RS485NotMyCmdCnt++;
    return NULL;
  }
  Device.Diag.RS485RequestCnt++;

  if(!strcmp(cmd, "*IDN?")){
    sprintf(buffer, "*IDN? %s", DEVICE_NAME);
  }
  else if(!strcmp(cmd, "*OPC?")){
    strcpy(buffer, "*OPC? OK");
  }
  else if(!strcmp(cmd, "*WHOIS?")){
    sprintf(buffer, "*WHOIS? %s", DEVICE_NAME);
  }
  else if(!strcmp(cmd, "FW?")){
    sprintf(buffer, "FW? %s", DEVICE_FW);
  }
  else if(!strcmp(cmd, "UID?")){
    sprintf(buffer, "UID? %4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
  }
  else if(!strcmp(cmd, "PCB?")){
    sprintf(buffer, "PCB? %s", DEVICE_PCB);
  }
  else if(!strcmp(cmd,"UPTIME?")){
     sprintf(buffer, "UPTIME? %08lX", Device.UpTimeSec);
  }
  else if(!strcmp(cmd,"UE?")) {
    sprintf(buffer, "UE? %08lX", Device.Diag.UartErrorCnt);
  }

  /*** CLOCKS ***/
  else if(!strcmp(cmd,"AUDIO?")){
     sprintf(buffer, "AUDIO? %02X", Device.AudioType.Curr);
  }
  else if(!strcmp(cmd,"MASTER:CLK?")){
     sprintf(buffer, "MASTER:CLK? %02X", Device.MasterClock);
  }
  else if(!strcmp(cmd,"FRQ:LRCK?")){
    sprintf(buffer, "FRQ:LRCK? %lX", Device.Meas.FreqLRCK_MHz);
  }
  else if(!strcmp(cmd,"FRQ:BCLK?")){
    sprintf(buffer, "FRQ:BCLK? %lX", Device.Meas.FreqBCLK_MHz);
  }

  /*** XMOS ***/
  else if(!strcmp(cmd,"XMOS:STATUS?")){
    sprintf(buffer, "XMOS:STATUS? %02X", Device.XmosStatus.Curr );
  }

  /*** VOLUME ***/
  else if(!strcmp(cmd,"DAC:VOL"))
  {
    sscanf(line, "#%x %s %x",&addr, cmd, &intarg);
    Device.Volume.Curr = intarg;
    strcpy(buffer, "DAC:VOL OK");
  }
  else if (!strcmp(cmd,"DAC:VOL?")){
    sprintf(buffer, "DAC:VOL? %01X", (uint8_t)Device.Volume.Curr);
  }

  /*** DAC CONFIG ***/
  else if(!strcmp(cmd,"DAC:CONFIG")){
    sscanf(line, "#%x %s %d",&addr, cmd, &intarg);

    Device.DacAudioFormat = intarg;

    BD34301_DigitalPowerOff();
    BD34301_SoftwareResetOn();

    //ToDo SetMasterClock(Device.MasterClock);
    BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormat]);

    BD34301_SoftwareResetOff();
    BD34301_DigitalPowerOn();
    BD34301_RamClear();
    BD34301_MuteOff();

    strcpy(buffer, "DAC:CONFIG OK");
  }
  else if(!strcmp(cmd,"DAC:CONFIG?")){
    sprintf(buffer, "DAC:CONFIG? %02X", (uint8_t)Device.DacAudioFormat);
  }

  /*** DAC PARAMS ***/
  else if(!strcmp(cmd,"DAC:PARAMS?")){
    sprintf(buffer, "DAC:PARAMS? %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
      BD34301_ModeList[Device.DacAudioFormat].Clock2,
      BD34301_ModeList[Device.DacAudioFormat].AudioIf3,
      BD34301_ModeList[Device.DacAudioFormat].DsdFilter,
      BD34301_ModeList[Device.DacAudioFormat].FirFilter1,
      BD34301_ModeList[Device.DacAudioFormat].FirFilter2,
      BD34301_ModeList[Device.DacAudioFormat].DeEmph1,
      BD34301_ModeList[Device.DacAudioFormat].DeEmph2,
      BD34301_ModeList[Device.DacAudioFormat].DeltaSigma);
  }
  else if(!strcmp(cmd,"DAC:PARAMS")){
    int arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8;
      sscanf(line, "#%x %s %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                    &addr, cmd, &arg1, &arg2, &arg3, &arg4, &arg5, &arg6, &arg7, &arg8);
      BD34301_ModeList[Device.DacAudioFormat].Clock2 = arg1;
      BD34301_ModeList[Device.DacAudioFormat].AudioIf3  = arg2;
      BD34301_ModeList[Device.DacAudioFormat].DsdFilter = arg3;
      BD34301_ModeList[Device.DacAudioFormat].FirFilter1 = arg4;
      BD34301_ModeList[Device.DacAudioFormat].FirFilter2 = arg5;
      BD34301_ModeList[Device.DacAudioFormat].DeEmph1 = arg6;
      BD34301_ModeList[Device.DacAudioFormat].DeEmph2 = arg7;
      BD34301_ModeList[Device.DacAudioFormat].DeltaSigma = arg8;
      strcpy(buffer, "DAC:CONFIG OK");
      Device.Diag.DacReConfgiurationCnt++;
      SetMasterClock(Device.MasterClock);
      BD34301_ModeSwitching(&BD34301_ModeList[Device.DacAudioFormat]);
  }

  /*** ROUTE ***/
  else if(!strcmp(cmd,"ROUTE?")){
     sprintf(buffer, "ROUTE? %02X", Device.Route.Curr);
  }
  else if(!strcmp(cmd,"ROUTE:ACTUAL?")){
     sprintf(buffer, "ROUTE:ACTUAL? %02X", Device.Route.Curr);
  }
  else if(!strcmp(cmd,"ROUTE")){
    sscanf(line, "#%x %s %d",&addr, cmd, &intarg);
    Device.Route.Curr = intarg;
    strcpy(buffer, "ROUTE OK");
  }

  /*** SRC***/
  else if(!strcmp(cmd,"SRC:PARAMS")){
    int arg1, arg2, arg3;
    sscanf(line, "#%x %s %02X:%02X:%02X",&addr, cmd, &arg1, &arg2, &arg3);
      Device.SRC.Data[3] = arg1; //System
      Device.SRC.Data[4] = arg2; //Filter
      Device.SRC.Data[5] = arg3; //Format
      strcpy(buffer, "SRC:PARAMS OK");
      SCR41Update(&Device.SRC);
  }
  else if (!strcmp(cmd,"SRC:PARAMS?")){
    sprintf(buffer, "SRC:PARAMS? %02X:%02X:%02X",
     Device.SRC.Data[3],  //System
     Device.SRC.Data[4],  //Filter
     Device.SRC.Data[5]); //Format
  }
  else if (!strcmp(cmd,"SRC:FSOUT?")){
    sprintf(buffer, "SRC:FSOUT? %02X", (Device.SrcConfig.Curr >> 2) & 0x07 );
  }
  else if(!strcmp(cmd,"SRC:FSOUT")){
    sscanf(line, "#%x %s %02X",&addr, cmd, &intarg);
      Device.SrcConfig.Curr &= ~0x1C;
      Device.SrcConfig.Curr |= (intarg << 2) & 0x1C ;
      strcpy(buffer, "SRC:FSOUT OK");
  }
  else if(!strcmp(cmd,"SRC:EN")){
    sscanf(line, "#%x %s %02X", &addr, cmd, &intarg);
      if(intarg)
        Device.SrcConfig.Curr |= 0x80;
      else
        Device.SrcConfig.Curr &= ~0x80;
      strcpy(buffer, "SRC:EN OK");
  }
  else if(!strcmp(cmd,"SRC:EN?")){
    sprintf(buffer, "SRC:EN? %02X", Device.SrcConfig.Curr & 0x80);
  }

  /*** --- MGUI ONLY --- ***/
  else if(!strcmp(cmd,"CFG")){
    int arg1, arg2, arg3, arg4;
    sscanf(line, "#%x %s %02x %02x %02x %08x",
        &addr, cmd, &arg1, &arg2, &arg3, &arg4);
    Device.Route.Curr = arg1;
    Device.Volume.Curr = arg2;
    Device.SrcConfig.Curr = arg3,
    Device.CustomDacConfig.Curr = arg4;

    sprintf(buffer, "STA %08lX %02X %02lX %02X %08lX %02X %08lX %02X %02X",
      Device.UpTimeSec,
      Device.Route.Pre,
      Device.Volume.Pre,
      Device.DacAudioFormat,
      Device.Diag.UartErrorCnt,
      Device.SrcConfig.Pre,
      Device.CustomDacConfig.Pre,
      Device.Diag.DacReConfgiurationCnt,
      Device.Diag.XmosMuteSignaledCnt);
  }
  else{
    Device.Diag.RS485UnknwonCnt++;
  }
  static char resp[UART_BUFFER_SIZE + 5];
  memset(resp, 0x00, UART_BUFFER_SIZE);
  sprintf(resp, "#%02X %s", CLIENT_TX_ADDR, buffer);
  strcat(resp,"\n");
  return resp;
}

void UartTxTask(void)
{
  uint8_t txn=strlen(UartTxBuffer);
  if( txn != 0)
  {
    Device.Diag.RS485ResponseCnt++;
    RS485DirTx();
    DelayMs(RS485_TX_HOLD_MS);
    HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, txn, 100);
    UartTxBuffer[0] = 0;
    RS485DirRx();
  }
}

void RS485DirTx(void)
{
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
}

void RS485DirRx(void)
{
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
}

void Reset(void)
{
  /*** SRC & DAC ***/
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
}


void DebugTask(DebugState_t dbg)
{
  switch(Device.DebugState){
    case SDBG_IDLE:{
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_MAKE_HARDFAULT:{
      *(__IO uint32_t *) 0xA0001000 = 0xFF;
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_HARD_RESET:{
      NVIC_SystemReset();
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_DAC_MUTE_ON:{
      /*** Mute On ***/
      BD34301_RegWrite(0x2A, 0x00);
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_DAC_MUTE_OFF:{
      /*** Mute Off ***/
      BD34301_RegWrite(0x2A, 0x03);
      dbg = SDBG_IDLE;
      break;
    }
    case SDBG_DAC_RECONFIG:{
      //DacSoftRstOn();
      //DacSetParams(&DacConfigurations[Device.DacAudioFormat], Device.MasterClock);
      //DacSoftRstOff();
      break;
    }

    case SDBG_LAST:{
      dbg = SDBG_IDLE;
      break;
    }
  }
}

/* printf --------------------------------------------------------------------*/
//nincs bekötve a TDO

/* Tools----------------------------------------------------------------------*/
void UpTimeTask(void)
{
  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.UpTimeSec++;
  }
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
