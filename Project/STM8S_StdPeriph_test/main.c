/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   Main program body
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8_tsl_api.h"
#include "stm8s_it.h"    /* SDCC patch: required by SDCC for interrupts */
#include "stdio.h"
#include "stm8s_it.h"    // SDCC requires ISR declaration to be included here

#define DEVICE STM8S103

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef _RAISONANCE_
  #define PUTCHAR_PROTOTYPE int putchar (char c)
  #define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
  #define PUTCHAR_PROTOTYPE char putchar (char c)
  #define GETCHAR_PROTOTYPE char getchar (void)
#elif defined (_SDCC_)                    /* SDCC patch: same types as stdio.h */
  #if SDCC_VERSION >= 30700               // declaration changed in sdcc 3.7.0
    #define PUTCHAR_PROTOTYPE int putchar (int c)
    #define GETCHAR_PROTOTYPE int getchar (void)
  #elif SDCC_VERSION >= 30605               // declaration changed in sdcc 3.6.5
    #define PUTCHAR_PROTOTYPE int putchar (int c)
    #define GETCHAR_PROTOTYPE char getchar (void)
  #else
    #define PUTCHAR_PROTOTYPE void putchar (char c)
    #define GETCHAR_PROTOTYPE unsigned char getchar (void)
  #endif 
#else /* _IAR_ */
  #define PUTCHAR_PROTOTYPE int putchar (int c)
  #define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */


/* LD3 on STM8/128-EVAL board + STM8S20xxK-TS1 daughter board (PB4) */
#define LED2_PIN_MASK  GPIO_PIN_5
#define LED2_PORT      GPIOB

#define LED2_ON()  {GPIO_WriteLow(LED2_PORT, (GPIO_Pin_TypeDef)LED2_PIN_MASK);}
#define LED2_OFF()  {GPIO_WriteHigh(LED2_PORT, (GPIO_Pin_TypeDef)LED2_PIN_MASK);}
#define LED2_TOG()  {GPIO_WriteReverse(LED2_PORT, (GPIO_Pin_TypeDef)LED2_PIN_MASK);}

#if NUMBER_OF_SINGLE_CHANNEL_KEYS > 0
#define KEY01_DETECTED (sSCKeyInfo[0].Setting.b.DETECTED)
#define KEY02_DETECTED (sSCKeyInfo[1].Setting.b.DETECTED)
#define KEY03_DETECTED (sSCKeyInfo[2].Setting.b.DETECTED)
#define KEY04_DETECTED (sSCKeyInfo[3].Setting.b.DETECTED)
#define KEY05_DETECTED (sSCKeyInfo[4].Setting.b.DETECTED)
#else
#define KEY01_DETECTED (0)
#define KEY02_DETECTED (0)
#define KEY03_DETECTED (0)
#define KEY04_DETECTED (0)
#define KEY05_DETECTED (0)
#endif


/* Private macro -------------------------------------------------------------*/
typedef enum
{
  OFF = 0,
  ON = 1
} LedState_T;


LedState_T Led1State;
LedState_T Led2State;

u8 Led1Delay = 0;
u8 Led2Delay = 0;

u8 Key02Touched = 0;
u8 Key04Touched = 0;

u8 ToggleLED1 = 0;
u8 ToggleLED2 = 0;

/* Private function prototypes -----------------------------------------------*/
void ExtraCode_Init(void);
void ExtraCode_StateMachine(void);

/* Private functions ---------------------------------------------------------*/


void main(void)
{
  uint8_t  val  = 0x00;
  //uint32_t addr = 0x40A5;   // address for flash read/write
  //uint32_t i;

  /* init High speed internal clock prescaler: 1 */
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  
  /* Initialize LED pins in Output Mode */
  GPIO_Init(LED2_PORT, (GPIO_Pin_TypeDef)LED2_PIN_MASK, GPIO_MODE_OUT_PP_LOW_FAST);
  LED2_OFF()
  
  // init UART1 to 115.2kBaud, 1/8/1, no parity, no clock
  UART1_DeInit();
  UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
              UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

  printf("Uart initialized\r\n");

  printf("%d - %d touch channels\r\n", NUMBER_OF_SINGLE_CHANNEL_KEYS, NUMBER_OF_MULTI_CHANNEL_KEYS);

  TSL_Init();

  ExtraCode_Init();

  printf("TSL Initialized\r\n");

  for (;;) {
    printf("Gate 1 SK1 State - %x %x\r\n", sSCKeyInfo[0].State.whole, val);

    // if (UART1_GetFlagStatus(UART1_FLAG_RXNE))
    // {
    //   printf("Gate 2\r\n");
    //   val = getchar();
    //   printf("read %c\r\n", val);
    // } 

    //printf("Gate 3\r\n");
    
    ExtraCode_StateMachine();

    //printf("Gate 4\r\n");
    
    TSL_Action();
    
  } // main loop

} // main()

void ExtraCode_Init(void)
{

  u8 i;

  /* All keys are implemented and enabled */

#if NUMBER_OF_SINGLE_CHANNEL_KEYS > 0
  for (i = 0; i < NUMBER_OF_SINGLE_CHANNEL_KEYS; i++)
  {
    sSCKeyInfo[i].Setting.b.IMPLEMENTED = 1;
    sSCKeyInfo[i].Setting.b.ENABLED = 1;
    sSCKeyInfo[i].DxSGroup = 0x00; /* 0x00 = DxS disabled, other values = DxS enabled */
  }
#endif

#if NUMBER_OF_MULTI_CHANNEL_KEYS > 0
  for (i = 0; i < NUMBER_OF_MULTI_CHANNEL_KEYS; i++)
  {
    sMCKeyInfo[i].Setting.b.IMPLEMENTED = 1;
    sMCKeyInfo[i].Setting.b.ENABLED = 1;
    sMCKeyInfo[i].DxSGroup = 0x00; /* 0x00 = DxS disabled, other values = DxS enabled */
  }
#endif

  LED2_OFF();
  
  enableInterrupts();
}

/**
  ******************************************************************************
  * @brief Example of LED switching using touch sensing keys
  * KEY1: LED1 toggles when the key is pressed
  * KEY2: LED1 ON if key is pressed, LED1 OFF if key is released
  * KEY3: LED1 ON then OFF after a delay
  * All KEYs: LED2 toggles continuously / LED2 is OFF 
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */
void ExtraCode_StateMachine(void) {

  if ((TSL_GlobalSetting.b.CHANGED) && (TSLState == TSL_IDLE_STATE))
  {
   
    TSL_GlobalSetting.b.CHANGED = 0;
    
    if (KEY01_DETECTED)
    {
      printf("KEY1");
      LED2_ON();
    } else {
      LED2_OFF();
    }    
  }  
}

/**
  * @brief Retargets the C library printf function to the UART.
  * @param c Character to send
  * @retval char Character sent
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART1_SendData8(c);
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);

  // non SDCC or newer SDCC version
  #if !defined(__SDCC) || (SDCC_VERSION >= 30605)
    return (c);
  #endif
}


/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval char Character to Read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
    c = UART1_ReceiveData8();
  return (c);
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  // avoid compiler warnings
  (void) file;
  (void) line;
  
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
