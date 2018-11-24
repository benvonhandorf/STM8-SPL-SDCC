/**
  ******************************************************************************
  * @file    STM8S207K6_Ex01\src\main.c
  * @author  MCD Application Team
  * @version V2.5.0
  * @date    14-October-2013
  * @brief   RC acquisition example with 5 touchkeys
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
#include "stm8_tsl_api.h"

/* Memory section ------------------------------------------------------------*/
#if defined(_COSMIC_) && defined(USE_PRAGMA_SECTION)
#pragma section [CONFIG_RAM]
#pragma section @tiny [CONFIG_RAM0]
#pragma section (CONFIG_CODE)
#pragma section const {CONFIG_CONST}
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* LD4 on STM8/128-EVAL board + STM8S20xxK-TS1 daughter board (PA1) */
#define LED1_PIN_MASK  ((u8)0x02)
#define LED1_PORT_ODR  GPIOA->ODR
#define LED1_PORT_DDR  GPIOA->DDR
#define LED1_PORT_CR1  GPIOA->CR1

/* LD3 on STM8/128-EVAL board + STM8S20xxK-TS1 daughter board (PB4) */
#define LED2_PIN_MASK  ((u8)0x10)
#define LED2_PORT_ODR  GPIOB->ODR
#define LED2_PORT_DDR  GPIOB->DDR
#define LED2_PORT_CR1  GPIOB->CR1

#define LED1_ON()  {LED1_PORT_ODR |= LED1_PIN_MASK; Led1State = ON;}
#define LED1_OFF() {LED1_PORT_ODR &= (u8)(~LED1_PIN_MASK); Led1State = OFF;}
#define LED1_TOG() {if (Led1State == OFF) LED1_ON() else LED1_OFF(); }

#define LED2_ON()  {LED2_PORT_ODR |= LED2_PIN_MASK; Led2State = ON;}
#define LED2_OFF() {LED2_PORT_ODR &= (u8)(~LED2_PIN_MASK); Led2State = OFF;}
#define LED2_TOG() {if (Led2State == OFF) LED2_ON() else LED2_OFF(); }

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

typedef enum
{
  OFF = 0,
  ON = 1
} LedState_T;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

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
void SystemRecovery_Action(void);

/* Private functions ---------------------------------------------------------*/

/**
  ******************************************************************************
  * @brief Main function.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */
void main(void)
{

/* Only if _stext routine is not used */
/*
#asm
		sim		// Disable interrupts		
	// To initialize the page 0
		clrw	x
loop0:
		clr	(x)
		incw	x
		cpw	x,#0x05FF
		jrne	loop0
#endasm
*/

	CLK->CKDIVR = 0x00; // Main freq divided by 1 = Full Freq

  /* Set all GPIOs to Output Push-Pull Low. Used GPIOs are configured by the application.	*/
	GPIOA->DDR |= 0x06;
  GPIOA->CR1 |= 0x06;
	GPIOB->DDR |= 0x3F;
  GPIOB->CR1 |= 0x3F;
  GPIOC->DDR |= 0xFE;
  GPIOC->CR1 |= 0xFE;
  GPIOD->DDR |= 0xFD;
  GPIOD->CR1 |= 0xFD;
  GPIOE->DDR |= 0x20;
  GPIOE->CR1 |= 0x20;
  GPIOF->DDR |= 0x10;
  GPIOF->CR1 |= 0x10;

	TSL_Init();

  ExtraCode_Init();

  for (;;) {
    
		ExtraCode_StateMachine();
		
    TSL_Action();
    
	}
  
}


/**
  ******************************************************************************
  * @brief Initialize all the keys, I/Os for LED
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */
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

  /* Init I/O in Output Push-Pull to drive the LED */
  LED1_PORT_ODR |= LED1_PIN_MASK; /* LED1 is ON by default */
  LED1_PORT_DDR |= LED1_PIN_MASK;
  LED1_PORT_CR1 |= LED1_PIN_MASK;

  /* Init I/O in Output Push-Pull to drive the LED */
  LED2_PORT_ODR |= LED2_PIN_MASK; /* LED2 is ON by default */
  LED2_PORT_DDR |= LED2_PIN_MASK;
  LED2_PORT_CR1 |= LED2_PIN_MASK;
  
  LED1_ON();
  LED2_ON();
  
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
void ExtraCode_StateMachine(void)	{

  if ((TSL_GlobalSetting.b.CHANGED) && (TSLState == TSL_IDLE_STATE))
  {
   
    TSL_GlobalSetting.b.CHANGED = 0;
    
    if (KEY01_DETECTED)
    {
      LED1_TOG();
    }
   
    if (KEY03_DETECTED)
    {
      LED2_TOG();
    }
   
    if (KEY02_DETECTED)
    {
      Key02Touched = 1;
      LED1_ON();
    }
   
    if (!KEY02_DETECTED && Key02Touched)
    {
      Key02Touched = 0;
      LED1_OFF();
    }
   
    if (KEY04_DETECTED)
    {
      Key04Touched = 1;
      LED2_ON();
    }
   
    if (!KEY04_DETECTED && Key04Touched)
    {
      Key04Touched = 0;
      LED2_OFF();
    }
    
    if (KEY05_DETECTED)
    {
      if (KEY02_DETECTED)
      {
        // Works only if DxS is disabled on the keys
        ToggleLED1 ^= 1;
        if (ToggleLED1)
        {
          LED1_TOG();
          Led1Delay = 2; /* 200ms */
          TSL_Tick_Flags.b.User1_Start_100ms = 1; /* Start timer */
        }
        else
        {
          LED1_OFF();
          Led1Delay = 0;
        }
      }
      else if (KEY04_DETECTED)
      {
        // Works only if DxS is disabled on the keys
        ToggleLED2 ^= 1;
        if (ToggleLED2)
        {
          LED2_TOG();
          Led2Delay = 2; /* 200ms */
          TSL_Tick_Flags.b.User2_Start_100ms = 1; /* Start timer */
        }
        else
        {
          LED2_OFF();
          Led2Delay = 0;
        }
      }     
      else
      {
        LED1_ON();
        LED2_ON();
        Led1Delay = 6; /* 600ms */
        Led2Delay = 8; /* 800ms */
        TSL_Tick_Flags.b.User1_Start_100ms = 1; /* Start timer */
        TSL_Tick_Flags.b.User2_Start_100ms = 1; /* Start timer */
      }
    }
    
  }

  /* Check end of timer to switch-off the LED1 */
  if (Led1Delay > 0)
  {
    if (TSL_Tick_Flags.b.User1_Flag_100ms)
    {
      TSL_Tick_Flags.b.User1_Flag_100ms = 0;
      Led1Delay--;
      if (Led1Delay == 0)
      {
        if (ToggleLED1)
        {
          LED1_TOG();
          Led1Delay = 2; /* 200ms */
          TSL_Tick_Flags.b.User1_Start_100ms = 1; /* "Start" timer again */
        }
        else
        {
          LED1_OFF();
        }
      }
      else
      {
        TSL_Tick_Flags.b.User1_Start_100ms = 1; /* "Start" timer again */
      }
    }
  }

  /* Check end of timer to toggle or switch-off the LED2 */
  if (Led2Delay > 0)
  {
    if (TSL_Tick_Flags.b.User2_Flag_100ms)
    {
      TSL_Tick_Flags.b.User2_Flag_100ms = 0;
      Led2Delay--;
      if (Led2Delay == 0)
      {
        if (ToggleLED2)
        {
          LED2_TOG();
          Led2Delay = 2; /* 200ms */
          TSL_Tick_Flags.b.User2_Start_100ms = 1; /* "Start" timer again */
        }
        else
        {
          LED2_OFF();
        }
      }
      else
      {
        TSL_Tick_Flags.b.User2_Start_100ms = 1; /* "Start" timer again */
      }
    }
  }
  
}

/* Public functions ----------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
