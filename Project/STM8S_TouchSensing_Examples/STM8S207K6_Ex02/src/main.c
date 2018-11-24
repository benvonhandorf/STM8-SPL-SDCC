/**
  ******************************************************************************
  * @file    STM8S207K6_Ex02\src\main.c
  * @author  MCD Application Team
  * @version V2.5.0
  * @date    14-October-2013
  * @brief   RC acquisition example with 1 linear sensor (mckey)
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

// LEDs IOs definition using STM8/128-EVAL board + STM8S20xxK-TS1 daughter board
// PD7 connected to CN5 pin 27 = PH3 = LD1
// Warning: need to put a 0 ohm resistor on R44
#define LED1_PIN_MASK  ((u8)0x80)
#define LED1_PORT_ODR  GPIOD->ODR
#define LED1_PORT_DDR  GPIOD->DDR
#define LED1_PORT_CR1  GPIOD->CR1
// PB5 connected to CN5 pin 26 = PH2 = LD2
// Warning: need to put a 0 ohm resistor on R43
#define LED2_PIN_MASK  ((u8)0x20)
#define LED2_PORT_ODR  GPIOB->ODR
#define LED2_PORT_DDR  GPIOB->DDR
#define LED2_PORT_CR1  GPIOB->CR1
// PB4 connected to CN5 pin 25 = PH1 = LD3
#define LED3_PIN_MASK  ((u8)0x10)
#define LED3_PORT_ODR  GPIOB->ODR
#define LED3_PORT_DDR  GPIOB->DDR
#define LED3_PORT_CR1  GPIOB->CR1
// PA1 connected to CN5 pin 22 = PH0 = LD4
#define LED4_PIN_MASK  ((u8)0x02)
#define LED4_PORT_ODR  GPIOA->ODR
#define LED4_PORT_DDR  GPIOA->DDR
#define LED4_PORT_CR1  GPIOA->CR1

#define LED1_ON()  {LED1_PORT_ODR |= LED1_PIN_MASK; Led1State = ON;}
#define LED1_OFF() {LED1_PORT_ODR &= (u8)(~LED1_PIN_MASK); Led1State = OFF;}
#define LED1_TOG() {if (Led1State == OFF) LED1_ON() else LED1_OFF(); }

#define LED2_ON()  {LED2_PORT_ODR |= LED2_PIN_MASK; Led2State = ON;}
#define LED2_OFF() {LED2_PORT_ODR &= (u8)(~LED2_PIN_MASK); Led2State = OFF;}
#define LED2_TOG() {if (Led2State == OFF) LED2_ON() else LED2_OFF(); }

#define LED3_ON()  {LED3_PORT_ODR |= LED3_PIN_MASK; Led3State = ON;}
#define LED3_OFF() {LED3_PORT_ODR &= (u8)(~LED3_PIN_MASK); Led3State = OFF;}
#define LED3_TOG() {if (Led3State == OFF) LED3_ON() else LED3_OFF(); }

#define LED4_ON()  {LED4_PORT_ODR |= LED4_PIN_MASK; Led4State = ON;}
#define LED4_OFF() {LED4_PORT_ODR &= (u8)(~LED4_PIN_MASK); Led4State = OFF;}
#define LED4_TOG() {if (Led4State == OFF) LED4_ON() else LED4_OFF(); }

typedef enum
{
  OFF = 0,
  ON = 1
} LedState_T;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
LedState_T Led1State;
LedState_T Led2State;
LedState_T Led3State;
LedState_T Led4State;

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
	/*GPIOA->DDR |= 0x06;
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
  GPIOF->CR1 |= 0x10;*/

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
  LED1_PORT_ODR &= (uint8_t)~LED1_PIN_MASK; /* LED OFF by default */
  LED1_PORT_DDR |= LED1_PIN_MASK;
  LED1_PORT_CR1 |= LED1_PIN_MASK;

  /* Init I/O in Output Push-Pull to drive the LED */
  LED2_PORT_ODR &= (uint8_t)~LED2_PIN_MASK; /* LED OFF by default */
  LED2_PORT_DDR |= LED2_PIN_MASK;
  LED2_PORT_CR1 |= LED2_PIN_MASK;

  /* Init I/O in Output Push-Pull to drive the LED */
  LED3_PORT_ODR &= (uint8_t)~LED3_PIN_MASK; /* LED OFF by default */
  LED3_PORT_DDR |= LED3_PIN_MASK;
  LED3_PORT_CR1 |= LED3_PIN_MASK;

  /* Init I/O in Output Push-Pull to drive the LED */
  LED4_PORT_ODR &= (uint8_t)~LED4_PIN_MASK; /* LED OFF by default */
  LED4_PORT_DDR |= LED4_PIN_MASK;
  LED4_PORT_CR1 |= LED4_PIN_MASK;

  LED1_OFF();
  LED2_OFF();
  LED3_OFF();
  LED4_OFF();
    
  enableInterrupts();

}


/**
  ******************************************************************************
  * @brief Example
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */
void ExtraCode_StateMachine(void)	{

  if (TSLState == TSL_IDLE_STATE)
  {
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    LED4_OFF();
    
    // The position resolution is configured using
    // the MCKEY_RESOLUTION_DEFAULT parameter.
    if (sMCKeyInfo[0].Position > 5) {
      LED1_ON();
    }
    if (sMCKeyInfo[0].Position > 11) {
      LED2_ON();
    }    
    if (sMCKeyInfo[0].Position > 19) {
      LED3_ON();
    }
    if (sMCKeyInfo[0].Position > 27) {
      LED4_ON();
    }
  }
  
}

/* Public functions ----------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
