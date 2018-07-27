#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_conf.h"
#include "gpio.h"

/** 
 *  @brief  Main initizialition function.
 *  @param  None.
 *  @retval None.
 */
void Init()
{

  // Initialize struct 
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*** PUSH-BUTTON ***/
  // Enable peripheral clock on GPIOA
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  

  // Configure PA0 as input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  // Initialize pin on GPIOA
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  // ---------- SysTick timer -------- //
  if (SysTick_Config(SystemCoreClock / 1000)) {
    // Capture error
    while (1){};
  }
}


/** 
 *  @brief  LED initizialition.
 *  @param  None.
 *  @retval None.
 */
void LedInit()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Initialize struct 
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  // Configure PD12, PD13, PD14 and PD15 as push-pull outputs
  GPIO_InitStructure.GPIO_Pin = 
    GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  // Initialize pins on GPIOD
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
