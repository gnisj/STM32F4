/** @file main.c
 *  @brief Tinkering with STM32F407VGT6 and nRF24L01.
 * 
 * Important information is important and will be written here.
 * More important information.
 * 
 * @author John Doe
 * @bug No known bugs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "main.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"

#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "gpio.h"


uint8_t i = 0;

int ConvertedValue = 0;
float value_in_volts = 0.0;


/**********  MAIN PROGRAM AREA **********/

/** @brief   Main function.
 *  @param   None.
 *  @retval  None.
 */
int main(void)
{
  LedInit();
  Init();
  UsartInit(9600);
  
  //csInit();
  // spiInit(SPI1);
  adc_config();
  char bufferadc[100];
  
  while(1){
    Dummy(); // Uncomment to run USART to terminal testing, BOO!
    //spiLoopbackTest();
    
    ConvertedValue = adc_convert();
    value_in_volts = 5.0 / 4096.0 * ConvertedValue;
    Delay(10000000L);
    char mybuf[8];
    int a = 4;
    sprintf(bufferadc, "Voltage: %fV\n\r", value_in_volts);//ConvertedValue);
    UsartPuts(USART1, bufferadc);
  }
  return 0;
}



/** 
 * @brief Dummy test function.
 *
 */
void Dummy()
{
  if (GPIOA->IDR & 0x0001) {
    if (i > 3) {
      i = 0;
    } else {
      switch(i) {
      case 0:
        /* Turn on green LED1, turn off blue LED4 */
        GPIOD->BSRRL = 0x1000;
        GPIOD->BSRRH = 0x8000;
        UsartPuts(USART1, "I'm back motherfuckers!\r\n");
        break;

      case 1:
        /* Turn on orange LED2, turn off green LED1 */
        GPIOD->BSRRL = 0x2000;
        GPIOD->BSRRH = 0x1000;
        break;        

      case 2:
        /* Turn on red LED3, turn off orange LED2 */
        GPIOD->BSRRL = 0x4000;
        GPIOD->BSRRH = 0x2000;
        break;        

      case 3:
        /* Turn on blue LED4, turn off red LED3 */
        GPIOD->BSRRL = 0x8000;
        GPIOD->BSRRH = 0x4000;
        break;        
      }
      
      i++;
    }
    Delay(3000000L);
  }
}

/**********  END MAIN PROGRAM AREA **********/

/**********  MISC INIT FUNCTIONS **********/

/*
 * Dummy function to avoid compiler error
 */
void _init()
{}
       
