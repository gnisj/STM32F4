/* Defined to prevent recursive inclusion */
#ifndef __MAIN_H
#define __MAIN_H

/* Includes 
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
*/

/* Prototypes */

void Dummy();
void LedInit();
void Init();


void spiTest();
void csInit();
uint8_t spi1Send(uint8_t);


void UsartInit(uint32_t);
void UsartPuts();
void UsartPutchar(USART_TypeDef*, uint16_t);
void UsartPutn(USART_TypeDef*, uint32_t);
void UsartIRQHandler();

void Delay();
void delay_ms();
void timing_handler();



#endif /* __MAIN_H */
