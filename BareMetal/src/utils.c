/*
 * utils.c
 *
 *  Created on: 11 jul 2012
 *      Author: benjamin
 */
#include "utils.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_conf.h"
#include "main.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"


// Private variables
volatile uint32_t time_var1, time_var2;


/*
 * Can be used with printf when double values are not
 * supported (because hardfloat is used)
 */
char* ftostr(float value, int places) {
	static char buffer[100];
	uint32_t whole;
	uint32_t fraction;
	char sign[2] = "";

	if (value < 0) {
		value = -value;
		sign[0] = '-';
		sign[1] = '\0';
	}

	whole = (uint32_t) value;
	fraction = (uint32_t) ((value - floorf(value)) * powf(10.0f, (float)places) + 0.5f);
	sprintf(buffer, "%s%lu.%*.*lu", sign, whole, places, places, fraction);

	return buffer;
}



/*
 * Simple delay function
 */
void Delay(__IO uint32_t nCount)
{
  while(nCount--) {
  }
}

/*
 * Another delay function
 */
void delay_ms(uint32_t ms)
{
  int c = 0;
  TIM7 -> PSC = 1000;
  //TIM7 -> ARR = tim_clk - 1;
  TIM7 -> CR1 |= TIM_CR1_CEN;
  
  while(c < ms) {
    while(!(TIM7-> SR & TIM_SR_UIF));
    c++;
  }
  
  TIM7 -> CR1 &= ~TIM_CR1_CEN;
}

/*
 * Called from systick handler
 */
void timing_handler()
{
  if (time_var1) {
    time_var1--;
  }
  time_var2++;
}
