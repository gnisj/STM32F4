#include "ch.h" 	// Standard header
#include "hal.h"	// Hardware abstaction layer header

int main(void) {
	// Initialize OS
	halInit();
	chSysInit();
	// Initialization complete
	
 	palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_OUTPUT_PUSHPULL);
 	palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_OUTPUT_PUSHPULL);
 	palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_OUTPUT_PUSHPULL);
 	palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_OUTPUT_PUSHPULL);
	//palSetPadMode(GPIOD, 4, PAL_MODE_INPUT);
	//palSetPadMode(GPIOD, 6, PAL_MODE_OUTPUT_PUSHPULL);

  while (TRUE) {
 	palSetPad(GPIOD, GPIOD_LED3);
	chThdSleepMilliseconds(100);
	palClearPad(GPIOD, GPIOD_LED3);

	palSetPad(GPIOD, GPIOD_LED5);
	chThdSleepMilliseconds(100);
	palClearPad(GPIOD, GPIOD_LED5);

	palSetPad(GPIOD, GPIOD_LED6);
	chThdSleepMilliseconds(100);
	palClearPad(GPIOD, GPIOD_LED6);

	palSetPad(GPIOD, GPIOD_LED4);
	chThdSleepMilliseconds(100);
	palClearPad(GPIOD, GPIOD_LED4);


	
	
/*
	int x;
	x=palReadPad(GPIOD,4);
	
	if(x) {
		palSetPad(GPIOD, 6);
	} 
	else {
		palClearPad(GPIOD, 6);
	}
*/
  }
}
