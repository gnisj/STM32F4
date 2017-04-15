#include "ch.h"
#include "hal.h"
//#include "test.h"

int main(void) {

  halInit();
  chSysInit();

  //palSetPadMode(GPIOD, GPIOD_LED1, PAL_MODE_OTUPUT_PUSHPULL);

palSetPadMode(GPIOD,4, PAL_MODE_INPUT);
palSetPadMode(GPIOD,6, PAL_MODE_OUTPUT_PUSHPULL);

  while (TRUE) {
	/*palSetPad(GPIOD, GPIOD_LED6);    
	chThdSleepMilliseconds(500);
	palClearPad(GPIO, GPIOD_LED6);
	chThdSleepMilliseconds(500);
*/
	int x;
	x=palReadPad(GPIOD,4);
	
	if(x) {
		palSetPad(GPIOD, 6);
	} 
	else {
		palClearPad(GPIOD, 6);
	}
  }
}
