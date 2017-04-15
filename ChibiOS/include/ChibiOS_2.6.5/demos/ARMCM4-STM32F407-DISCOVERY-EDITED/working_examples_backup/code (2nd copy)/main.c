/* http://recursive-labs.com/blog/2012/05/12/chibios-stm32-priority-inversion/ */

#include "ch.h" 	// Standard header
#include "hal.h"	// Hardware abstaction layer header

Semaphore flag;

void busyLoop() {
	volatile unsigned int i, j;
	for (i = 0; i < 20; i++) {
		for (j = 0; j < 10000000; j++);
	}
}

static WORKING_AREA(wathreadHigh, 128);
static WORKING_AREA(wathreadMedium, 128);
static WORKING_AREA(wathreadLow, 128);

/*

	palSetPad(GPIOD, GPIOD_LED5); 	// Red LED
	palClearPad(GPIOD, GPIOD_LED4);
	palClearPad(GPIOD, GPIOD_LED6);

	palSetPad(GPIOD, GPIOD_LED4); // Green LED
	palClearPad(GPIOD, GPIOD_LED5);
	palClearPad(GPIOD, GPIOD_LED6);

	palSetPad(GPIOD, GPIOD_LED6); 	// Blue LED
	palClearPad(GPIOD, GPIOD_LED4);
	palClearPad(GPIOD, GPIOD_LED5);

*/

static msg_t threadHigh(void *arg) {
	chThdSleepMilliseconds(1000);
	chSemWait(&flag);
    palSetPad(GPIOD, GPIOD_LED6);
    palClearPad(GPIOD, GPIOD_LED5);
	chSemSignal(&flag);
}

static msg_t threadMedium(void *arg) {
	chThdSleepMilliseconds(2000);
	chSemWait(&flag);
    palSetPad(GPIOD, GPIOD_LED5);
    palClearPad(GPIOD, GPIOD_LED4);
    palClearPad(GPIOD, GPIOD_LED6);
	chSemSignal(&flag);
	//while(1);
}

static msg_t threadLow(void *arg) {
	chSemWait(&flag); /* This operation decreases the semaphore counter. If the result is negative then the invoking thread is queued. */
    palSetPad(GPIOD, GPIOD_LED4);
	//busyLoop();
    chThdExit(&flag);
	chSemSignal(&flag); /* This operation increases the semaphore counter. If the result is non-negative then a waiting thread is removed from the queue and resumed. */
}

int main(void) {
	/* Initialize OS */
	halInit();
	chSysInit();

 	palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_OUTPUT_PUSHPULL);
 	palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_OUTPUT_PUSHPULL); 	

    palClearPad(GPIOD, GPIOD_LED4); 
	palClearPad(GPIOD, GPIOD_LED5);
	palClearPad(GPIOD, GPIOD_LED6); 

	chSemInit(&flag, 1);
 
	chThdCreateStatic(wathreadHigh,     sizeof(wathreadHigh),   NORMALPRIO + 10,    threadHigh,     NULL);
	chThdCreateStatic(wathreadMedium, 	sizeof(wathreadMedium), NORMALPRIO + 7,     threadMedium,   NULL);
	chThdCreateStatic(wathreadLow,      sizeof(wathreadLow),    NORMALPRIO + 5,     threadLow,      NULL);	

  while (true) {
	chThdSleepMilliseconds(100);
  }

}
