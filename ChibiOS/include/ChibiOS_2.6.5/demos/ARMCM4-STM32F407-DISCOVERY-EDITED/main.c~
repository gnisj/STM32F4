/* http://forum.chibios.org/phpbb/viewtopic.php?f=4&t=7 */

#include "ch.h" 	// Standard header.
#include "hal.h"	// Hardware abstaction layer header.


// ADCConfig structure for stm32 MCUs is empty.
static ADCConfig adccfg = {};

#define ADC_BUF_DEPTH 2
#define ADC_CH_NUM 2

// Results array
static adcsample_t samples_buf[ADC_BUF_DEPTH * ADC_CH_NUM];

static ADCConversionGroup adccg = {
	TRUE,                       // Enables the circular buffer mode for the group. 
	(uint16_t)(ADC_CH_NUM),     // Number of the analog channels belonging to the conversion group. 
	NULL,                       // Callback function associated to the group or NULL. 
	0,                          // Error callback or NULL. 
	0,                          // ADC CR1 register initialization data.   
	0,                          // ADC CR2 register initialization data. 
	0,                          // ADC SMPR1 register initialization data. 
	((ADC_CH_NUM - 1) << 20),   // ADC SMPR2 register initialization data.
	0,                          // ADC SQR1 register initialization data. 
	(15 | (10 << 5))            // ADC SQR2 register initialization data. 
};
	

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  // Setup pins of our MCU as analog inputs
  palSetPadMode(IOPORT3, 5, PAL_MODE_INPUT_ANALOG); // this is 15th channel
  palSetPadMode(IOPORT3, 0, PAL_MODE_INPUT_ANALOG); // this is 10th channel

  // Following 3 functions use previously created configuration
  // to initialize ADC block, start ADC block and start conversion.
  // &ADCD1 is pointer to ADC driver structure, defined in the depths of HAL.
  // Other arguments defined ourself earlier.
  adcInit();
  adcStart(&ADCD1, &adccfg);
  adcStartConversion(&ADCD1, &adccg, &samples_buf[0], ADC_BUF_DEPTH);

  // Thats all. Now your conversion run in background without assistance.

  uint16_t i = 0;

  while (TRUE) {
    i = samples_buf[0] / 2;
  }
  return 0;
}
