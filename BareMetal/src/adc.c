#include "stm32f4xx_conf.h"
#include "stm32f4xx_adc.h"
#include "adc.h"


void adc_config(void)
{
  // Structure for ADC configuration
  ADC_InitTypeDef ADC_init_structure;
  // Structure for analog input pin
  GPIO_InitTypeDef GPIO_initStructure;
  
  // Enable clock for ADC1 which is connected
  // to the APB2 peripheral bus
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE);
  
  // Analog pin config
  // Channel 10 is connected to PC0
  GPIO_initStructure.GPIO_Pin = GPIO_Pin_0;
  // PC0 to analog mode
  GPIO_initStructure.GPIO_Mode = GPIO_Mode_AN;
  // No pullupdown
  GPIO_initStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  // Apply to selected GPIO or something like that
  GPIO_Init(GPIOC, &GPIO_initStructure);

  ADC_DeInit();
  ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_init_structure.ADC_Resolution = ADC_Resolution_12b;
  ADC_init_structure.ADC_ContinuousConvMode = ENABLE;
  ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_init_structure.ADC_NbrOfConversion = 1;
  ADC_init_structure.ADC_ScanConvMode = DISABLE;
  ADC_Init(ADC1, &ADC_init_structure);
  
  ADC_Cmd(ADC1, ENABLE);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);
}

int adc_convert()
{
  ADC_SoftwareStartConv(ADC1);
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  return ADC_GetConversionValue(ADC1);
}
