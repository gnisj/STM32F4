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
#include "SPI.h"
//#include "UART.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "adc.h"
#include "uart.h"



// Private variables
volatile uint32_t time_var1, time_var2;

uint8_t i = 0;

int ConvertedValue = 0;
float value_in_volts = 0.0;

uint8_t txbuf[4];
uint8_t rxbuf[4];
uint16_t txbuf16[4];
uint16_t rxbuf16[4];

static const uint16_t speeds[] = {
  [SPI_SLOW] = SPI_BaudRatePrescaler_64,
  [SPI_MEDIUM] = SPI_BaudRatePrescaler_8,
  [SPI_FAST] = SPI_BaudRatePrescaler_2 };

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
  
  csInit();
  spiInit(SPI1);
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


void spiLoopbackTest()
{
  int i,j;
  for(i = 0; i < 8; i++) {
    for(j = 0; j < 4; j++)
      txbuf[j] = i*4 + j;
    GPIO_WriteBit(GPIOC, GPIO_Pin_3 , 0);
    spiReadWrite(SPI1, rxbuf, txbuf, 4, SPI_SLOW);
    GPIO_WriteBit(GPIOC, GPIO_Pin_3 , 1);
  
  }
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

/********** END MISC INIT FUNCTIONS **********/

/********** SPI FUNCTIONS **********/

uint8_t eepromReadStatus()
{
  //uint8_t cmd[] = {cmdRDSR, 0xff};
  uint8_t res[2];
  //https://github.com/nalepae/stm32_tutorial/tree/master/src
}

/** 
 * @brief  Initializes SPIx on PORTA. Current version only works for SPI1.
 *         To be expanded for SPI2 and SPI 3 (TBD).
 * @param  SPIx SPI interface where x is 1, 2 or 3.
 * @retval None
 */
void spiInit(SPI_TypeDef *SPIx) 
{
  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_StructInit(&GPIO_InitStructure);
  SPI_StructInit(&SPI_InitStructure);

  if (SPIx == SPI1) {
    // Enable peripheral clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    // Enable clock for used IO pins
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Configure IO pins on Port A
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Connect SPI1 pins to SPI alternate function
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  } else {
    return;
  }

  // Configure the chip select signal on GPIOE (CHECK THIS! Where's it from)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIOE->BSRRL |= GPIO_Pin_7;

  // Configure SPIx
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = speeds[SPI_SLOW];
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPIx, &SPI_InitStructure);

  SPI_Cmd(SPIx, ENABLE);
}

/** 
 * @brief SPI test function.
 * Broke after spi1Send stopped compiling
 */ 
/*void spiTest()
  {
  uint8_t received_val = 0;
  GPIOE->BSRRH |= GPIO_Pin_7; // set PE7 (CS) low
  spi1Send(0xAA);  // transmit data
  received_val = spi1Send(0x00); // transmit dummy byte and receive data
  GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 (CS) high
  
  }
*/ 


/** 
 * @brief Chip select
 * Copied from 
 * https://www.mattjquinn.com/technical_writing/2015/06/27/stm32-spi-visual.html
 * https://github.com/thaletterb/STM32-SPI/blob/master/SPI/main.c
 */ 
void csInit(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Initialize chip select to high.
  GPIO_WriteBit(GPIOC, GPIO_Pin_3, 1);
}

/** @brief Simple send function for SPI1
 *
 *  @param data Character to be sent.
 *  @return Returns the DR register contents of SPI1
 *
 uint8_t spi1Send(uint8_t data)
 {
 SPI1->DR = data;
 while( !(SPI1->SR & SPI_I2S_FLAG_TXE) );
 while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) );
 while( SPI1->SR & SPI_I2S_FLAG_BSY);
 return SPI1->DR;
 }*/

// See https://github.com/geoffreymbrown/STM32-Template

int spiReadWrite(SPI_TypeDef* SPIx,
                 uint8_t *rbuf,
                 const uint8_t *tbuf,
                 int cnt,
                 enum spiSpeed speed)
{
  int i;
  
  SPIx->CR1 = (SPIx->CR1 & ~SPI_BaudRatePrescaler_256) | speeds[speed];

  for (i = 0; i < cnt; i++) {
    if (tbuf) {
      SPI_I2S_SendData(SPIx, *tbuf++);
    } else {
      SPI_I2S_SendData(SPIx, 0xff);
    }
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    if (rbuf) {
      *rbuf++ = SPI_I2S_ReceiveData(SPIx);
    } else {
      SPI_I2S_ReceiveData(SPIx);
    }
  }
  return i;
}

/********** END SPI FUNCTIONS **********/

/********** USART FUNCTIONS **********/

/********** END USART FUNCTIONS **********/

/********** MISC FUNCTIONS **********/

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

/*
 * Dummy function to avoid compiler error
 */
void _init()
{}

/********** END MISC FUNCTIONS **********/
                                          
/*
void ADC_Config(void)
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
*/
