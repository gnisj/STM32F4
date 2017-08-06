/** @file main.c
 *  @brief Tinkering with STM32F407VGT6 and nRF24L01.
 * 
 * Important information is important and will be written here.
 * More important information.
 * 
 * @author Martin Oredsson
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
#include "UART.h"

#define MAX_STRLEN 12

// Private variables
volatile uint32_t time_var1, time_var2;
volatile char received_string[MAX_STRLEN+1];
uint8_t i = 0;

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
  
  // Chip select
  csInit();

  spiInit(SPI1);
  
  /*
  for (m=0; m<8; m++) {
    for (n=0; n<4; n++) {
      txbuf[n] = m*4 + n;
      GPIO_WriteBit(GPIOA, GPIO_Pin_3, 0);
      spiReadWrite(SPI1, rxbuf, txbuf, 4, SPI_SLOW);
      GPIO_WriteBit(GPIOA, GPIO_Pin_3, 1);
      for (m=0; m<4; m++) {
        if (rxbuf[m] != txbuf[m]){}
          //assert_failed(__FILE__, __LINE__);
      }
    }
  }
  */
  
  while(1){
    //Dummy(); // Uncomment to run USART to terminal testing, BOO!
    spiLoopbackTest();
    
  /*if(GPIOA->IDR & 0x0001) {
      GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    }
    GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
    GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
      */
    //USART_putchar(USART1,SPI1_send(0x56));    
    //USART_putn(USART1, 10);
    //USART_puts(USART1, "\r\n");
    
    // USART_puts(USART1, "BOO!\r\n");
    //Delay(5000000L);
    //SPI1_send(0x55);
    //spiReadWrite(SPI1, rxbuf, txbuf, 4, SPI_SLOW);
    
    //USART_puts(USART1, "Not BOO.\r\n");
    
    //GPIOA->BSRRH |= GPIO_Pin_4;
    //GPIO_ToggleBits(GPIOA, GPIO_Pin_4);
    //SPI1_send(0xAA);
    //received_val = SPI1_send(0x55);
    //GPIOA->BSRRL |= GPIO_Pin_7;
    //GPIO_ToggleBits(GPIOA, GPIO_Pin_4);
    //USART_puts(USART1, received_val);
    
    //USART_putn(USART1, 2);
   
    Delay(50000L);
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
  
    /*for(j = 0; j < 4; j++)
      if(rxbuf[j] != txbuf[j])
        assert_failed(__FILE__ , __LINE__);
    */
  }
  /*for(i = 0; i < 8; i++) {
    for(j = 0; j < 4; j++)
      txbuf16[j] = i*4 + j + (i << 8);
    GPIO_WriteBit(GPIOC, GPIO_Pin_3 , 0);
    spiReadWrite16(SPI1, rxbuf16, txbuf16, 4, SPI_SLOW);
    GPIO_WriteBit(GPIOC, GPIO_Pin_3 , 1);
    for(j = 0; j < 4; j++)
      if(rxbuf16[j] != txbuf16[j])
        assert_failed(__FILE__ , __LINE__);
        }*/
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
        UsartPuts(USART1, "BOO!\r\n");
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
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPIx, &SPI_InitStructure);

  SPI_Cmd(SPIx, ENABLE);
}

/** 
 * @brief SPI test function.
 *
 */ 
void spiTest()
{
  uint8_t received_val = 0;
  GPIOE->BSRRH |= GPIO_Pin_7; // set PE7 (CS) low
  spi1Send(0xAA);  // transmit data
  received_val = spi1Send(0x00); // transmit dummy byte and receive data
  GPIOE->BSRRL |= GPIO_Pin_7; // set PE7 (CS) high
  
}

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
 */
uint8_t spi1Send(uint8_t data)
{
  SPI1->DR = data;
  while( !(SPI1->SR & SPI_I2S_FLAG_TXE) );
  while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) );
  while( SPI1->SR & SPI_I2S_FLAG_BSY);
  return SPI1->DR;
}

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

/** 
 *  @brief  USART1 peripheral initizialition.
 *          Options for non USART1 is TBD.
 *  @param  baudrate Specifies the baud.
 *          Valid baud are: TDB.
 *  @retval None.
 */
void UsartInit(uint32_t baud)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
 
  // Enable APB2 peripheral clock for USART1  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
  // Enable AHB1 peripheral clock for USART1 pins (PB6 and PB7)
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  
  // Set up pins PB6 and PB7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
  
  // Set up USART parameters
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = 
    USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART1, &USART_InitStructure);
  
  // Setup USART RX interrupts
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable complete USART1 peripheral
  USART_Cmd(USART1, ENABLE);
}

/** @brief USARTx string transmit function.
 *  @param USARTx USART1 to USART6.
 *  @param *s Pointer to first char in string.
 *  @retval None.
 */
void UsartPuts(USART_TypeDef* USARTx, volatile char *s)
{
  while(*s){
    /* Check TX Buffer Empty flag and wait until flag is set.
     * TXE = 0 indicates that internal TX buffer is not ready to 
     * be loaded with next data. Can also use 
     * USART_SendData(USARTx, *s) here.
     */
    while( (USARTx->SR & USART_SR_TXE) == 0 );//0x00000040));
    USARTx->DR = (*s & (uint16_t)0x01FF);
    *s++;
  }
}

/** 
 *  @brief USARTx char transmit function.
 *  @param USARTx USART1 to USART6.
 *  @param Data Character to be transmitted.
 *  @retval None.
 */
void UsartPutchar(USART_TypeDef* USARTx, uint16_t Data)
{
  while(!(USARTx -> SR & USART_SR_TXE));
  USARTx->DR = (Data & (uint16_t)0x01FF);
}

/** 
 *  @brief USARTx number transmit function.
 *         NB! DOES NOT WORK PROPERLY. TBF.
 *  @param USARTx USART1 to USART6.
 *  @param num Number to be transmitted.
 *  @retval None.
 */
void UsartPutn(USART_TypeDef* USARTx, uint32_t num)
{
  char value[10];
  int i = 0;

  do {
    value[i++] = (char)(num % 10) + '0';
    num /= 10;
  } while(num);

  while(i) {
    UsartPuts(USART1, &value[--i]);
  }
}

/*
 * USART interrupt handler 
 */
void UsartIRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE)) {
    static uint8_t cnt = 0;
    char t = USART1->DR;
    
    if((t != '\n') && (cnt < MAX_STRLEN)) {
      received_string[cnt] = t;
      cnt++;
    } else {
      cnt = 0;
      UsartPuts(USART1, received_string);
    }
  }
}

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
