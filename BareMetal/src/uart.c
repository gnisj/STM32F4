#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "uart.h"

#define MAX_STRLEN 12

volatile char received_string[MAX_STRLEN+1];

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
