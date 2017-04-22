#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_conf.h"
#include "utils.h"
#include <misc.h>
//#include "usbd_cdc_core.h"
//#include "usbd_usr.h"
//#include "usbd_desc.h"
//#include "usbd_cdc_vcp.h"
//#include "usbd_dcd_int.h"



#define MAX_STRLEN 12
// Private variables
volatile uint32_t time_var1, time_var2;
volatile char received_string[MAX_STRLEN+1];

//void Delay(__IO uint32_t time);
//extern __IO uint32_t TimingDelay;


// Private function prototypes
void init();

/*
void Delay(__IO uint32_t time) {
  TimingDelay = time;
  while(TimingDelay != 0);
}
*/

int main(void) {
  init();
  init_USART(9600);
  //USART_puts(USART1, "Init completed successfully.\r\n");
  //for(int j=2; j<8; j++) {
  //USART_putchar(USART1, 1);
    //delay_ms(1);
    //for(int k=0; k<8000000; k++);
  //}
    // USART_putchar(USART1, 2);
  //USART_puts(USART1, "AAA");
  volatile int i;
  while(1){
   
    GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
    GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
    USART_puts(USART1, "ABC\r\n");
    //USART_putchar(USART1, "ABC\r\n");

    for(i=0; i<10000000; i++);
  }
  return 0;
}

void init() {
  //SystemInit();
  //SysTick_Config(SystemCoreClock / 1000);
  
  //USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);

  // Enable peripheral clock on GPIOD
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  

  // Initialize struct 
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /*** LEDS ***/
  // Configure PD12, PD13, PD14 and PD15 as push-pull outputs
  GPIO_InitStructure.GPIO_Pin = 
    GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  // Initialize pins on GPIOD
  GPIO_Init(GPIOD, &GPIO_InitStructure);

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
  //GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  // ---------- SysTick timer -------- //
  if (SysTick_Config(SystemCoreClock / 1000)) {
    // Capture error
    while (1){};
  }
}


/*
 * This function initializes the USART1 peripheral.
 * Argument: baudrate specifies the, well, baudrate.
 */
void init_USART(uint32_t baudrate) {
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
  USART_InitStructure.USART_BaudRate = baudrate;
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

void USART_puts(USART_TypeDef* USARTx, volatile char *s) {
  while(*s){
    // Wait until data register is empty
    while(!(USARTx->SR & 0x00000040));
    USART_SendData(USARTx, *s);
    *s++;
  }
}

/*
 * Basic UART transmit procedure that polls the 
 * USART_FLAG_TXE flag.NB! DOES NOT WORK (YET)
 */
int USART_putchar(USART_TypeDef* USARTx, int c) {
  while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
  USARTx->DR = (c & 0xff);
  return 0;
}


void USART1_IRQHandler(void){
  if(USART_GetITStatus(USART1, USART_IT_RXNE)) {
    static uint8_t cnt = 0;
    char t = USART1->DR;
    
    if((t != '\n') && (cnt < MAX_STRLEN)) {
      received_string[cnt] = t;
      cnt++;
    } else {
      cnt = 0;
      USART_puts(USART1, received_string);
    }
  }
}


/*
 * Called from systick handler
 */
void timing_handler() {
  if (time_var1) {
    time_var1--;
  }

  time_var2++;
}

void delay_ms(int ms) {
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
 * Dummy function to avoid compiler error
 */
void _init() {

}

