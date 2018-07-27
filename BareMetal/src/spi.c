#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "spi.h"


uint8_t txbuf[4];
uint8_t rxbuf[4];
uint16_t txbuf16[4];
uint16_t rxbuf16[4];


static const uint16_t speeds[] = {
  [SPI_SLOW] = SPI_BaudRatePrescaler_64,
  [SPI_MEDIUM] = SPI_BaudRatePrescaler_8,
  [SPI_FAST] = SPI_BaudRatePrescaler_2 };


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


uint8_t eepromReadStatus()
{
  //uint8_t cmd[] = {cmdRDSR, 0xff};
  uint8_t res[2];
  //https://github.com/nalepae/stm32_tutorial/tree/master/src
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

/*
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
*/
