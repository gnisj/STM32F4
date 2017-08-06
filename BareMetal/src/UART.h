#ifndef __USART_H
#define __USART_H

void USART_putn(USART_TypeDef* USARTx, uint32_t num);
void init_USART(uint32_t baudrate);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USART_putchar(USART_TypeDef* USARTx, uint16_t Data);

#endif
