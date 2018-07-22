#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

/* Function prototypes */ 
void UsartInit(uint32_t baud);
void UsartPuts(USART_TypeDef* USARTx, volatile char *s);
void UsartPutchar(USART_TypeDef* USARTx, uint16_t Data);
void UsartPutn(USART_TypeDef* USARTx, uint32_t num);
void UsartIRQHandler(void);

#endif
