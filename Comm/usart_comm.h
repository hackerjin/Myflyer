#ifndef _USART_COMM_H
#define _USART_COMM_H
#include <stdbool.h>
#include <stdint.h>


#define UARTSLK_TYPE             USART2	
#define UARTSLK_PERIF            RCC_APB1Periph_USART2
#define ENABLE_UARTSLK_RCC       RCC_APB1PeriphClockCmd
#define UARTSLK_IRQ              USART2_IRQn

#define UARTSLK_DMA_IRQ          DMA1_Stream6_IRQn	
#define UARTSLK_DMA_IT_TC        DMA_IT_TC
#define UARTSLK_DMA_STREAM       DMA1_Stream6
#define UARTSLK_DMA_CH           DMA_Channel_4
#define UARTSLK_DMA_IT_TCIF    	 DMA_IT_TCIF6

#define UARTSLK_GPIO_PERIF       RCC_AHB1Periph_GPIOA 
#define UARTSLK_GPIO_PORT        GPIOA
#define UARTSLK_GPIO_TX_PIN      GPIO_Pin_2
#define UARTSLK_GPIO_RX_PIN      GPIO_Pin_3
#define UARTSLK_GPIO_AF_TX_PIN   GPIO_PinSource2
#define UARTSLK_GPIO_AF_RX_PIN   GPIO_PinSource3
#define UARTSLK_GPIO_AF_TX       GPIO_AF_USART2
#define UARTSLK_GPIO_AF_RX       GPIO_AF_USART2

#define UARTSLK_TXEN_PERIF       RCC_AHB1Periph_GPIOA
#define UARTSLK_TXEN_PORT        GPIOA
#define UARTSLK_TXEN_PIN         GPIO_Pin_0
#define UARTSLK_TXEN_EXTI        EXTI_Line0

bool get_data_from_usart(uint8_t *c);
void usart_dma_send_nrf( uint8_t* data,uint32_t size);
#endif