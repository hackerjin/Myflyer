#include "stdio.h"
#include "usart.h"

UART_HandleTypeDef usart1;
UART_HandleTypeDef usart2;
DMA_HandleTypeDef usart2_dma;



#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/***********************************************END*******************************************/

void usart1_init()
{
    usart1.Instance = USART1;
    usart1.Init.BaudRate = 9600;
    usart1.Init.WordLength = UART_WORDLENGTH_8B;
    usart1.Init.StopBits = UART_STOPBITS_1;
    usart1.Init.Parity = UART_PARITY_NONE;
    usart1.Init.Mode = UART_MODE_TX_RX;
    usart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    usart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&usart1);
    
}


uint8_t nrf_data;

void usart2_init(void)
{

    usart2.Instance = USART2;
    usart2.Init.BaudRate = 1000000;
    usart2.Init.WordLength = UART_WORDLENGTH_8B;
    usart2.Init.StopBits = UART_STOPBITS_1;
    usart2.Init.Parity = UART_PARITY_NONE;
    usart2.Init.Mode = UART_MODE_TX_RX;
    usart2.Init.HwFlowCtl = UART_HWCONTROL_CTS;
    usart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&usart2);
    
    HAL_UART_Receive_IT(&usart2,&nrf_data,1);

}




void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {

    
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
      

    __HAL_RCC_DMA1_CLK_ENABLE();

    
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //串口2的DMA初始化，后续使用DMA传输数据到NRF
    usart2_dma.Instance = DMA1_Stream6;
    usart2_dma.Init.Channel = DMA_CHANNEL_4;
    usart2_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    usart2_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    usart2_dma.Init.MemInc = DMA_MINC_ENABLE;
    usart2_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    usart2_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    usart2_dma.Init.Mode = DMA_NORMAL;
    usart2_dma.Init.Priority = DMA_PRIORITY_HIGH;
    usart2_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    usart2_dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    usart2_dma.Init.MemBurst = DMA_MBURST_SINGLE;
    usart2_dma.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&usart2_dma);
    
    __HAL_LINKDMA(huart,hdmatx,usart2_dma);

    //串口2的接收中断使能，用来接收来自NRF的数据
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    
    
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  }
  else if(huart->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
      
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_3;    
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
  }

}


void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {

    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA0-WKUP     ------> USART2_CTS
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&usart2);
    
}

extern void fill_queue(uint8_t data);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
       
        fill_queue(nrf_data);
        HAL_UART_Receive_IT(&usart2,&nrf_data,1);
    }
}


