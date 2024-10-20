#include "iic.h"



I2C_HandleTypeDef i2c1;


void I2C1_Init(void)
{

  
  i2c1.Instance = I2C1;
  i2c1.Init.ClockSpeed = 400000;
  i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  i2c1.Init.OwnAddress1 = 0;
  i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  i2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  i2c1.Init.OwnAddress2 = 0;
  i2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  i2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&i2c1);
 
}





/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(hi2c->Instance==I2C1)
  {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    
    
  
  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

}


uint8_t read_bytes( uint16_t dev_addr, uint16_t mem_addr,uint16_t size,uint8_t* buffer)
{
    
    if(HAL_I2C_Mem_Read(&i2c1,dev_addr,mem_addr,I2C_MEMADD_SIZE_8BIT,buffer,size,1000) == HAL_OK)
        return 1;
    else
        return 0;
    
}

uint8_t read_byte(uint16_t dev_addr, uint16_t mem_addr,uint8_t* data)
{
    
    if(HAL_I2C_Mem_Read(&i2c1,dev_addr,mem_addr,I2C_MEMADD_SIZE_8BIT,data,1,10000) == HAL_OK)
        return 1;
    else
        return 0;
    
}

uint8_t read_bits(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start, uint8_t bit_length,uint8_t* data)
{
    
    uint8_t byte = 0;
    
    if(HAL_I2C_Mem_Read(&i2c1,dev_addr,mem_addr,I2C_MEMADD_SIZE_8BIT,&byte,1,1000) == HAL_OK)
    {
    
        uint8_t mask = ((1 << bit_length) - 1) << (bit_start - bit_length + 1);
        byte &= mask;
        byte >>= (bit_start - bit_length + 1);   
        *data = byte;
        return 1;
        
    }
    else
        return 0; 
}

uint8_t read_bit(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start,uint8_t* data)
{
	uint8_t byte;
	
    if(read_byte(dev_addr,mem_addr,&byte))
    {
        *data = byte & (1 << bit_start);
        return 1;
    }
    else
        return 0;

}
    
uint8_t write_bytes(uint16_t dev_addr, uint16_t mem_addr,uint16_t size,uint8_t* buffer)
{
    
    if(HAL_I2C_Mem_Write(&i2c1,dev_addr,mem_addr,I2C_MEMADD_SIZE_8BIT,buffer,size,1000) == HAL_OK)
        return 1;
    else
        return 0;
    
}

uint8_t write_byte(uint16_t dev_addr, uint16_t mem_addr,uint8_t data)
{
    if(HAL_I2C_Mem_Write(&i2c1,dev_addr,mem_addr,I2C_MEMADD_SIZE_8BIT,&data,1,1000) == HAL_OK)
        return 1;
    else
        return 0;
}



uint8_t write_bits(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start, uint8_t bit_length,uint8_t data)
{
    uint8_t byte;
    
    if(read_byte(dev_addr,mem_addr,&byte))
    {
        uint8_t mask = ((1 << bit_length) - 1) << (bit_start - bit_length + 1);
		data <<= (bit_start - bit_length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		byte &= ~(mask); // zero all important bits in existing byte
		byte |= data; // combine data with existing byte
        if(write_byte(dev_addr,mem_addr,byte))
            return 1;
        else
            return 0;
    }
    else
        return 0;  
}

uint8_t write_bit(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start,uint8_t data)
{
     
	uint8_t byte;
	
    if(read_byte(dev_addr,mem_addr,&byte))
    {
         byte = (data != 0) ? (byte | (1 << bit_start)) : (byte & ~(1 << bit_start));
        if(write_byte(dev_addr,mem_addr,byte))
            return 1;
        else
            return 0;
    }
    else
        return 0;
}



