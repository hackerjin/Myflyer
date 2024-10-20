#ifndef __IIC_H
#define __IIC_H
#include "stm32f4xx_hal.h"


void I2C1_Init(void);
uint8_t get_device_id();

uint8_t read_bytes(uint16_t dev_addr, uint16_t mem_addr,uint16_t size,uint8_t* buffer);
uint8_t read_byte(uint16_t dev_addr, uint16_t mem_addr,uint8_t* data);
uint8_t read_bits(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start, uint8_t bit_length,uint8_t* data);
uint8_t read_bit(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start,uint8_t* data);


uint8_t write_bytes(uint16_t dev_addr, uint16_t mem_addr,uint16_t size,uint8_t* buffer);
uint8_t write_byte(uint16_t dev_addr, uint16_t mem_addr,uint8_t data);
uint8_t write_bits(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start, uint8_t bit_length,uint8_t data);
uint8_t write_bit(uint16_t dev_addr, uint16_t mem_addr,uint8_t bit_start,uint8_t data);

#endif



