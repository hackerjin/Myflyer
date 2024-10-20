#ifndef __SPL06_H
#define __SPL06_H
#include "stm32f4xx.h"

#define SPL06_ADDR 0xec
#define SPL_ID_ADDR 0X0d

#define SPL06_PRESSURE_MSB_REG			(0x00)  /* Pressure MSB Register */
#define SPL06_PRESSURE_LSB_REG			(0x01)  /* Pressure LSB Register */
#define SPL06_PRESSURE_XLSB_REG			(0x02)  /* Pressure XLSB Register */
#define SPL06_TEMPERATURE_MSB_REG		(0x03)  /* Temperature MSB Reg */
#define SPL06_TEMPERATURE_LSB_REG		(0x04)  /* Temperature LSB Reg */
#define SPL06_TEMPERATURE_XLSB_REG		(0x05)  /* Temperature XLSB Reg */
#define SPL06_PRESSURE_CFG_REG			(0x06)	/* Pressure configuration Reg */
#define SPL06_TEMPERATURE_CFG_REG		(0x07)	/* Temperature configuration Reg */
#define SPL06_MODE_CFG_REG				(0x08)  /* Mode and Status Configuration */
#define SPL06_INT_FIFO_CFG_REG			(0x09)	/* Interrupt and FIFO Configuration */
#define SPL06_INT_STATUS_REG			(0x0A)	/* Interrupt Status Reg */
#define SPL06_FIFO_STATUS_REG			(0x0B)	/* FIFO Status Reg */
#define SPL06_RST_REG					(0x0C)  /* Softreset Register */
#define SPL06_COEFFICIENT_CALIB_REG		(0x10)  /* Coeffcient calibraion Register */

#define SPL06_CALIB_COEFFICIENT_LENGTH	(18)
#define SPL06_DATA_FRAME_SIZE			(6)

#define SPL06_CONTINUOUS_MODE			(0x07)

#define TEMPERATURE_INTERNAL_SENSOR		(0)
#define TEMPERATURE_EXTERNAL_SENSOR		(1)

//�������� times / S
#define SPL06_MWASURE_1					(0x00)
#define SPL06_MWASURE_2					(0x01)
#define SPL06_MWASURE_4					(0x02)
#define SPL06_MWASURE_8					(0x03)
#define SPL06_MWASURE_16				(0x04)
#define SPL06_MWASURE_32				(0x05)
#define SPL06_MWASURE_64				(0x06)
#define SPL06_MWASURE_128				(0x07)

//��������
#define SPL06_OVERSAMP_1				(0x00)
#define SPL06_OVERSAMP_2				(0x01)
#define SPL06_OVERSAMP_4				(0x02)
#define SPL06_OVERSAMP_8				(0x03)
#define SPL06_OVERSAMP_16				(0x04)
#define SPL06_OVERSAMP_32				(0x05)
#define SPL06_OVERSAMP_64				(0x06)
#define SPL06_OVERSAMP_128				(0x07)


#define P_MEASURE_RATE 			SPL06_MWASURE_16 	//ÿ���������
#define P_OVERSAMP_RATE 		SPL06_OVERSAMP_64	//��������
#define SPL06_PRESSURE_CFG		(P_MEASURE_RATE<<4 | P_OVERSAMP_RATE)

#define T_MEASURE_RATE 			SPL06_MWASURE_16 	//ÿ���������
#define T_OVERSAMP_RATE 		SPL06_OVERSAMP_8	//��������
#define SPL06_TEMPERATURE_CFG	(TEMPERATURE_EXTERNAL_SENSOR<<7 | T_MEASURE_RATE<<4 | T_OVERSAMP_RATE)

#define SPL06_MODE				(SPL06_CONTINUOUS_MODE)


typedef enum 
{
	PRESURE_SENSOR, 
	TEMPERATURE_SENSOR
}spl06Sensor_e;

typedef struct 
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06CalibCoefficient_t;

void spl0601_get_calib_param(void);
void spl0601_rateset(spl06Sensor_e sensor, uint8_t measureRate, uint8_t oversamplRate);
float spl0601_get_temperature(int32_t rawTemperature);
float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature);

void SPL06GetData(float* pressure, float* temperature, float* asl);
void pressureFilter(float* in, float* out);/*�޷�ƽ���˲���*/
float SPL06PressureToAltitude(float pressure/*, float* groundPressure, float* groundTemp*/);

#endif


