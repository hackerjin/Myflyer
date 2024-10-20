#include <math.h>
#include "stdbool.h"
#include "spl06.h"
#include "iic.h"
/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * SPL06驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define P_MEASURE_RATE 			SPL06_MWASURE_16 	//每秒测量次数
#define P_OVERSAMP_RATE 		SPL06_OVERSAMP_64	//过采样率
#define SPL06_PRESSURE_CFG		(P_MEASURE_RATE<<4 | P_OVERSAMP_RATE)

#define T_MEASURE_RATE 			SPL06_MWASURE_16 	//每秒测量次数
#define T_OVERSAMP_RATE 		SPL06_OVERSAMP_8	//过采样率
#define SPL06_TEMPERATURE_CFG	(TEMPERATURE_EXTERNAL_SENSOR<<7 | T_MEASURE_RATE<<4 | T_OVERSAMP_RATE)

#define SPL06_MODE				(SPL06_CONTINUOUS_MODE)

const uint32_t scaleFactor[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};


spl06CalibCoefficient_t  spl06Calib;


int32_t kp = 0;
int32_t kt= 0;
int32_t SPL06RawPressure = 0;
int32_t SPL06RawTemperature = 0;


static void SPL06GetPressure(void);


void spl0601_get_calib_param()
{
    uint8_t buffer[SPL06_CALIB_COEFFICIENT_LENGTH] = {0};
	
    read_bytes(SPL06_ADDR, SPL06_COEFFICIENT_CALIB_REG, SPL06_CALIB_COEFFICIENT_LENGTH, buffer);
	
	
	spl06Calib.c0 = (int16_t)buffer[0]<<4 | buffer[1]>>4;
	spl06Calib.c0 = (spl06Calib.c0 & 0x0800) ? (spl06Calib.c0 | 0xF000) : spl06Calib.c0;
	
	spl06Calib.c1 = (int16_t)(buffer[1] & 0x0F)<<8 | buffer[2];
	spl06Calib.c1 = (spl06Calib.c1 & 0x0800) ? (spl06Calib.c1 | 0xF000) : spl06Calib.c1;
	
	spl06Calib.c00 = (int32_t)buffer[3]<<12 | (int32_t)buffer[4]<<4 | (int32_t)buffer[5]>>4;
	spl06Calib.c00 = (spl06Calib.c00 & 0x080000) ? (spl06Calib.c00 | 0xFFF00000) : spl06Calib.c00;
	
	spl06Calib.c10 = (int32_t)(buffer[5] & 0x0F)<<16 | (int32_t)buffer[6]<<8 | (int32_t)buffer[7];
	spl06Calib.c10 = (spl06Calib.c10 & 0x080000) ? (spl06Calib.c10 | 0xFFF00000) : spl06Calib.c10;
	
	spl06Calib.c01 = (int16_t)buffer[8]<<8 | buffer[9];
	spl06Calib.c11 = (int16_t)buffer[10]<<8 | buffer[11];
	spl06Calib.c20 = (int16_t)buffer[12]<<8 | buffer[13];
	spl06Calib.c21 = (int16_t)buffer[14]<<8 | buffer[15];
	spl06Calib.c30 = (int16_t)buffer[16]<<8 | buffer[17];
    
    
}

void spl0601_rateset(spl06Sensor_e sensor, uint8_t measureRate, uint8_t oversamplRate)
{
    uint8_t reg;
	if (sensor == PRESURE_SENSOR)
	{
		kp = scaleFactor[oversamplRate];
        write_byte(SPL06_ADDR,SPL06_PRESSURE_CFG_REG, measureRate<<4 | oversamplRate);
		
		if (oversamplRate > SPL06_OVERSAMP_8)
		{
            read_byte(SPL06_ADDR,SPL06_INT_FIFO_CFG_REG, &reg);
			
			write_byte( SPL06_ADDR,SPL06_INT_FIFO_CFG_REG, reg | 0x04);
		}
	}
	else if (sensor == TEMPERATURE_SENSOR)
	{
		kt = scaleFactor[oversamplRate];
		write_byte(SPL06_ADDR, SPL06_TEMPERATURE_CFG_REG, measureRate<<4 | oversamplRate | 0x80);//Using mems temperature
		if (oversamplRate > SPL06_OVERSAMP_8)
		{
			read_byte(SPL06_ADDR, SPL06_INT_FIFO_CFG_REG, &reg);
			write_byte(SPL06_ADDR, SPL06_INT_FIFO_CFG_REG, reg | 0x08);
		}
	}
    
    
}


void SPL06GetPressure(void)
{
    uint8_t data[SPL06_DATA_FRAME_SIZE];

    read_bytes(SPL06_ADDR , SPL06_PRESSURE_MSB_REG, SPL06_DATA_FRAME_SIZE, data);
	SPL06RawPressure = (int32_t)data[0]<<16 | (int32_t)data[1]<<8 | (int32_t)data[2];
    SPL06RawPressure = (SPL06RawPressure & 0x800000) ? (0xFF000000 | SPL06RawPressure) : SPL06RawPressure;
	
    SPL06RawTemperature = (int32_t)data[3]<<16 | (int32_t)data[4]<<8 | (int32_t)data[5];
	SPL06RawTemperature = (SPL06RawTemperature & 0x800000) ? (0xFF000000 | SPL06RawTemperature) : SPL06RawTemperature;
}

float spl0601_get_temperature(int32_t rawTemperature)
{
    float fTCompensate;
    float fTsc;

    fTsc = rawTemperature / (float)kt;
    fTCompensate =  spl06Calib.c0 * 0.5 + spl06Calib.c1 * fTsc;
    return fTCompensate;
}


float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = rawTemperature / (float)kt;
    fPsc = rawPressure / (float)kp;
    qua2 = spl06Calib.c10 + fPsc * (spl06Calib.c20 + fPsc* spl06Calib.c30);
    qua3 = fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);
	//qua3 = 0.9f *fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);

    fPCompensate = spl06Calib.c00 + fPsc * qua2 + fTsc * spl06Calib.c01 + qua3;
	//fPCompensate = spl06Calib.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06Calib.c01 + qua3;
    return fPCompensate;
}

void SPL06GetData(float* pressure, float* temperature, float* asl)
{
    static float t;
    static float p;
	
	SPL06GetPressure();

	t = spl0601_get_temperature(SPL06RawTemperature);		
	p = spl0601_get_pressure(SPL06RawPressure, SPL06RawTemperature);		

//	pressureFilter(&p,pressure);
	*temperature = (float)t;/*单位度*/
	*pressure = (float)p ;	/*单位hPa*/	
	
	*asl=SPL06PressureToAltitude(*pressure);	/*转换成海拔*/	
}

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float SPL06PressureToAltitude(float pressure/*, float* groundPressure, float* groundTemp*/)
{	
    if(pressure)
    {
		return 44330.f * (powf((1015.7f / pressure), 0.190295f) - 1.0f);
    }
    else
    {
        return 0;
    }
}
