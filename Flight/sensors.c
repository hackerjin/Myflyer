#include "iic.h"
#include "sensors.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdbool.h"
#include "mpu9250.h"
#include "spl06.h"
#include "math.h"


#define SENSORS_GYRO_FS_CFG       MPU9250_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU9250_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU9250_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     MPU9250_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速计采样个数 */

// MPU9250主机模式读取数据 缓冲区长度



#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN       	6
#define SENSORS_BARO_BUFF_LEN		6

#define AK8963_RA_HXL             0x03
#define AK8963_RA_HXH             0x04
#define AK8963_RA_HYL             0x05
#define AK8963_RA_HYH             0x06
#define AK8963_RA_HZL             0x07
#define AK8963_RA_HZH             0x08


typedef struct
{
	Axis3f     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

BiasObj	gyroBiasRunning;
static Axis3f  gyroBias;

static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

static bool isInit = false;
static sensorData_t sensors;
static Axis3i16	gyroRaw;
static Axis3i16	accRaw;
static Axis3i16 magRaw;

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];



uint8_t buffer[29] = {0};

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;


uint64_t num = 0;

void set_clk_source()
{
    
    write_bits(MPU_ADDR ,MPU9250_RA_PWR_MGMT_1,MPU9250_PWR1_CLKSEL_BIT , MPU9250_PWR1_CLKSEL_LENGTH,MPU9250_CLOCK_PLL_XGYRO);
   
}
void enable_temp(bool flag)
{
    
    write_bit(MPU_ADDR ,MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_TEMP_DIS_BIT, !flag);
}

void set_int(bool flag)
{
    
    write_byte(MPU_ADDR ,MPU9250_RA_INT_ENABLE, flag);
}

void set_bypass(bool flag)
{
    write_bit(MPU_ADDR ,MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, flag);
}

void set_gyro_range(uint8_t range)
{
    write_bits(MPU_ADDR ,MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT,MPU9250_GCONFIG_FS_SEL_LENGTH, range);
    
}

void set_acc_range(uint8_t range)
{
 
    
    write_bits(MPU_ADDR, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT,MPU9250_ACONFIG_AFS_SEL_LENGTH, range);
}

void set_acc_dlpf(uint8_t range)
{
  
    write_bits(MPU_ADDR, MPU9250_RA_ACCEL_CONFIG_2, MPU9250_ACONFIG2_DLPF_BIT,MPU9250_ACONFIG2_DLPF_LENGTH, range);
}

void set_sample_rate(uint8_t rate)
{
    write_byte(MPU_ADDR, MPU9250_RA_SMPLRT_DIV, rate);
}

void set_dlpf_mode(uint8_t mode)
{
    write_bits( MPU_ADDR, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT,MPU9250_CFG_DLPF_CFG_LENGTH, mode);
}

void enable_mpu_exti()
{
    GPIO_InitTypeDef gpio_init;
    gpio_init.Pin = GPIO_PIN_4;
    gpio_init.Mode = GPIO_MODE_IT_RISING;
    gpio_init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA,&gpio_init);
    
    HAL_NVIC_SetPriority(EXTI4_IRQn,6,0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    
    
}

static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

void mpu9250_init()
{
  
    printf("初始化开始\n");
    
    sensorsDataReady = xSemaphoreCreateBinary();/*创建传感器数据就绪二值信号量*/
    sensorsBiasObjInit(&gyroBiasRunning);
    
    HAL_Delay(100);
    //复位mpu9250
    write_bit(MPU_ADDR,MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_DEVICE_RESET_BIT, 1);
    //write_byte(MPU_ADDR,MPU9250_RA_PWR_MGMT_1,0x80);   
    HAL_Delay(100);
    
    //读取id
    uint8_t byte= 0;
    read_bits(MPU_ADDR ,MPU_ID_ADDR,6,6,&byte); 
    if(byte == 0x38)
        printf("mpu9250's id is 0x%X\n",byte);
    
    //唤醒
    write_bit(MPU_ADDR,MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, false);  
    //write_byte(MPU_ADDR, MPU9250_RA_PWR_MGMT_1,0X00);
       
    vTaskDelay(100);		
    set_clk_source();
    vTaskDelay(100);
      
    
    enable_temp(true);
    set_int(false);
    set_bypass(true);  
    set_gyro_range(MPU9250_GYRO_FS_2000);
    set_acc_range(MPU9250_ACCEL_FS_16);
    set_acc_dlpf(MPU9250_ACCEL_DLPF_BW_41);
    set_sample_rate(0);  
    set_dlpf_mode(MPU9250_DLPF_BW_98);
    
    
    
}


void spl06_init()
{
    
    uint8_t byte = 0;
    //读取id
    read_byte(SPL06_ADDR,SPL_ID_ADDR,&byte);
    if(byte == 0x10)
        printf("SPL06 ID IS: 0x%X\n",byte);
    
    
    spl0601_get_calib_param();
    spl0601_rateset(PRESURE_SENSOR, SPL06_MWASURE_16, SPL06_OVERSAMP_64);
	spl0601_rateset(TEMPERATURE_SENSOR, SPL06_MWASURE_16, SPL06_OVERSAMP_64);   
	write_byte(SPL06_ADDR, SPL06_MODE_CFG_REG, SPL06_MODE);

}

#define AK8963_RA_WIA             0x00
#define AK8963_MODE_FUSEROM       0x0F
void ak8963_init()
{
    uint8_t byte = 0;
    //读取id
    read_byte(AK8963_ADDR,AK8963_RA_WIA ,&byte); 
    printf("AK8963's ID IS: 0x%X\n",byte);
    printf("初始化完成\n");
    
    write_byte(AK8963_ADDR, 0x0b, 0x01);
    HAL_Delay(100);
    write_byte(AK8963_ADDR, 0x0a, 0x11);
    HAL_Delay(100);
}
void device_init()
{
    mpu9250_init();
    
    spl06_init();
    ak8963_init();
    enable_mpu_exti();
    /*创建传感器数据队列*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
}

void change_mpu_mode()
{
    /*
    write_bits(MPU_ADDR, MPU9250_RA_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_MST_DLY_BIT,
      MPU9250_I2C_SLV4_MST_DLY_LENGTH, 19);  
    set_bypass(false);
    write_bits(MPU_ADDR, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_BIT,
      MPU9250_I2C_MST_CLK_LENGTH, 13);
    write_bit(MPU_ADDR, MPU9250_RA_I2C_MST_CTRL, MPU9250_WAIT_FOR_ES_BIT, true);
    
    write_bit(MPU_ADDR, MPU9250_RA_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_BIT, false);
    */
    
    write_bit(MPU_ADDR, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_LEVEL_BIT, 0);
    write_bit(MPU_ADDR, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_OPEN_BIT, 0);
    write_bit(MPU_ADDR, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_LATCH_INT_EN_BIT, 0);
    write_bit(MPU_ADDR, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_INT_RD_CLEAR_BIT,1);
    
    
    /*
    uint8_t num = 0;
    uint8_t address = 0x80 | AK8963_ADDR;
    
    write_byte( MPU_ADDR, MPU9250_RA_I2C_SLV0_ADDR + num * 3, address);
    write_byte( MPU_ADDR, MPU9250_RA_I2C_SLV0_REG + num * 3, AK8963_RA_ST1);
    write_bits( MPU_ADDR, MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_LEN_BIT,
      MPU9250_I2C_SLV_LEN_LENGTH,SENSORS_MAG_BUFF_LEN);    
    write_bit( MPU_ADDR,  MPU9250_RA_I2C_MST_DELAY_CTRL, num, true);
    write_bit( MPU_ADDR,   MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_EN_BIT,true);
    
    
    num = 1;
    address = 0x80 | SPL06_ADDR;
    write_byte( MPU_ADDR, MPU9250_RA_I2C_SLV0_ADDR + num * 3, address);
    write_byte( MPU_ADDR, MPU9250_RA_I2C_SLV0_REG + num * 3, SPL06_MODE_CFG_REG);
    write_bits( MPU_ADDR, MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_LEN_BIT,
      MPU9250_I2C_SLV_LEN_LENGTH,SENSORS_BARO_STATUS_LEN);    
    write_bit( MPU_ADDR,  MPU9250_RA_I2C_MST_DELAY_CTRL, num, true);
    write_bit( MPU_ADDR,   MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_EN_BIT,true);
    
    num = 2;
    write_byte( MPU_ADDR, MPU9250_RA_I2C_SLV0_ADDR + num * 3, address);
    write_byte( MPU_ADDR, MPU9250_RA_I2C_SLV0_REG + num * 3, SPL06_PRESSURE_MSB_REG);
    write_bits( MPU_ADDR, MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_LEN_BIT,
      MPU9250_I2C_SLV_LEN_LENGTH,SENSORS_BARO_DATA_LEN);    
    write_bit( MPU_ADDR,  MPU9250_RA_I2C_MST_DELAY_CTRL, num, true);
    write_bit( MPU_ADDR,   MPU9250_RA_I2C_SLV0_CTRL + num * 3, MPU9250_I2C_SLV_EN_BIT,true);
    
    
    write_bit( MPU_ADDR,  MPU9250_RA_USER_CTRL, MPU9250_USERCTRL_I2C_MST_EN_BIT, true);
    */
    write_bit( MPU_ADDR,  MPU9250_RA_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, true);
}


/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
/*从队列读取磁力计数据*/
bool sensorsReadMag(Axis3f *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
/*从队列读取气压数据*/
bool sensorsReadBaro(baro_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}


/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}



/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	uint32_t i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}



/**
 * 根据样本计算重力加速度缩放因子
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}

/**
 * 计算陀螺方差
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/*处理气压计数据*/
void processBarometerMeasurements(const uint8_t *buffer)
{
	static float temp;
    static float pressure;
	

    int32_t rawPressure = (int32_t)buffer[0]<<16 | (int32_t)buffer[1]<<8 | (int32_t)buffer[2];
    rawPressure = (rawPressure & 0x800000) ? (0xFF000000 | rawPressure) : rawPressure;
    
    int32_t rawTemp = (int32_t)buffer[3]<<16 | (int32_t)buffer[4]<<8 | (int32_t)buffer[5];
    rawTemp = (rawTemp & 0x800000) ? (0xFF000000 | rawTemp) : rawTemp;
    
    temp = spl0601_get_temperature(rawTemp);
    pressure = spl0601_get_pressure(rawPressure, rawTemp);
    sensors.baro.pressure = pressure / 100.0f;
    sensors.baro.temperature = (float)temp; /*单位度*/
    sensors.baro.asl = SPL06PressureToAltitude(sensors.baro.pressure) * 100.f; //cm
    if(num %  2000 == 0)
    {
        printf("温度为摄氏度%f\n",sensors.baro.temperature);
	}
   
}

#define MAG_GAUSS_PER_LSB		(float)(666.7f)


/*处理磁力计数据*/
void processMagnetometerMeasurements(const uint8_t *buffer)
{
	
    int16_t headingx = (((int16_t) buffer[1]) << 8) | buffer[0];
    int16_t headingy = (((int16_t) buffer[3]) << 8) | buffer[2];
    int16_t headingz = (((int16_t) buffer[5]) << 8) | buffer[4];

    sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
    sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
    sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;	

	
    magRaw.x = headingx;/*用于上传到上位机*/  
    magRaw.y = headingy;
    magRaw.z = headingz;
    
    if(num %  2000 == 0)
    {
        float dir =  atan2(headingy,-headingx)*180/3.14159265358979323846;
        if(dir <0 )
            dir += 360;
        printf("磁力方向是 %f\n",dir);
	}
}

float lpf2pApply(lpf2pData* lpfData, float sample)
{
	float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
	if (!isfinite(delay_element_0)) 
	{
		// don't allow bad values to propigate via the filter
		delay_element_0 = sample;
	}

	float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

	lpfData->delay_element_2 = lpfData->delay_element_1;
	lpfData->delay_element_1 = delay_element_0;
	return output;
}

/*二阶低通滤波*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (uint8_t i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}

/*处理加速计和陀螺仪数据*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*注意传感器读取方向(旋转270°x和y交换)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
    
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

    
	accRaw.x = ax;/*用于上传到上位机*/
	accRaw.y = ay;
	accRaw.z = az;
    
	gyroRaw.x = gx - gyroBias.x;
	gyroRaw.y = gy - gyroBias.y;
	gyroRaw.z = gz - gyroBias.z;

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*计算accScale*/
	}
	
	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*单位 °/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
		

	sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;	/*单位 g(9.8m/s^2)*/
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;

   
    if(num++ %  2000 == 0)
    {
        printf("***********************************************\n");
        printf("角速度 x is %f, y is %f, z is %f\n",sensors.gyro.x,sensors.gyro.y,sensors.gyro.z);
        printf("加速度 x is %f, y is %f, z is %f\n",sensors.acc.x,sensors.acc.y,sensors.acc.z);
    }
    
    applyAxis3fLpf(gyroLpf, &sensors.gyro);
	applyAxis3fLpf(accLpf, &sensors.acc);
    
   

}



void sensors_task(void* arg)
{
   
    device_init();
    change_mpu_mode();
    
    while (1)
	{   
        //如果二值信号量可以获取到，说明在数据已经准备好，因为数据准备好，mpu会产生外部中断调用exti4_callback打开信号量
        //此时就可以读取数据
		if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
		{
			

            read_bytes(MPU_ADDR,MPU9250_RA_ACCEL_XOUT_H,SENSORS_MPU6500_BUFF_LEN, buffer);
			processAccGyroMeasurements(&(buffer[0]));

			read_bytes(AK8963_ADDR,AK8963_RA_HXL, SENSORS_MAG_BUFF_LEN, &buffer[SENSORS_MPU6500_BUFF_LEN]);  
            write_byte(AK8963_ADDR, 0x0a, 0x11);
            processMagnetometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
			
			read_bytes(SPL06_ADDR,SPL06_PRESSURE_MSB_REG, SENSORS_BARO_BUFF_LEN, &buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN]);
            processBarometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN]));
			
			
			vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
			xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
			xQueueOverwrite(gyroDataQueue, &sensors.gyro);
			
            xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
        
            xQueueOverwrite(barometerDataQueue, &sensors.baro);
			
			xTaskResumeAll();
		}
	}	
}

void EXTI4_Callback()
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}
