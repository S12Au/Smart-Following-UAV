#ifndef GY86_H
#define GY86_H
#include "FreeRTOS.h"
#include "Queue.h"

#define MPU6050_DEVICE_ADRESS 0x68		//MPU6050设备地址
#define MPU6050_PWR_MGMT_1 0x6B  		// 电源管理1寄存器
#define MPU6050_PWR_MGMT_2 0x6C  		// 电源管理2寄存器
#define MPU6050_SMPLRT_DIV 0x19			//配置陀螺仪输出速率的分频系数
#define MPU6050_CONFIG 0x1A				//配置数字低通滤波器(DLPF)的带宽
#define MPU6050_GYRO_CONFIG 0x1B			//设置陀螺仪的自检功能和量程
#define MPU6050_ACCEL_CONFIG 0x1C		//设置加速度计的自检功能、量程和高速滤波器
#define MPU6050_INT_ENABLE 0x38			//中断使能
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_TEMP_OUT_L   0x42
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_GYRO_XOUT_L  0x44
#define MPU6050_REG_GYRO_YOUT_H  0x45
#define MPU6050_REG_GYRO_YOUT_L  0x46
#define MPU6050_REG_GYRO_ZOUT_H  0x47
#define MPU6050_REG_GYRO_ZOUT_L  0x48

// MPU6050辅助I2C相关寄存器定义
#define MPU6050_USER_CTRL       0x6A    // 用户控制寄存器
#define MPU6050_INT_PIN_CFG     0x37    // INT引脚/旁路使能配置寄存器
#define MPU6050_I2C_MST_CTRL    0x24    // I2C主控模式控制寄存器
#define MPU6050_I2C_SLV0_ADDR   0x25    // I2C从机0地址寄存器
#define MPU6050_I2C_SLV0_REG    0x26    // I2C从机0寄存器地址
#define MPU6050_I2C_SLV0_CTRL   0x27    // I2C从机0控制寄存器
#define MPU6050_I2C_SLV0_DO     0x63    // I2C从机0写数据寄存器
#define MPU6050_EXT_SENS_DATA_00 0x49   // 外部传感器数据寄存器00

#define HMC5883L_ADDRESS 0x1E 			// HMC5883L的I2C地址（7位地址）
#define HMC5883L_REG_CONFA 0x00 			// 配置寄存器A
#define HMC5883L_REG_CONFB 0x01 			// 配置寄存器B
#define HMC5883L_REG_MODE 0x02 			// 模式寄存器
#define HMC5883L_X_OUT_H 0x03 	
#define HMC5883L_X_OUT_L 0x04
#define HMC5883L_Z_OUT_H 0x05
#define HMC5883L_Z_OUT_L 0x06
#define HMC5883L_Y_OUT_H 0x07
#define HMC5883L_Y_OUT_L 0x08
#define HMC5883L_MODE_CONTINOUS 0x00 	// 连续模式
#define HMC5883L_AVERAGING_8 0x40 		// 8次平均值
#define HMC5883L_RATE_15 0x03 			// 15Hz的数据输出率

#define MS5611_DEVICE_ADRESS 0X77
#define MS5611_CMD_RESET         0x1E    // 复位命令
/* PROM读取命令 - 工厂校准系数 */
#define MS5611_CMD_READ_PROM     0xA0    // PROM读取起始地址
#define MS5611_PROM_C1           (MS5611_CMD_READ_PROM + 0)  // 压力灵敏度
#define MS5611_PROM_C2           (MS5611_CMD_READ_PROM + 2)  // 压力偏移
#define MS5611_PROM_C3           (MS5611_CMD_READ_PROM + 4)  // 温度压力系数
#define MS5611_PROM_C4           (MS5611_CMD_READ_PROM + 6)  // 温度偏移系数
#define MS5611_PROM_C5           (MS5611_CMD_READ_PROM + 8)  // 参考温度
#define MS5611_PROM_C6           (MS5611_CMD_READ_PROM + 10) // 温度灵敏度
// 气压转换（不同过采样率）
#define MS5611_CMD_ADC_READ      0x00    // ADC读取命令
#define MS5611_CMD_CONV_D1_256   0x40    // 气压转换，OSR=256
#define MS5611_CMD_CONV_D1_512   0x42    // 气压转换，OSR=512
#define MS5611_CMD_CONV_D1_1024  0x44    // 气压转换，OSR=1024
#define MS5611_CMD_CONV_D1_2048  0x46    // 气压转换，OSR=2048
#define MS5611_CMD_CONV_D1_4096  0x48    // 气压转换，OSR=4096
// 温度转换（不同过采样率）
#define MS5611_CMD_CONV_D2_256   0x50    // 温度转换，OSR=256
#define MS5611_CMD_CONV_D2_512   0x52    // 温度转换，OSR=512
#define MS5611_CMD_CONV_D2_1024  0x54    // 温度转换，OSR=1024
#define MS5611_CMD_CONV_D2_2048  0x56    // 温度转换，OSR=2048
#define MS5611_CMD_CONV_D2_4096  0x58    // 温度转换，OSR=4096

extern uint16_t c[7];
void IIC_Write(uint8_t addr,uint8_t RegAddr,uint8_t data);   	//iic写取一个字节
void IIC_Write_MS5611(uint8_t addr, uint8_t cmd);					//MS5611写指令
uint8_t IIC_Read(uint8_t addr,uint8_t RegAddr);						//iic读取一个字节
uint32_t MS5611_Read(uint8_t cmd);										//MS5611读取气压或温度数据
void MPU6050_Init();
uint8_t MPU6050_Check();													
void HMC5883L_Init();
uint8_t HMC5883L_CheckID();  // 检查HMC5883L ID
void HMC5883L_WriteReg(uint8_t reg, uint8_t data);  // 通过MPU6050辅助I2C写入HMC5883L寄存器
void MS5611_Init();
void MPU6050_Read_GYRO(int16_t *arr);
void MPU6050_Read_ACCEL(int16_t *arr);
uint32_t MS5611_Read_Pressure();											//读取并计算数据，最终返回气压值
void HMC5883L_Read_MAG(int16_t *arr);									//读取磁力计的值
void Get_GYROandACCEL();
void Get_Pressure();
void Get_MAG();


extern QueueHandle_t QueueGYROACCEL;
extern QueueHandle_t QueueMAG;
extern QueueHandle_t QueuePressure;

struct GYRO_ACCEL_Data {
	int16_t gyro[3];
	int16_t accel[3];
};

struct MAG_Data {
	int16_t mag[3];
};

struct Pressure_Data {
	uint32_t pressure;
};
#endif
