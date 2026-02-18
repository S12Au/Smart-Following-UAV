/**
 * physicalConstants.h - 物理常数定义
 */
#ifndef PHYSICAL_CONSTANTS_H_
#define PHYSICAL_CONSTANTS_H_

#include <math.h>

/* 数学常数 */
#ifndef M_PI
#define M_PI            3.14159265358979323846f
#endif

#ifndef M_PI_F
#define M_PI_F          3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2          1.57079632679489661923f
#endif

#ifndef M_PI_4
#define M_PI_4          0.78539816339744830962f
#endif

/* 重力加速度 (m/s^2) */
#define GRAVITY_MAGNITUDE   9.80665f

/* 角度转弧度 */
#define DEG_TO_RAD          (M_PI_F / 180.0f)

/* 弧度转角度 */
#define RAD_TO_DEG          (180.0f / M_PI_F)

/* 标准大气压 (Pa) */
#define SEA_LEVEL_PRESSURE  101325.0f

/* 空气温度递减率 (K/m) */
#define TEMPERATURE_LAPSE_RATE  0.0065f

/* 标准海平面温度 (K) */
#define SEA_LEVEL_TEMPERATURE   288.15f

/* MPU6050 灵敏度参数 */
/* 陀螺仪量程 ±2000°/s，灵敏度 16.4 LSB/(°/s) */
#define GYRO_SENSITIVITY_2000   16.4f

/* 加速度计量程 ±2g，灵敏度 16384 LSB/g */
#define ACCEL_SENSITIVITY_2G    16384.0f

/* 加速度计量程 ±4g，灵敏度 8192 LSB/g */
#define ACCEL_SENSITIVITY_4G    8192.0f

/* 加速度计量程 ±8g，灵敏度 4096 LSB/g */
#define ACCEL_SENSITIVITY_8G    4096.0f

/* 加速度计量程 ±16g，灵敏度 2048 LSB/g */
#define ACCEL_SENSITIVITY_16G   2048.0f

#endif /* PHYSICAL_CONSTANTS_H_ */
