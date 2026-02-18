/**
 * attitude_estimator.h - 姿态解算模块
 * 使用Mahony互补滤波算法进行姿态估计
 */
#ifndef ATTITUDE_ESTIMATOR_H_
#define ATTITUDE_ESTIMATOR_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * 欧拉角结构体（度）
 */
typedef struct {
    float roll;     /* 横滚角 [-180, 180] */
    float pitch;    /* 俯仰角 [-90, 90] */
    float yaw;      /* 航向角 [-180, 180] */
} EulerAngle_t;

/**
 * 四元数结构体
 */
typedef struct {
    float q0;   /* w */
    float q1;   /* x */
    float q2;   /* y */
    float q3;   /* z */
} Quaternion_t;

/**
 * 传感器数据结构体（SI单位）
 */
typedef struct {
    float gyro[3];      /* 陀螺仪数据 (rad/s) */
    float accel[3];     /* 加速度计数据 (m/s^2) */
    float mag[3];       /* 磁力计数据 (归一化) */
} SensorData_t;

/**
 * 姿态估计器结构体
 */
typedef struct {
    Quaternion_t q;         /* 当前姿态四元数 */
    EulerAngle_t euler;     /* 当前欧拉角 */
    
    float twoKp;            /* 2 * 比例增益 */
    float twoKi;            /* 2 * 积分增益 */
    float integralFBx;      /* 积分误差 x */
    float integralFBy;      /* 积分误差 y */
    float integralFBz;      /* 积分误差 z */
    
    float dt;               /* 采样周期 (s) */
    bool useMag;            /* 是否使用磁力计 */
    bool initialized;       /* 是否已初始化 */
} AttitudeEstimator_t;

/**
 * @brief 初始化姿态估计器
 * @param estimator 姿态估计器指针
 * @param dt 采样周期 (s)
 * @param kp 比例增益 (典型值: 2.0f)
 * @param ki 积分增益 (典型值: 0.005f)
 * @param useMag 是否使用磁力计
 */
void AttitudeEstimator_Init(AttitudeEstimator_t* estimator, float dt, 
                            float kp, float ki, bool useMag);

/**
 * @brief 更新姿态估计
 * @param estimator 姿态估计器指针
 * @param sensor 传感器数据（SI单位）
 */
void AttitudeEstimator_Update(AttitudeEstimator_t* estimator, 
                              const SensorData_t* sensor);

/**
 * @brief 仅使用IMU更新姿态（无磁力计）
 * @param estimator 姿态估计器指针
 * @param gx, gy, gz 陀螺仪数据 (rad/s)
 * @param ax, ay, az 加速度计数据 (归一化或原始均可)
 */
void AttitudeEstimator_UpdateIMU(AttitudeEstimator_t* estimator,
                                  float gx, float gy, float gz,
                                  float ax, float ay, float az);

/**
 * @brief 获取当前欧拉角
 * @param estimator 姿态估计器指针
 * @return 欧拉角指针
 */
const EulerAngle_t* AttitudeEstimator_GetEuler(AttitudeEstimator_t* estimator);

/**
 * @brief 获取当前四元数
 * @param estimator 姿态估计器指针
 * @return 四元数指针
 */
const Quaternion_t* AttitudeEstimator_GetQuaternion(AttitudeEstimator_t* estimator);

/**
 * @brief 重置姿态估计器
 * @param estimator 姿态估计器指针
 */
void AttitudeEstimator_Reset(AttitudeEstimator_t* estimator);

/**
 * @brief 设置Mahony滤波器增益
 * @param estimator 姿态估计器指针
 * @param kp 比例增益
 * @param ki 积分增益
 */
void AttitudeEstimator_SetGains(AttitudeEstimator_t* estimator, float kp, float ki);

/**
 * @brief 将原始传感器数据转换为SI单位
 * @param rawGyro 原始陀螺仪数据 (LSB)
 * @param rawAccel 原始加速度计数据 (LSB)
 * @param sensor 输出的SI单位传感器数据
 */
void SensorData_ConvertFromRaw(const int16_t rawGyro[3], 
                               const int16_t rawAccel[3],
                               SensorData_t* sensor);

#endif /* ATTITUDE_ESTIMATOR_H_ */
