#ifndef ATTITUDE_CONTROL_H_
#define ATTITUDE_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "AttitudeEstimator.h"

/**
 * 控制输入结构体（面向四轴运动语义）
 * lift: 上升/下降（总推力）[1000, 2000]
 * forward: 前进/后退指令 [1000, 2000]（内部映射为pitch目标）
 * lateral: 左右横移指令 [1000, 2000]（内部映射为roll目标）
 * yaw: 偏航指令 [1000, 2000]（摇杆偏转为转向，回中后保持偏航角）
 */
typedef struct {
    uint16_t lift;
    uint16_t forward;
    uint16_t lateral;
    uint16_t yaw;
    bool armed;
} ControlInput_t;

/**
 * 电机输出结构体（us）
 */
typedef struct {
    uint16_t m1; /* 前左  (CCW) */
    uint16_t m2; /* 前右  (CW)  */
    uint16_t m3; /* 后右  (CCW) */
    uint16_t m4; /* 后左  (CW)  */
} MotorOutput_t;

/**
 * 姿态目标结构体
 */
typedef struct {
    float roll;         /* deg */
    float pitch;        /* deg */
    float yawRate;      /* deg/s */
} AttitudeSetpoint_t;

/**
 * 姿态状态结构体
 */
typedef struct {
    float roll;         /* deg */
    float pitch;        /* deg */
    float yaw;          /* deg */
    float rollRate;     /* deg/s */
    float pitchRate;    /* deg/s */
    float yawRate;      /* deg/s */
} AttitudeState_t;

/**
 * 姿态控制器结构体（串级PID）
 */
typedef struct {
    PidObject rollAnglePid;
    PidObject pitchAnglePid;
    PidObject rollRatePid;
    PidObject pitchRatePid;
    PidObject yawRatePid;

    float dt;

    float rollRateSp;
    float pitchRateSp;
    float yawRateSp;

    float rollOut;
    float pitchOut;
    float yawOut;
} AttitudeController_t;

typedef enum {
    PID_PROFILE_SAFE = 0,
    PID_PROFILE_NORMAL,
    PID_PROFILE_AGILE
} PidProfile_t;

/**
 * @brief 初始化姿态控制器
 */
void AttitudeController_Init(AttitudeController_t* ctrl, float dt);

/**
 * @brief 一键加载PID参数档位
 */
void AttitudeController_LoadProfile(AttitudeController_t* ctrl, PidProfile_t profile);

/**
 * @brief 重置姿态控制器积分项
 */
void AttitudeController_Reset(AttitudeController_t* ctrl, const AttitudeState_t* state);

/**
 * @brief 根据遥控器输入生成姿态目标
 */
void AttitudeController_GenerateSetpoint(const ControlInput_t* input, AttitudeSetpoint_t* sp);

/**
 * @brief 更新姿态控制器（串级）
 * @param ctrl 控制器
 * @param sp 姿态目标
 * @param state 当前姿态状态
 * @param isAcro true: 角速度模式 false: 角度模式
 */
void AttitudeController_Update(AttitudeController_t* ctrl,
                               const AttitudeSetpoint_t* sp,
                               const AttitudeState_t* state,
                               bool isAcro);

/**
 * @brief 四旋翼X构型混控
 */
void AttitudeController_MixToMotor(const AttitudeController_t* ctrl,
                                   uint16_t throttle,
                                   MotorOutput_t* motor);

/**
 * @brief 输出电机PWM
 */
void Motor_WriteOutput(const MotorOutput_t* motor);

#endif /* ATTITUDE_CONTROL_H_ */
