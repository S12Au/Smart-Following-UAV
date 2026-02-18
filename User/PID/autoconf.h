/**
 * autoconf.h - 自动配置文件
 * 定义编译时配置选项
 */
#ifndef AUTOCONF_H_
#define AUTOCONF_H_

/**
 * 控制器配置选项
 */

/* 是否对整个PID输出进行滤波（而非仅D项滤波）
 * 0: 仅对D项滤波
 * 1: 对整个输出滤波
 */
#define CONFIG_CONTROLLER_PID_FILTER_ALL    0

/* 控制器更新频率 (Hz) */
#define CONFIG_CONTROLLER_RATE_HZ           500

/* 姿态解算更新频率 (Hz) */
#define CONFIG_ATTITUDE_RATE_HZ             500

/* 是否使用四元数姿态解算 */
#define CONFIG_USE_QUATERNION_ATTITUDE      1

/* 是否启用抗积分饱和 */
#define CONFIG_ANTI_WINDUP                  1

/* 电机最小/最大油门值（us） */
#define CONFIG_MOTOR_MIN_THROTTLE           1000
#define CONFIG_MOTOR_MAX_THROTTLE           2000
#define CONFIG_MOTOR_IDLE_THROTTLE          1100

/* 遥控器油门死区 */
#define CONFIG_THROTTLE_MIN                 1050
#define CONFIG_THROTTLE_MAX                 1950

/* 飞行模式 */
#define CONFIG_FLIGHT_MODE_ANGLE            0
#define CONFIG_FLIGHT_MODE_RATE             1
#define CONFIG_DEFAULT_FLIGHT_MODE          CONFIG_FLIGHT_MODE_ANGLE

/* PID 参数档位：0=SAFE, 1=NORMAL, 2=AGILE */
#define CONFIG_PID_PROFILE_DEFAULT          1

/* 角度限制（度） */
#define CONFIG_MAX_ROLL_ANGLE               45.0f
#define CONFIG_MAX_PITCH_ANGLE              45.0f

/* 角速度限制（度/秒） */
#define CONFIG_MAX_ROLL_RATE                360.0f
#define CONFIG_MAX_PITCH_RATE               360.0f
#define CONFIG_MAX_YAW_RATE                 180.0f

/* 遥控输入有效范围 */
#define CONFIG_PPM_MIN_VALID                900
#define CONFIG_PPM_MAX_VALID                2100

/* 安全逻辑 */
#define CONFIG_PPM_FAILSAFE_TIMEOUT_MS      300
#define CONFIG_ARM_HOLD_MS                  1000
#define CONFIG_DISARM_HOLD_MS               600
#define CONFIG_ARM_YAW_HIGH                 1900
#define CONFIG_DISARM_YAW_LOW               1100

/* 串口调试输出频率 */
#define CONFIG_UART_DEBUG_RATE_HZ           20

/* 传感器零偏校准 */
#define CONFIG_SENSOR_CALIB_SAMPLES         1000
#define CONFIG_GYRO_STILL_THRESHOLD_DPS     3.0f
#define CONFIG_ACCEL_STILL_TOL_G            0.15f

#endif /* AUTOCONF_H_ */
