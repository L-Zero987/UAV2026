#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "dji_motor.hpp"

namespace Config_n
{
/*============================== 代码层配置 ==============================*/
#define IS_DEBUG_MODE 0u // 是否是debug模式，0为否，1为是，开启debug模式后，常规任务将被关闭，开启debug任务，一些debug变量也被赋予内存，方便ozone查看


/*============================== 设备层配置 ==============================*/
/* region 1.发射机构 */
// region 摩擦轮
#define FRICTION_MOTOR_NUMS 3           // 摩擦轮电机数量
#define FRICTION_WHEEL_RADIUS 0.06f     // 轮半径，单位m
// endregion
// region 下摩擦轮
#define FRICTION_WHEEL_DOWN_PID_KP 6.0f
#define FRICTION_WHEEL_DOWN_PID_KI 0.0f
#define FRICTION_WHEEL_DOWN_PID_KD 5.0f
#define FRICTION_WHEEL_DOWN_ID     0x01u
// endregion
// region 左摩擦轮
#define FRICTION_WHEEL_L_PID_KP    8.0f
#define FRICTION_WHEEL_L_PID_KI    0.0f
#define FRICTION_WHEEL_L_PID_KD    5.0f
#define FRICTION_WHEEL_L_ID        0x02u
// endregion
// region 右摩擦轮
#define FRICTION_WHEEL_R_PID_KP    10.0f
#define FRICTION_WHEEL_R_PID_KI    0.0f
#define FRICTION_WHEEL_R_PID_KD    5.0f
#define FRICTION_WHEEL_R_ID        0x03u
// endregion
// region 拨弹盘
#define RELOADER_MOTOR_PID_A_KP    0.15f
#define RELOADER_MOTOR_PID_A_KI    0.0f
#define RELOADER_MOTOR_PID_A_KD    0.67f
#define RELOADER_MOTOR_PID_S_KP    10.0f
#define RELOADER_MOTOR_PID_S_KI    0.0f
#define RELOADER_MOTOR_PID_S_KD    2.0f
#define RELOADER_MOTOR_ID          0x04u
// endregion
// endregion

/* region 2.云台 */
// region yaw GM6020
#define YAW_MOTOR_PID_A_KP 6.0f
#define YAW_MOTOR_PID_A_KI 0.0f
#define YAW_MOTOR_PID_A_KD 0.2f
#define YAW_MOTOR_PID_S_KP 3.0f
#define YAW_MOTOR_PID_S_KI 0.0f
#define YAW_MOTOR_PID_S_KD 0.3f
#define YAW_MOTOR_SPEED_MAX_OUT 300
#define YAW_ECD_MAX 8000
#define YAW_ECD_MIN 5600
// endregion
// region pitch DM电机
#define PITCH_MOTOR_PID_A_KP 8.5f
#define PITCH_MOTOR_PID_A_KI 0.0f
#define PITCH_MOTOR_PID_A_KD 1.7f
#define PITCH_POS_MAX  0.24  //8度
#define PITCH_POS_MIN -0.75 //-40.5度
// endregion
// region IMU BMI088

// endregion
// endregion

/*============================== 软件层配置 ==============================*/



/*============================== 应用层配置 ==============================*/
}

#endif //CONFIG_HPP
