#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "dji_motor.hpp"

namespace Config_n
{
/*============================== 代码层配置 ==============================*/
#define IS_DEBUG_MODE 1u // 是否是debug模式，0为否，1为是，开启debug模式后，常规任务将被关闭，开启debug任务，一些debug变量也被赋予内存，方便ozone查看


/*============================== 设备层配置 ==============================*/
/* 1.发射机构 */
// 1.1 摩擦轮
#define FRICTION_MOTOR_NUMS 3

// 1.1.1 下摩擦轮
#define FRICTION_WHEEL_DOWN_PID_KP 10.0f
#define FRICTION_WHEEL_DOWN_PID_KI 0.1f
#define FRICTION_WHEEL_DOWN_PID_KD 0.0f
#define FRICTION_WHEEL_DOWN_ID     1u

// 1.1.2 左摩擦轮
#define FRICTION_WHEEL_L_PID_KP    10.0f
#define FRICTION_WHEEL_L_PID_KI    0.1f
#define FRICTION_WHEEL_L_PID_KD    0.0f
#define FRICTION_WHEEL_L_ID        2u

// 1.1.3 右摩擦轮
#define FRICTION_WHEEL_R_PID_KP    10.0f
#define FRICTION_WHEEL_R_PID_KI    0.1f
#define FRICTION_WHEEL_R_PID_KD    0.0f
#define FRICTION_WHEEL_R_ID        3u

// 1.2 拨弹盘
#define RELOADER_MOTOR_PID_A_KP    0.15f
#define RELOADER_MOTOR_PID_A_KI    0.0f
#define RELOADER_MOTOR_PID_A_KD    0.67f
#define RELOADER_MOTOR_PID_S_KP    10.0f
#define RELOADER_MOTOR_PID_S_KI    0.0f
#define RELOADER_MOTOR_PID_S_KD    2.0f
#define RELOADER_MOTOR_ID          4u

#define RELOADER_ENCODER_TO_SHOOT1 36782  // 打一发编码器转动值// 8192*36/8 //-294257 //转 一圈


/*============================== 软件层配置 ==============================*/



/*============================== 应用层配置 ==============================*/
}

#endif //CONFIG_HPP
