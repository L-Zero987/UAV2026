#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace Config_n
{
/*============================== 代码层配置 ==============================*/
#define IS_DEBUG_MODE 1u // 是否是debug模式，0为否，1为是，开启debug模式后，常规任务将被关闭，开启debug任务，一些debug变量也被赋予内存，方便ozone查看


/*============================== 设备层配置 ==============================*/
/* region 发射机构 */
// 摩擦轮
#define FRICTION_MOTOR_NUMS     3       // 摩擦轮电机数量
#define FRICTION_WHEEL_RADIUS   0.06f   // 轮半径，单位m

// 下摩擦轮
#define FRICTION_WHEEL_DOWN_PID_KP 6.0f
#define FRICTION_WHEEL_DOWN_PID_KI 0.0f
#define FRICTION_WHEEL_DOWN_PID_KD 5.0f
#define FRICTION_WHEEL_DOWN_ID     0x01u

// 左摩擦轮
#define FRICTION_WHEEL_L_PID_KP    8.0f
#define FRICTION_WHEEL_L_PID_KI    0.0f
#define FRICTION_WHEEL_L_PID_KD    5.0f
#define FRICTION_WHEEL_L_ID        0x02u

// 右摩擦轮
#define FRICTION_WHEEL_R_PID_KP    10.0f
#define FRICTION_WHEEL_R_PID_KI    0.0f
#define FRICTION_WHEEL_R_PID_KD    5.0f
#define FRICTION_WHEEL_R_ID        0x03u

// 拨弹盘
#define RELOADER_MOTOR_PID_A_KP    0.15f
#define RELOADER_MOTOR_PID_A_KI    0.0f
#define RELOADER_MOTOR_PID_A_KD    0.67f
#define RELOADER_MOTOR_PID_S_KP    10.0f
#define RELOADER_MOTOR_PID_S_KI    0.0f
#define RELOADER_MOTOR_PID_S_KD    2.0f
#define RELOADER_MOTOR_ID          0x04u
// endregion

/* region 云台 */
// yaw GM6020
#define YAW_MOTOR_PID_A_KP 6.0f
#define YAW_MOTOR_PID_A_KI 0.0f
#define YAW_MOTOR_PID_A_KD 0.2f
#define YAW_MOTOR_PID_S_KP 3.0f
#define YAW_MOTOR_PID_S_KI 0.0f
#define YAW_MOTOR_PID_S_KD 0.3f
#define YAW_MOTOR_SPEED_MAX_OUT 300
#define YAW_ECD_MAX 8000
#define YAW_ECD_MIN 5600

// pitch DM电机
#define PITCH_MOTOR_PID_A_KP 8.5f
#define PITCH_MOTOR_PID_A_KI 0.0f
#define PITCH_MOTOR_PID_A_KD 1.7f
#define PITCH_POS_MAX  0.24  //8度
#define PITCH_POS_MIN -0.75 //-40.5度

// IMU BMI088


// endregion

/*============================== 应用层配置 ==============================*/
/* region 云台灵敏度 */
#define YAW_SENSOR_RC 0.0004f // 0.0006
#define PITCH_SENSOR_RC 0.00042f// 0.00022f //发送给大妙为
#define YAW_SENSOR_TC   0.001f
#define PITCH_SENSOR_TC 0.001f
// endregion

/* region 弹速与弹频 */
#define MAX_SHOOT_FREQ 8.0f
#define MIN_SHOOT_FREQ 1.0f
#define MAX_SHOOT_SPEED 6000.0f
#define MIN_SHOOT_SPEED 3000.0f
#define SPEED_SENSOR_RC 1.0f
#define FREQ_SENSOR_RC 0.001f
#define SPEED_SENSOR_TC 1.0f
#define FREQ_SENSOR_TC 0.001f
}

#endif //CONFIG_HPP
