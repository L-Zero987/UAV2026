#ifndef __ROBOT_DEF_HPP
#define __ROBOT_DEF_HPP

#ifdef __cplusplus
extern "C"
{
#endif 
#include "stdlib.h"
// #include "rm_cilent_ui.h"
#ifdef __cplusplus
}
#include "bsp_dwt.hpp"
#include "vision.hpp"
#include "dji_motor.hpp"
#include "dm_mit_mode.hpp"
#include "dm_ls_mode.hpp"
#include "bsp_referee.hpp"


#include "user_maths.hpp"
#include "lqr_alg.hpp"
#include "filter.hpp"

#define LIMIT_SPEED 24.5f // 不喜欢25弹速

//重力补偿力矩 = 云台pitch连杆力臂---mgl* cos(thita)
#define LEVER_ARM_OF_PITCH 0.0885//0.37131808 // mgl = 88.5mm *0.595kg*9.8
#define LEVER_ARM_TO_PITCH_COF  4.77//4.7度
#define MG 

// 操作灵敏度
#define PITCH_SENSOR_RC 0.00042f// 0.00022f //发送给大妙为
#define YAW_SENSOR_RC 0.0004f // 0.0006
#define YAW_SENSOR_TC 0.001
#define PITCH_SENSOR_TC 0.001
#define YAW_USE_LQR 1
    //#define YAW_USE_PID 1
//YAW电机
#define YAW_ECD_MAX 8000
#define YAW_ECD_MIN 5600
//pitch 零点为-0.0083
#define PITCH_POS_MAX  0.24  //8度
#define PITCH_POS_MIN -0.75 //-40.5度
//shoot  
#define MOTORRPM_TO_FIRESPEED  0.00416//0.00361f// m/s = (rpm*2pi /60 )* r  r=60mm
// 一圈的编码值 * rpm /60= 每秒转过的编码值 e
//  编码值 e/36782 == 弹频
#define MOTORRPM_TO_FIRERATE 0.133f    // HZ = rpm/60 *8
// 发/s——>rpm
#define FIRERATE_TO_MOTORRPM 266
typedef enum {
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_ENABLE, 
} gimbal_mode_e;
typedef enum
{
    AIM_MANAUL_MODE = 0, // 手瞄
    AUTO_BUT_NOT_AIM,    // 开自瞄但没瞄上
    AUTO_AIM_MODE,    // 自瞄

} vision_aim_e;

/* 机器人运动状态 */
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} robot_state_e;

// 发射模式设置:用于cmd
typedef enum
{
    SHOOT_OFF = 0, // 发射机构急停
    SHOOT_ON,
    
} shoot_mode_e;

typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LOADER_OFF = 0,
    LOADER_ON,
    LOADER_ON_REVERSE,
} loader_mode_e;
struct gimbal_control_t
{
    vision_aim_e vision_is_on = AIM_MANAUL_MODE;
    gimbal_mode_e mode = GIMBAL_ZERO_FORCE;
    float yaw_imu_target = 0; //陀螺仪闭环的目标值增量
    float pitch_imu_target= -35; //陀螺仪闭环的目标值增量,初始为-35
    bool use_ecd = false; //是否使用编码闭环
    float yaw_ecd_target = 0;
    float pitch_ecd_target = 0;
};
struct shoot_mood_t
{
    shoot_mode_e shoot_mode = SHOOT_OFF;
    friction_mode_e friction_mode = FRICTION_OFF;
    loader_mode_e loader_mode = LOADER_OFF;
    uint8_t shoot_num = 0; //发射数量 0为速度环，1为单发，8为8发
    uint8_t fire_cmd = 0;//发弹更新指令
    uint8_t fire_hz = 0; //发射频率默认0为25hz，最高1为36
    // 需不需要加连发模式--》火力控制
    float firction_compansate = 0;
};

#endif
#endif /*__ROBOT_DEF_HPP*/
