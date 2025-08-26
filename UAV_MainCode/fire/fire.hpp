#ifndef __FIRE_HPP
#define __FIRE_HPP

#include "bsp_dwt.hpp"
#include "dji_motor.hpp"
#include "robot_cmd.hpp"
#include "Config.hpp"

namespace Fire_n
{
    typedef enum
    {
        Disable    = 0x00, // 失能
        Enable     = 0x01, // 使能
        Ready      = 0x02, // 摩擦轮转动，允许播弹盘
        OneShoot   = 0x03, // 单发逻辑
        StartShoot = 0x04, // 连发逻辑
        Stuck      = 0x05, // 卡弹
    } Firc_State_e;

    class Fire_c
    {
    public:
        /* motor instance ptr */
        DJI_Motor_n::DJI_Motor_Instance* friction_motor[FRICTION_MOTOR_NUMS] = {nullptr};
        DJI_Motor_n::DJI_Motor_Instance* reloader_motor = nullptr;

        /* date */
        float shoot_speed = 4500.0f;                // 摩擦轮转速，单位 转/min
        float shoot_freq  = 4.0f;                   // 弹频，单位Hz

        /* function */
        static Fire_c *Get_InstancePtr();
        void ChangeState(Firc_State_e new_state);

        /* Loop */
        friend void StateLoop(void);                       // 状态主循环，1ms执行一次

    private:
        /* state */
        Firc_State_e current_state = Disable;

        /* flag */
        bool is_loop               = false;
        bool is_triggers_locked    = false; // 触发锁定标志位，用于防止某些状态循环切换(Enable、Ready)

        /* timer */
        float timer_delta_t                  = 0.0f;
        uint32_t timer_cnt                   = 0;
        BSP_DWT_n::BSP_DWT_c *timer_instance = nullptr;

        /* command */
        RobotCMD_n::RobotCMD_c* cmd_instance;

        /* class */
        Fire_c();

        /* function */
        void Friction_Init(void);
        void Friction_Enable(void);
        void Friction_Disable(void);
        void Friction_Stop(void);
        void Friction_UpdateSpeed(void);
        void Friction_AddSpeed(float val);

        void Reloader_Init(void);
        void Reloader_Enable(void);
        void Reloader_Disable(void);
        void Reloader_Stop(void);
        void Reloader_Clear(void);
        void Reloader_StuckBack(void);
        void Reloader_AddFreq(float val);
        void DoShoot(uint8_t num, float freq);
        void DoShoot(void);
        bool Check_Stuck(void);

        void Change_ShootVal_DT16(void);
        void Change_ShootVal_TC(void);

        void StateStart(void);
        void StateExit(void);
    };
    void StateLoop(void);
}



#endif //! __FIRE_HPP
