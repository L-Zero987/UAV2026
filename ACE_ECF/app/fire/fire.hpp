#ifndef __FIRE_HPP
#define __FIRE_HPP

#include "dji_motor.hpp"
#include "bsp_dwt.hpp"
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
        DJI_Motor_n::DJI_Motor_Instance* friction_motor[FRICTION_MOTOR_NUMS] = {nullptr};
        DJI_Motor_n::DJI_Motor_Instance* reloader_motor = nullptr;

        static Fire_c *Get_InstancePtr();

        void StateLoop(void);               // 状态主循环，1ms执行一次
        void Change_State(Firc_State_e new_state);
        float shoot_speed = 10;             // 弹速，单位m/s
        float shoot_freq  = 50;             // 弹频，单位Hz

    private:
        Firc_State_e current_state = Disable;
        Firc_State_e last_state    = Disable;
        bool is_loop = false;

        Fire_c();

        void Friction_Init(void);
        void Reloader_Init(void);
        void Friction_Enable(void);
        void Frictuin_Disable(void);
        void Friction_Stop(void);
        void Friction_SetSpeed(void);
        void Reloader_Enable(void);
        void Reloader_Disable(void);
        void Reloader_Stop(void);

        void DoShoot(uint8_t num, float freq);

        void StateStart(void);
        void StateExit(void);
    };
}



#endif //! __FIRE_HPP
