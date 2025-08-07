#ifndef __SHOOT_H
#define __SHOOT_H

#ifdef __cplusplus
extern "C"
{
#endif



#ifdef __cplusplus
}
#include "robot_def.hpp"
#endif
namespace SHOOT_N
{
#define FRICTION_MOTOR_NUMS 2 // 两个摩擦轮
    class shoot_c
    {
    public:
        DJI_Motor_n::DJI_Motor_Instance *friction_motor[FRICTION_MOTOR_NUMS] = {nullptr}; // 摩擦轮电机 0-左 1-右
        DJI_Motor_n::DJI_Motor_Instance *loader_motor = nullptr;
        uint8_t loader_is_blocked=0;
        shoot_mood_t *cmd = nullptr;
        float shoot_speed=0; //发射速度，由拨弹盘决定
        float friction_speed=0; //摩擦轮速度
        shoot_c();
        void friction_init();
        void loader_init();
        void loader_on_pos(uint8_t num, float freq);
        void fire_on();
        void fire_off();
        uint16_t DISABLE_cnt = 0;
        uint32_t now_ecd=0;
        void speed_cheak(float cheak_speed);
        bool check_stuck();
        // 获取唯一实例的静态方法
        static shoot_c *get_instance();

    private:
        void loader_on();
        void friction_on();
    };
    void shoot_task();
    void shoot_init();
    void motor_clear_ecd(DJI_Motor_n::DJIMotorMeasure_t *measure);
}

#endif /*__SHOOT_H*/