#include "debug_task.hpp"

namespace Debug_n
{
    Debug_c::Debug_c(void)
    {
        // 初始化开始，关中断
        __disable_irq();
        // 只有速度环
        Motor_General_Def_n::Motor_Init_Config_s config = {
            .controller_param_init_config = {
              .speed_PID = {
                .Kp = 2.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .mode = Output_Limit,
                .max_out = 9000,
                .deadband = 0,
              },
            },
            .controller_setting_init_config = {
              .outer_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
              .close_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
              .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
              .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
              // 反馈来源可设置为 MOTOR_FEED OTHER_FEED SPEED_APS ANGULAR_SPEED LINEAR_SPEED
              .speed_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
            },
            .motor_type = Motor_General_Def_n::M3508,
            .can_init_config = {
              .can_handle = &hcan2,
              .tx_id = 3,// 看电调闪几下就填几
            }
        };
        this->test_motor = new DJI_Motor_n::DJI_Motor_Instance(config);
        __enable_irq();

        this->test_motor->DJIMotorEnable();
        this->test_motor->DJIMotorSetRef(1000.0);
    }

    void Debug_c::Loop1(void)
    {
        this->test_motor->DJIMororStopExpThis();
        DJI_Motor_n::DJIMotorControl();
    }
}
