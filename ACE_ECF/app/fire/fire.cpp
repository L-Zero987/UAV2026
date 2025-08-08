#include "fire.hpp"

namespace Fire_n
{
    BSP_DWT_n::BSP_DWT_c *shoot_delay = nullptr;

/*===================================== 实例创建 =====================================*/
    Fire_c* Fire_c::Get_InstancePtr()
    {
        static Fire_c _instance;
        return &_instance;
    }

    Fire_c::Fire_c()
    {
        shoot_delay = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
        Friction_Init();
        // Reloader_Init();
        this->is_loop = true;
    }
// 实例创建end

/*=====================================  初始化  =====================================*/

    void Fire_c::Friction_Init()
    {
        Motor_General_Def_n::Motor_Init_Config_s _config_friction = {
            .controller_param_init_config = {
                .speed_PID = {
                    .Kp = FRICTION_WHEEL_L_PID_KP,
                    .Ki = FRICTION_WHEEL_L_PID_KI,
                    .Kd = FRICTION_WHEEL_L_PID_KD,
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
                .tx_id = FRICTION_WHEEL_L_ID,
              }
        };
        this->friction_motor[0] = new DJI_Motor_n::DJI_Motor_Instance(_config_friction);

        _config_friction.controller_param_init_config.speed_PID.Kp = FRICTION_WHEEL_R_PID_KP;
        _config_friction.controller_param_init_config.speed_PID.Ki = FRICTION_WHEEL_R_PID_KI;
        _config_friction.controller_param_init_config.speed_PID.Kd = FRICTION_WHEEL_R_PID_KD;
        _config_friction.can_init_config.tx_id = FRICTION_WHEEL_R_ID;
        this->friction_motor[1] = new DJI_Motor_n::DJI_Motor_Instance(_config_friction);

        _config_friction.controller_param_init_config.speed_PID.Kp = FRICTION_WHEEL_DOWN_PID_KP;
        _config_friction.controller_param_init_config.speed_PID.Ki = FRICTION_WHEEL_DOWN_PID_KI;
        _config_friction.controller_param_init_config.speed_PID.Kd = FRICTION_WHEEL_DOWN_PID_KD;
        _config_friction.can_init_config.tx_id = FRICTION_WHEEL_DOWN_ID;
        this->friction_motor[2] = new DJI_Motor_n::DJI_Motor_Instance(_config_friction);
    }

    void Fire_c::Reloader_Init()
    {
        Motor_General_Def_n::Motor_Init_Config_s _config_reloader = {
            .controller_param_init_config = {
                .speed_PID = {
                    .Kp = RELOADER_MOTOR_PID_S_KP,
                    .Ki = RELOADER_MOTOR_PID_S_KI,
                    .Kd = RELOADER_MOTOR_PID_S_KD,
                    .ActualValueSource = nullptr,
                    .mode = Output_Limit | Integral_Limit,
                    .max_out = 9000,
                    .max_Ierror = 3000,
                    .deadband = 0.3,
                    // .stepIn = 3000,
                },
                .angle_PID = {
                    .Kp = RELOADER_MOTOR_PID_A_KP,
                    .Ki = RELOADER_MOTOR_PID_A_KI,
                    .Kd = RELOADER_MOTOR_PID_A_KD,
                    .ActualValueSource = nullptr,
                    .mode = Integral_Limit,
                    .max_out = 9000,
                    .max_Ierror = 3000,
                    .deadband = 0.3,
                    // .stepIn = 1000,
                },
            },
            .controller_setting_init_config = {
                .outer_loop_type = Motor_General_Def_n::SPEED_LOOP, // 使用速度环
                .close_loop_type = Motor_General_Def_n::SPEED_LOOP,
                .motor_reverse_flag = Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
                .feedback_reverse_flag = Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
                .angle_feedback_source = Motor_General_Def_n::MOTOR_FEED,
                .speed_feedback_source = Motor_General_Def_n::MOTOR_FEED, // 电机反馈
            },
            .motor_type = Motor_General_Def_n::M2006, // 2006电机
            .can_init_config = {
                .can_handle = &hcan1,       // 使用can2 //(O,o)! 暂时修改
                .tx_id = RELOADER_MOTOR_ID,
            },
            .zero_offset = 0,
        }; // 初始位置为0
        this->reloader_motor = new DJI_Motor_n::DJI_Motor_Instance(_config_reloader);
    }
// 初始化end

/*===================================== 功能函数 =====================================*/
    void Fire_c::Set_FrictionStop()
    {
        this->friction_motor[0]->DJIMotorSetRef(0);
        this->friction_motor[1]->DJIMotorSetRef(0);
        this->friction_motor[2]->DJIMotorSetRef(0);
    }

    void Fire_c::Set_FrictionSpeed(void)
    {
        this->friction_motor[0]->DJIMotorSetRef(this->shoot_speed * 80); // (O,o)! 临时的80，要修改宏定义
        this->friction_motor[1]->DJIMotorSetRef(this->shoot_speed * 80);
        this->friction_motor[2]->DJIMotorSetRef(this->shoot_speed * 80);
    }

    void Fire_c::Shoot(uint8_t num, float freq)
    {
        static float Last_Shoot_Time = 0;
        // 到位检测
        if (this->reloader_motor->MotorMeasure.measure.record_ecd >= RELOADER_ENCODER_TO_SHOOT1 * num)
        {
            this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
            this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
            this->reloader_motor->DJIMotorSetRef(0);
        }
        if (shoot_delay->ECF_DWT_GetTimeline_s() - Last_Shoot_Time > (1.0f / freq))
        {
            this->reloader_motor->MotorMeasure.measure.record_ecd = 0;
            Last_Shoot_Time = shoot_delay->ECF_DWT_GetTimeline_s();
            this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::ANGLE_AND_SPEED_LOOP;
            this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::ANGLE_LOOP;
            this->is_stuck = false;
            this->reloader_motor->DJIMotorSetRef(this->reloader_motor->MotorMeasure.measure.feedback_ecd + num * RELOADER_ENCODER_TO_SHOOT1);
        }
    }

    void Fire_c::Shoot(float freq)
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->DJIMotorSetRef(4500);// (O,o)! 暂时的值，要修改
    }


// 功能函数end

/*=====================================  状态机  =====================================*/
    void Fire_c::Change_State(Firc_State_e new_state)
    {
        this->StateExit();
        this->last_state    = this->current_state;
        this->current_state = new_state;
        this->StateStart();
    }

    void Fire_c::StateStart(void)
    {
        switch (this->current_state)
        {
        case Disable:
            this->friction_motor[0]->DJIMotorStop();
            this->friction_motor[1]->DJIMotorStop();
            this->friction_motor[2]->DJIMotorStop();
            this->reloader_motor->DJIMotorStop();
            break;
        case Enable:
            this->friction_motor[0]->DJIMotorEnable();
            this->friction_motor[1]->DJIMotorEnable();
            this->friction_motor[2]->DJIMotorEnable();
            this->reloader_motor->DJIMotorEnable();
            break;
        case Ready:
            break;
        case OneShoot:
            break;
        case StartShoot:
            break;
        case Stuck:
            break;
        default:
            break;
        }
    }

    void Fire_c::StateExit(void)
    {
        switch (this->current_state)
        {
        case Disable:
            break;
        case Enable:
            break;
        case Ready:
            break;
        case OneShoot:
            break;
        case StartShoot:
            break;
        case Stuck:
            break;
        default:
            break;
        }
    }

    void Fire_c::StateLoop(void)
    {
        if (!this->is_loop)return;
        switch (this->current_state)
        {
        case Disable:
            break;
        case Enable:
            this->Set_FrictionStop();
            break;
        case Ready:
            this->Set_FrictionSpeed();
            break;
        case OneShoot:
            this->Shoot(1, this->shoot_freq);
            break;
        case StartShoot:
            this->Shoot(this->shoot_freq);
            break;
        case Stuck:
            break;
        default:
            break;
        }
    }
// 状态机end

}
