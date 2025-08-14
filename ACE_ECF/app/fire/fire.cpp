#include "fire.hpp"

namespace Fire_n
{
/*=====================================   宏   =====================================*/
#define DISABLE_TIME 0.5f
#define ONE_SHOT_TIME 0.3f
#define STUCK_TIME 0.22f
#define RELOADER_GEAR_RATIO 36
#define RELOADER_ENCODER_TO_SHOOT1 36782  // 打一发编码器转动值// 8192*36/8

/*===================================== 实例创建 =====================================*/
    Fire_c* Fire_c::Get_InstancePtr()
    {
        static Fire_c _instance;
        return &_instance;
    }

    Fire_c::Fire_c()
    {
        this->timer_instance = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
        Friction_Init();
        Reloader_Init();
        this->is_loop = true;
    }
// 实例创建end

/*=====================================  初始化  =====================================*/

    void Fire_c::Friction_Init()
    {
        Motor_General_Def_n::Motor_Init_Config_s _config_friction = {
            .controller_param_init_config = {
                .speed_PID = {
                    .Kp = FRICTION_WHEEL_DOWN_PID_KP,
                    .Ki = FRICTION_WHEEL_DOWN_PID_KI,
                    .Kd = FRICTION_WHEEL_DOWN_PID_KD,
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
                .tx_id = FRICTION_WHEEL_DOWN_ID,
              }
        };
        this->friction_motor[0] = new DJI_Motor_n::DJI_Motor_Instance(_config_friction);

        _config_friction.controller_param_init_config.speed_PID.Kp = FRICTION_WHEEL_L_PID_KP;
        _config_friction.controller_param_init_config.speed_PID.Ki = FRICTION_WHEEL_L_PID_KI;
        _config_friction.controller_param_init_config.speed_PID.Kd = FRICTION_WHEEL_L_PID_KD;
        _config_friction.can_init_config.tx_id = FRICTION_WHEEL_L_ID;
        this->friction_motor[1] = new DJI_Motor_n::DJI_Motor_Instance(_config_friction);

        _config_friction.controller_param_init_config.speed_PID.Kp = FRICTION_WHEEL_R_PID_KP;
        _config_friction.controller_param_init_config.speed_PID.Ki = FRICTION_WHEEL_R_PID_KI;
        _config_friction.controller_param_init_config.speed_PID.Kd = FRICTION_WHEEL_R_PID_KD;
        _config_friction.can_init_config.can_handle = &hcan1; // (O,o)! 临时的can1捏
        _config_friction.can_init_config.tx_id = FRICTION_WHEEL_R_ID;
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
                .can_handle = &hcan2,       // 使用can2
                .tx_id = RELOADER_MOTOR_ID,
            },
            .zero_offset = 0,
        }; // 初始位置为0
        this->reloader_motor = new DJI_Motor_n::DJI_Motor_Instance(_config_reloader);
    }
// 初始化end

/*===================================== 功能函数 =====================================*/
    void Fire_c::Friction_Enable(void)
    {
        this->friction_motor[0]->DJIMotorEnable();
        this->friction_motor[1]->DJIMotorEnable();
        this->friction_motor[2]->DJIMotorEnable();
    }

    void Fire_c::Friction_Disable(void)
    {
        this->friction_motor[0]->DJIMotorStop();
        this->friction_motor[1]->DJIMotorStop();
        this->friction_motor[2]->DJIMotorStop();
    }

    void Fire_c::Friction_Stop(void)
    {
        this->friction_motor[0]->DJIMotorSetRef(0);
        this->friction_motor[1]->DJIMotorSetRef(0);
        this->friction_motor[2]->DJIMotorSetRef(0);
    }

    void Fire_c::Friction_UpdateSpeed(void)
    {
        if(this->shoot_speed > 4500)
        {
            this->shoot_speed = 4500;
        }
        else if(this->shoot_speed < 0)
        {
            this->shoot_speed = 0;
        }
        this->friction_motor[0]->DJIMotorSetRef(shoot_speed);
        this->friction_motor[1]->DJIMotorSetRef(shoot_speed);
        this->friction_motor[2]->DJIMotorSetRef(shoot_speed);
    }

    inline void Fire_c::Reloader_Enable(void)
    {
        this->reloader_motor->DJIMotorEnable();
    }

    inline void Fire_c::Reloader_Disable(void)
    {
        this->reloader_motor->DJIMotorStop();
    }

    void Fire_c::Reloader_Stop(void)
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->DJIMotorSetRef(0);
    }

    void Fire_c::Reloader_Clear(void)
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->DJIMotorSetRef(-4500);
    }

    void Fire_c::Reloader_StuckBack(void)
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::OPEN_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::OPEN_LOOP;
        this->reloader_motor->DJIMotorSetRef(-1000);
    }

    bool Fire_c::CheckStuck(void)
    {
        if(abs(this->reloader_motor->MotorMeasure.measure.feedback_real_current) > 7900)
        {
            return true;
        }
        return false;
    }

    void Fire_c::DoShoot(uint8_t num, float freq)
    {
        static float Last_Shoot_Time = 0;
        // 到位检测
        if (this->reloader_motor->MotorMeasure.measure.record_ecd >= RELOADER_ENCODER_TO_SHOOT1 * num)
        {
            this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
            this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
            this->reloader_motor->DJIMotorSetRef(0);
        }
        if (this->timer_instance->ECF_DWT_GetTimeline_s() - Last_Shoot_Time > (1.0f / freq))
        {
            this->reloader_motor->MotorMeasure.measure.record_ecd = 0;
            Last_Shoot_Time = this->timer_instance->ECF_DWT_GetTimeline_s();
            this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::ANGLE_AND_SPEED_LOOP;
            this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::ANGLE_LOOP;
            this->reloader_motor->DJIMotorSetRef(this->reloader_motor->MotorMeasure.measure.feedback_ecd + num * RELOADER_ENCODER_TO_SHOOT1);
        }
    }

    void Fire_c::DoShoot(void)
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->DJIMotorSetRef(this->shoot_freq * 60 * RELOADER_GEAR_RATIO / 8 );
    }
// 功能函数end

/*=====================================  状态机  =====================================*/
    void Fire_c::Change_State(Firc_State_e new_state)
    {
        this->is_loop = false;
        this->StateExit();
        this->current_state = new_state;
        this->StateStart();
        this->is_loop = true;
    }

    void Fire_c::StateStart(void)
    {
        switch (this->current_state)
        {
        case Disable:
            this->timer_delta_t = 0.0f;
            this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
            this->Friction_Stop();
            this->Reloader_Stop();
            break;
        case Enable:
            this->is_wheel_middle = false;
            this->Friction_Stop();
            this->Reloader_Stop();
            break;
        case Ready:
            this->is_wheel_middle = false;
            this->Reloader_Stop();
            break;
        case OneShoot:
            this->timer_delta_t = 0.0f;
            this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
            break;
        case StartShoot:
            break;
        case Stuck:
            this->timer_delta_t = 0.0f;
            this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
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
            if(this->timer_delta_t < DISABLE_TIME)
            {
                this->Friction_Enable();
                this->Reloader_Enable();
                this->timer_delta_t += this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
            }
            else
            {
                this->Friction_Disable();
                this->Reloader_Disable();
            }
            /* check code */
            /*
             * 检测 拨杆 拨中/上拨 是否进入Enable状态
             */
            break;
        case Enable:
            this->Friction_Enable();
            this->Reloader_Enable();
            /* check code */
            /*
             * 检测 拨杆 下拨 是否进入Disable状态
             * 棘轮居中标志位为0 检测 棘轮 居中 居中标志位置1
             * 棘轮居中标志位为1 检测 棘轮 上拨一点 是否进入Ready状态
             * 检测 棘轮 上拨到底 是:拨弹盘反转给速度（退弹） 否:拨弹盘速度给0
             */
            break;
        case Ready:
            this->Friction_Enable();
            this->Reloader_Enable();
            this->Friction_UpdateSpeed();
            /* check code */
            /*
             * 检测 拨杆 下拨 是否进入Disable状态
             * 棘轮居中标志位为0 检测 棘轮 居中 居中标志位置1
             * 棘轮居中标志位为1 检测 棘轮 上拨一点 是否进入Enable状态
             * 检测 棘轮 上拨到底 是:拨弹盘反转给速度（退弹） 否:拨弹盘速度给0
             * 检测 棘轮 下拨 拨杆 居中 是否进入OneShoot状态
             * 检测 棘轮 下拨 拨杆 上拨 是否进入StartShoot状态
             */
            break;
        case OneShoot:
            this->Friction_Enable();
            this->Reloader_Enable();
            this->Friction_UpdateSpeed();
            if(this->CheckStuck())
            {
                this->Change_State(Stuck);
            }
            if(this->timer_delta_t < ONE_SHOT_TIME)
            {
                this->timer_delta_t += this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
                this->DoShoot(1,2);
            }
            else
            {
                /* check code */
                /*
                 * 检测 棘轮 居中以上 进入Ready状态
                 */
            }
            /*
             * 检测 拨杆 下拨 是否进入Disable状态
             */
            break;
        case StartShoot:
            this->Friction_Enable();
            this->Reloader_Enable();
            this->Friction_UpdateSpeed();
            this->DoShoot();
            if(this->CheckStuck())
            {
                this->Change_State(Stuck);
            }
            /* check code */
            /*
             * 检测 拨杆 下拨 是否进入Disable状态
             * 检测 棘轮 居中以上 进入Ready状态
             */
            break;
        case Stuck:
            this->Friction_Enable();
            this->Reloader_Enable();
            this->Friction_UpdateSpeed();
            if(this->timer_delta_t < STUCK_TIME)
            {
                this->timer_delta_t += this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
                this->Reloader_StuckBack();
            }
            else
            {
                this->Change_State(Ready);
            }
            break;
        default:
            break;
        }
    }
// 状态机end

}
