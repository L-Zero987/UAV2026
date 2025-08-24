/******************************** ********************************
 * @file fire.cpp
 * @author L_Zero
 * @version 0.1
 * @date 2025-8-17
 * @brief 发射机构驱动代码
 *
 *
 */
#include "fire.hpp"

namespace Fire_n
{
/* region 宏以及变量声明 */
#define DISABLE_TIME               0.5f   // 失能处理时间
#define ONE_SHOT_TIME              0.3f   // 单发时间
#define STUCK_TIME                 0.22f  // 卡弹回退时间
#define RELOADER_GEAR_RATIO        36     // 减速比
#define RELOADER_ENCODER_TO_SHOOT1 36782  // 打一发编码器转动值// 8192*36/8

Fire_c* this_ptr = nullptr;
// endregion

/* region 实例创建 */
    Fire_c* Fire_c::Get_InstancePtr()
    {
        static Fire_c _instance;
        this_ptr = &_instance;
        return &_instance;
    }

    Fire_c::Fire_c()
    {
        this->is_loop = false;
        this->timer_instance = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
        this->cmd_instance   = RobotCMD_n::RobotCMD_c::Get_InstancePtr();
        this->Friction_Init();
        this->Reloader_Init();
        this->is_loop = true;
    }
// endregion

/* region 初始化 */
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
// endregion

/* region 功能函数 */
    inline void Fire_c::Friction_Enable()
    {
        if(!this->friction_motor[0]->is_lost_dji && this->friction_motor[0]->MotorMeasure.measure.init_flag)
        {
            this->friction_motor[0]->DJIMotorEnable();
        }
        if(!this->friction_motor[1]->is_lost_dji && this->friction_motor[1]->MotorMeasure.measure.init_flag)
        {
            this->friction_motor[1]->DJIMotorEnable();
        }
        if(!this->friction_motor[2]->is_lost_dji && this->friction_motor[2]->MotorMeasure.measure.init_flag)
        {
            this->friction_motor[2]->DJIMotorEnable();
        }
    }

    inline void Fire_c::Friction_Disable()
    {
        this->friction_motor[0]->DJIMotorStop();
        this->friction_motor[1]->DJIMotorStop();
        this->friction_motor[2]->DJIMotorStop();
    }

    inline void Fire_c::Friction_Stop()
    {
        this->friction_motor[0]->DJIMotorSetRef(0);
        this->friction_motor[1]->DJIMotorSetRef(0);
        this->friction_motor[2]->DJIMotorSetRef(0);
    }

    void Fire_c::Friction_UpdateSpeed()
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

    inline void Fire_c::Reloader_Enable()
    {
        if(!this->reloader_motor->is_lost_dji && this->reloader_motor->MotorMeasure.measure.init_flag)
        {
            this->reloader_motor->DJIMotorEnable();
        }
    }

    inline void Fire_c::Reloader_Disable()
    {
        this->reloader_motor->DJIMotorStop();
    }

    inline void Fire_c::Reloader_Stop()
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->DJIMotorSetRef(0);
    }

    inline void Fire_c::Reloader_Clear()
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->DJIMotorSetRef(-4500);
    }

    inline void Fire_c::Reloader_StuckBack()
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::OPEN_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::OPEN_LOOP;
        this->reloader_motor->DJIMotorSetRef(-1000);
    }

    bool Fire_c::Check_Stuck()
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

    inline void Fire_c::DoShoot()
    {
        this->reloader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        this->reloader_motor->DJIMotorSetRef(this->shoot_freq * 60 * RELOADER_GEAR_RATIO / 8 );
    }
// endregion

/* region 状态机 */
    void Fire_c::ChangeState(Firc_State_e new_state)
    {
        this->is_loop = false;
        this->StateExit();
        this->current_state = new_state;
        this->StateStart();
        this->is_loop = true;
    }

    void Fire_c::StateStart()
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

    void Fire_c::StateExit()
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

    void StateLoop()
    {
        if (this_ptr == nullptr)return;
        if (!this_ptr->is_loop)return;
        if (this_ptr->cmd_instance->connect_state == RobotCMD_n::DR16_CMD)
        {
            // region DT7下的状态机
            switch (this_ptr->current_state)
            {
                case Disable: // region Disable
                    /*
                     * 计时器未到指定时间，摩擦轮、拨弹盘(速控)使能
                     * 计时器未到指定时间，计时器计时
                     * 计时器到指定时间，摩擦轮、拨弹盘失能
                     * 检测 拨杆 拨中/上拨 是否进入Enable状态
                     */
                    if(this_ptr->timer_delta_t < DISABLE_TIME)
                    {
                        this_ptr->Friction_Enable();
                        this_ptr->Reloader_Enable();
                        this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                    }
                    else
                    {
                        this_ptr->Friction_Disable();
                        this_ptr->Reloader_Disable();
                    }

                    if(this_ptr->cmd_instance->Get_RC_SW2State() >= RobotCMD_n::CMD_MIDDLE)
                    {
                        this_ptr->ChangeState(Enable);
                        return;
                    }
                    // endregion
                    break;
                case Enable: // region Enable
                    /*
                     * 摩擦轮、拨弹盘使能
                     * 检测 拨杆 下拨 是否进入Disable状态
                     * 棘轮居中标志位为0 检测 棘轮 居中 居中标志位置1
                     * 棘轮居中标志位为1 检测 棘轮 上拨一点 是否进入Ready状态
                     * 检测 棘轮 上拨到底 是:拨弹盘反转给速度（退弹） 否:拨弹盘速度给0
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_Stop();
                    this_ptr->Reloader_Stop();
                    if(this_ptr->cmd_instance->Get_RC_SW2State() <= RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }
                    if(this_ptr->cmd_instance->Get_RC_RatchetState() == RobotCMD_n::CMD_HIGH)
                    {
                        this_ptr->Reloader_Clear();
                    }
                    if(this_ptr->is_wheel_middle)
                    {
                        if(this_ptr->cmd_instance->Get_RC_RatchetState() <= RobotCMD_n::CMD_MIDDLE_HIGH)
                        {
                            this_ptr->ChangeState(Ready);
                            return;
                        }
                    }
                    else
                    {
                        if(this_ptr->cmd_instance->Get_RC_RatchetState() <= RobotCMD_n::CMD_MIDDLE)
                        {
                            this_ptr->is_wheel_middle = true;
                        }
                    }
                    // endregion
                    break;
                case Ready: // region Ready
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 检测 拨杆 下拨 是否进入Disable状态
                     * 棘轮居中标志位为0 检测 棘轮 居中 居中标志位置1
                     * 棘轮居中标志位为1 检测 棘轮 上拨一点 是否进入Enable状态
                     * 检测 棘轮 上拨到底 是:拨弹盘反转给速度（退弹） 否:拨弹盘速度给0
                     * 检测 棘轮 下拨 拨杆 居中 是否进入OneShoot状态
                     * 检测 棘轮 下拨 拨杆 上拨 是否进入StartShoot状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    this_ptr->Reloader_Stop();
                    if(this_ptr->cmd_instance->Get_RC_SW2State() <= RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }
                    if(this_ptr->cmd_instance->Get_RC_RatchetState() == RobotCMD_n::CMD_HIGH)
                    {
                        this_ptr->Reloader_Clear();
                    }
                    if(this_ptr->is_wheel_middle)
                    {
                        if(this_ptr->cmd_instance->Get_RC_RatchetState() <= RobotCMD_n::CMD_MIDDLE_HIGH)
                        {
                            this_ptr->ChangeState(Enable);
                            return;
                        }
                    }
                    else
                    {
                        if(this_ptr->cmd_instance->Get_RC_RatchetState() <= RobotCMD_n::CMD_MIDDLE)
                        {
                            this_ptr->is_wheel_middle = true;
                        }
                    }
                    if(this_ptr->cmd_instance->Get_RC_RatchetState() == RobotCMD_n::CMD_LOW)
                    {
                        if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_MIDDLE)
                        {
                            this_ptr->ChangeState(OneShoot);
                            return;
                        }
                        if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
                        {
                            this_ptr->ChangeState(StartShoot);
                            return;
                        }
                    }
                    // endregion
                    break;
                case OneShoot: // region OneShoot
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 检测 调用卡弹检测函数 是否进入Stuck状态
                     * 检测 拨杆 下拨 是否进入Disable状态
                     * 计时器没到单发时长，计时器计时
                     * 计时器没到单发时长，调用发射函数
                     * 计时器到单发时长，检测 棘轮 居中以上 进入Ready状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    if(this_ptr->cmd_instance->Get_RC_SW2State() <= RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }

                    if(this_ptr->Check_Stuck())
                    {
                        this_ptr->ChangeState(Stuck);
                        return;
                    }

                    if(this_ptr->timer_delta_t < ONE_SHOT_TIME)
                    {
                        this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                        this_ptr->DoShoot(1,2);
                    }
                    else
                    {
                        if(this_ptr->cmd_instance->Get_RC_SW2State() >= RobotCMD_n::CMD_MIDDLE)
                        {
                            this_ptr->ChangeState(Ready);
                            return;
                        }
                    }
                    // endregion
                    break;
                case StartShoot: // region StartShoot
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 调用发射函数
                     * 检测 调用卡弹检测函数 是否进入Stuck状态
                     * 检测 拨杆 下拨 是否进入Disable状态
                     * 检测 棘轮 居中以上 进入Ready状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    this_ptr->DoShoot();
                    if(this_ptr->cmd_instance->Get_RC_SW2State() <= RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }

                    if(this_ptr->Check_Stuck())
                    {
                        this_ptr->ChangeState(Stuck);
                        return;
                    }

                    if(this_ptr->cmd_instance->Get_RC_SW2State() >= RobotCMD_n::CMD_MIDDLE)
                    {
                        this_ptr->ChangeState(Ready);
                        return;
                    }
                    // endregion
                    break;
                case Stuck: // region Stuck
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 计时器没到回退时长，计时器计时
                     * 计时器没到回退时长，拨弹盘反转
                     * 计时器到回退时长，进入Ready状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    if(this_ptr->timer_delta_t < STUCK_TIME)
                    {
                        this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                        this_ptr->Reloader_StuckBack();
                    }
                    else
                    {
                        this_ptr->ChangeState(Ready);
                        return;
                    }
                    // endregion
                    break;
                default:
                    break;
            }
            // endregion
        }
        else if(this_ptr->cmd_instance->Get_ConnectState() == RobotCMD_n::TC_CMD)
        {
            // region TC下的状态机
            switch (this_ptr->current_state)
            {
                case Disable: // region Disable
                    // 检测到图传链接，直接进入Enable状态
                    this_ptr->ChangeState(Enable);
                    return;
                    // endregion
                    break;
                case Enable: // region Enable
                    /*
                     * 摩擦轮、拨弹盘使能
                     * 检测 R键 由按下变为抬起 进入Ready状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_Stop();
                    this_ptr->Reloader_Stop();
                    if(this_ptr->cmd_instance->Check_TC_KeyUp('R',this_ptr->cmd_instance->TC_cmd->kb.bit.R))
                    {
                        this_ptr->ChangeState(Ready);
                        return;
                    }
                    // endregion
                    break;
                case Ready: // region Ready
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 检测 R键 按下 进入Enable状态
                     * 检测 左键 抬起 且弹频率<=1 进入OneShoot状态
                     * 检测 左键 按住 且弹频率 >1 进入StartShoot状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    this_ptr->Reloader_Stop();
                    if(this_ptr->cmd_instance->Check_TC_KeyDown('R',this_ptr->cmd_instance->TC_cmd->kb.bit.R))
                    {
                        this_ptr->ChangeState(Enable);
                        return;
                    }
                    if(this_ptr->cmd_instance->Check_TC_KeyUp('l',this_ptr->cmd_instance->TC_cmd->mouse.press_l) &&
                        this_ptr->shoot_freq <= 1)
                    {
                        this_ptr->ChangeState(OneShoot);
                        return;
                    }
                    if(this_ptr->cmd_instance->Check_TC_KeyPress('l',this_ptr->cmd_instance->TC_cmd->mouse.press_l) &&
                        this_ptr->shoot_freq > 1)
                    {
                        this_ptr->ChangeState(StartShoot);
                        return;
                    }
                    // endregion
                    break;
                case OneShoot: // region OneShoot
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 检测 调用卡弹检测函数 是否进入Stuck状态
                     * 计时器没到单发时长，计时器计时
                     * 计时器没到单发时长，调用发射函数
                     * 计时器到单发时长，回到Ready状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    if(this_ptr->cmd_instance->Get_RC_SW2State() <= RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }
                    if(this_ptr->Check_Stuck())
                    {
                        this_ptr->ChangeState(Stuck);
                        return;
                    }
                    if(this_ptr->timer_delta_t < ONE_SHOT_TIME)
                    {
                        this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                        this_ptr->DoShoot(1,2);
                    }
                    else
                    {
                        this_ptr->ChangeState(Ready);
                        return;
                    }
                    // endregion
                    break;
                case StartShoot: // region StartShoot
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 调用发射函数
                     * 检测 调用卡弹检测函数 是否进入Stuck状态
                     * 检测 左键 抬起 回到Ready状态
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    this_ptr->DoShoot();
                    if(this_ptr->Check_Stuck())
                    {
                        this_ptr->ChangeState(Stuck);
                        return;
                    }
                    if(this_ptr->cmd_instance->Check_TC_KeyUp('l',this_ptr->cmd_instance->TC_cmd->mouse.press_l))
                    {
                        this_ptr->ChangeState(Ready);
                        return;
                    }
                    // endregion
                    break;
                case Stuck: // region Stuck
                    /*
                     * 摩擦轮给指定速度
                     * 摩擦轮、拨弹盘使能
                     * 计时器没到回退时长，计时器计时
                     * 计时器没到回退时长，拨弹盘反转
                     * 计时器到回退时长，进入Ready状态 (O,o)! 理论上会出现单发卡弹直接不发的情况，可能需要last_state解决
                     */
                    this_ptr->Friction_Enable();
                    this_ptr->Reloader_Enable();
                    this_ptr->Friction_UpdateSpeed();
                    if(this_ptr->timer_delta_t < STUCK_TIME)
                    {
                        this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                        this_ptr->Reloader_StuckBack();
                    }
                    else
                    {
                        this_ptr->ChangeState(Ready);
                        return;
                    }
                    // endregion
                    break;
                default:
                    break;
            }
            // endregion
        }
    }

// endregion
}
