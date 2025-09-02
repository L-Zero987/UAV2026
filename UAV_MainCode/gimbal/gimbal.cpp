#include "gimbal.hpp"
#include "Config.hpp"

extern "C"
{
#include "main.h"
#include "spi.h"
}

namespace Gimbal_n
{
    /* region 神秘DM框架适配 */
    void pitch_rx_call_back(BSP_CAN_Part_n::CANInstance_c *registerinstance)
    {
        Gimbal_c *ins = Gimbal_c::Get_InstancePtr();
        int p_int, v_int, t_int;
        uint8_t rx_buf[8] = {0};
        memcpy(rx_buf, registerinstance->rx_buff, 8); // 存储数据，防止变化
//        ins->pitch_motor->get_data_.id = (rx_buf[0]) & 0x0F;
        ins->pitch_motor->get_data_.nowState = (DM_Motor_n::DM_NowState_e)(rx_buf[0] >> 4);
        p_int = (rx_buf[1] << 8) | rx_buf[2];
        v_int = (rx_buf[3] << 4) | (rx_buf[4] >> 4);
        t_int = ((rx_buf[4] & 0xF) << 8) | rx_buf[5];
        ins->pitch_motor->get_data_.mos_temperture = rx_buf[6];
        ins->pitch_motor->get_data_.motor_temperture = rx_buf[7];
        ins->pitch_motor->get_data_.postion = DM_Motor_n::uint_to_float(p_int, -2, 2, 16);    // (-12.5,12.5)
        ins->pitch_motor->get_data_.velocity = DM_Motor_n::uint_to_float(v_int, -45, 45, 12); // (-45.0,45.0)
        ins->pitch_motor->get_data_.toeque = DM_Motor_n::uint_to_float(t_int, -18, 18, 12);   //(-18.0,18.0)
    }
// endregion

    /* region 宏以及变量声明 */
#define RAD_TO_ANGLE               57.29f // 弧度转角度的系数
#define DISABLE_TIME               0.5f   // 失能处理时间
#define CHANGE_STATE_TIME          2.0f   // 摇杆切换状态时间

Gimbal_c* this_ptr = nullptr;
user_maths_c math_;
// endregion

    /* region 实例创建 */
    Gimbal_c* Gimbal_c::Get_InstancePtr()
    {
        static Gimbal_c _instance;
        this_ptr = &_instance;
        return &_instance;
    }

    Gimbal_c::Gimbal_c()
    {
        this->Yaw_Init();
        this->Pitch_Init();
        this->IMU_Init();
        this->cmd_instance   = RobotCMD_n::RobotCMD_c::Get_InstancePtr();
        this->timer_instance = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
        this->is_loop = true;
    }
// endregion

    /* region 初始化 */
    void Gimbal_c::Yaw_Init()
    {
        Motor_General_Def_n::Motor_Init_Config_s _config = {
                .controller_param_init_config = {
//                        .current_PID = {
//                                .Kp = 1,
//                                .Ki = 0,
//                                .Kd = 0,
//                                .ActualValueSource = NULL,
//                                .mode = Output_Limit | Integral_Limit,
//                                .max_out = 0,
//                                .max_Ierror = 200, // (O,o)? 没积分为什么要限幅
//                                .deadband = 5,
//                        },
                        .speed_PID = {
                                .Kp = YAW_MOTOR_PID_A_KP,
                                .Ki = YAW_MOTOR_PID_A_KI,
                                .Kd = YAW_MOTOR_PID_A_KD,
                                .ActualValueSource = NULL,
                                .mode = Output_Limit | Integral_Limit,
                                .max_out = YAW_MOTOR_SPEED_MAX_OUT,
                                .max_Ierror = 200,
                                .deadband = 0.3,
                        },
                        .angle_PID = {
                                .Kp = YAW_MOTOR_PID_S_KP,
                                .Ki = YAW_MOTOR_PID_S_KI,
                                .Kd = YAW_MOTOR_PID_S_KD,
                                .ActualValueSource = NULL,
                                .mode = Output_Limit | Integral_Limit,
                                .max_out = 16000, // 角度转出角速度后最大为300
                                .max_Ierror = 200,
                                .deadband = 0.3,
                        },
                },
                .controller_setting_init_config = {
                        .outer_loop_type = Motor_General_Def_n::ANGLE_LOOP,
                        .close_loop_type = Motor_General_Def_n::ANGLE_AND_SPEED_LOOP,
                        .motor_reverse_flag = Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
                        .feedback_reverse_flag = Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
                        .angle_feedback_source = Motor_General_Def_n::MOTOR_FEED, // 陀螺仪反馈
                        .speed_feedback_source = Motor_General_Def_n::MOTOR_FEED, // 一般使用陀螺仪反馈
                },
                .motor_type = Motor_General_Def_n::GM6020,
                .can_init_config = {
                        .can_handle = &hcan1,
                        .tx_id = 2, // 看电调闪几下就填几
                },
                .zero_offset = 120,
        };
        this->yaw_motor = new DJI_Motor_n::DJI_Motor_Instance(_config);
    }

    void Gimbal_c::Pitch_Init()
    {
        DM_Motor_n::DM_ModePrame_s _config = {
                // 速度kp-0.00372 0.01572
                .kp_max = 500,
                .kp_min = 0,
                .kd_min = 0,
                .kd_max = 5,
                .v_min = -30,
                .v_max = 30,
                .p_min = -2,
                .p_max = 2,
                .t_min = -10,
                .t_max = 10,
                // 位置速度
                .postion_bits = 16,
                .velocity_bits = 12,
                .toeque_bits = 12,
                .kp_bits = 12,
                .kd_bits = 12,
                .can_init_config = {
                        .can_handle = &hcan1,
                        .tx_id = 0x01,
                        .rx_id = 0x11,
                        .SAND_IDE = CAN_ID_STD,
                },
                .output = {.p_des = 0, .v_des = 0, .Kp = 0, .Kd = 0, .t_des = 0},
        };
        this->pitch_motor = new DM_Motor_n::DM_Mit_Mode_c(&_config);
        this->pitch_motor->ECF_SetRxCallBack(pitch_rx_call_back);
        this->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);

        pid_alg_n::PID_Init_Config_t _pid_config = {
                .Kp = PITCH_MOTOR_PID_A_KP,
                .Ki = PITCH_MOTOR_PID_A_KI,
                .Kd = PITCH_MOTOR_PID_A_KD,
                .ActualValueSource = NULL,
                .mode = Output_Limit | Integral_Limit,
                .max_out = 16000, // 角度转出角速度后最大为300
                .max_Ierror = 200,
                .deadband = 0.3,
        };
        this->pitch_angle_pid = new pid_alg_n::pid_alg_c(_pid_config);
    }

    void Gimbal_c::IMU_Init()
    {
        this->imu = new BMI088Instance_c();
        this->imu->BMI088Init(&hspi1,
                              1,
                              CS1_ACCEL_GPIO_Port,
                              CS1_ACCEL_Pin,
                              CS1_GYRO_GPIO_Port,
                              CS1_GYRO_Pin);
        this->imu_date = this->imu->Get_INS_Data_Point();
    }
// endregion

    /* region 功能函数 */
    inline void Gimbal_c::Change_PIDWithEncoder()
    {
        this->target_yaw = this->yaw_motor->MotorMeasure.measure.record_ecd;
        this->target_pitch = this->pitch_motor->get_data_.postion;
        this->Yaw_SetOutput_Encoder();
        this->Pitch_SetOutput_Encoder();
    }

    inline void Gimbal_c::Change_PIDWithIMU()
    {
        this->target_yaw = this->actual_yaw;
        this->target_pitch = this->actual_pitch;
        this->Yaw_SetOutput_IMU();
        this->Pitch_SetOutput_IMU();
    }

    inline void Gimbal_c::Update_ActualAngle()
    {
        // 看情况加滤波
        this->actual_yaw = this->imu_date->Yaw;
        this->actual_pitch = this->imu_date->Pitch;
    }

    void Gimbal_c::Yaw_AngleLimit()
    {
        // math_.loop_fp32_constrain()

        // 转为int防止问题
        float can_increased = (YAW_ECD_MAX - (int16_t)this->yaw_motor->MotorMeasure.measure.feedback_ecd) / 8192.0f * 360.0f;
        float can_decreased = ((int16_t)this->yaw_motor->MotorMeasure.measure.feedback_ecd - YAW_ECD_MIN) / 8192.0f * 360.0f;
        float yaw_min = this->actual_yaw - can_increased; // YAW周和编码值相反 ,yaw_min为正，yaw_max为负
        float yaw_max = this->actual_yaw + can_decreased;
        // yaw_min = filter_alg_n::Recursive_ave_filter(&yaw_filter, yaw_min,20);
        // yaw_max = filter_alg_n::Recursive_ave_filter(&yaw_filter, yaw_max,20);
        if ((YAW_ECD_MAX - (int16_t)this->yaw_motor->MotorMeasure.measure.feedback_ecd) < -1500 ||
        ((int16_t)this->yaw_motor->MotorMeasure.measure.feedback_ecd - YAW_ECD_MIN) < -1500)
        {
            yaw_motor->DJIMotorStop(); // (O,o)! 需要修改急停方案
        }
        this->target_yaw = math_.user_val_limit(this->target_yaw, yaw_min, yaw_max);
        // 机头向前陀螺仪左正右负，编码值左小右大
        //  滤波
    }

    void Gimbal_c::Pitch_AngleLimit()
    {
        float can_increased = user_abs((PITCH_POS_MAX - this->pitch_motor->get_data_.postion) * RAD_TO_ANGLE);
        float can_decreased = user_abs((this->pitch_motor->get_data_.postion - PITCH_POS_MIN) * RAD_TO_ANGLE);
        float pitch_min = actual_pitch - can_decreased;
        float pitch_max = actual_pitch + can_increased;
        // 滤波
        // pitch_min = filter_alg_n::Recursive_ave_filter(&pitch_filter, pitch_min,20);
        // pitch_max = filter_alg_n::Recursive_ave_filter(&pitch_filter, pitch_max,20);
        // if (can_increased<-10||can_decreased>10) //陀螺仪出现问题
        // {
        //     pitch_m->disable();
        //     //考虑启用编码闭环
        //     // pitch_max = actual_pitch - 5;
        //     // pitch_min = actual_pitch + 5;
        // }
        this->target_pitch = math_.user_val_limit(this->target_pitch, pitch_min, pitch_max);
    }

    inline void Gimbal_c::Pitch_SetOutput_Encoder()
    {
        this->pitch_motor->Postion();
        this->pitch_motor->HandOver(this->target_pitch,0,PITCH_MOTOR_PID_A_KP,PITCH_MOTOR_PID_A_KD,0);
    }

    inline void Gimbal_c::Pitch_SetOutput_IMU()
    {
        this->pitch_motor->Torque();
        float _output = this->pitch_angle_pid->ECF_PID_Calculate(this->target_pitch, this->actual_pitch);
        this->pitch_motor->HandOver(0,0,0,0,_output);
    }

    inline void Gimbal_c::Yaw_SetOutput_Encoder()
    {
        this->yaw_motor->motor_settings.angle_feedback_source = Motor_General_Def_n::MOTOR_FEED;
        this->yaw_motor->DJIMotorSetRef(this->target_yaw);
    }

    inline void Gimbal_c::Yaw_SetOutput_IMU()
    {
        if(this->yaw_motor->motor_settings.angle_feedback_source != Motor_General_Def_n::OTHER_FEED)
        {
            this->yaw_motor->Set_ANGLE_PID_other_feedback(&this->actual_yaw);
        }
        this->yaw_motor->DJIMotorSetRef(this->target_yaw);
    }

    inline void Gimbal_c::CtrlMove_DR16()
    {
        this->target_yaw   += this->cmd_instance->Get_RC_RJoyLRValue() * YAW_SENSOR_RC;
        this->target_pitch += this->cmd_instance->Get_RC_RJoyUDValue() * PITCH_SENSOR_RC;
        // (O,o)! 看25的代码疑似编码器模式与陀螺仪模式pitch上下翻转，调试需要注意
    }

    inline void Gimbal_c::CtrlMove_TC()
    {
        this->target_yaw   -= this->cmd_instance->Get_TC_MouseXValue() * YAW_SENSOR_TC;
        this->target_pitch += this->cmd_instance->Get_TC_MouseYValue() * PITCH_SENSOR_TC;
    }

    void Gimbal_c::CtrlMove_Auto()
    {
        // todo 相机的通讯
    }
// endregion

    /* region 状态机 */
    void Gimbal_c::ChangeState(Gimbal_n::Gimbal_State_e _new_state)
    {
        this->is_loop = false;
        this->StateExit();
        this->current_state = _new_state;
        this->StateStart();
        this->is_loop = true;
    }

    void Gimbal_c::StateStart(void)
    {
        switch (this->current_state)
        {
            case Disable:
                this->Change_PIDWithEncoder();
                this->timer_delta_t = 0;
                this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
                break;
            case ControlWithEncoder:
                this->Change_PIDWithEncoder();
                this->timer_delta_t = 0;
                this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
                break;
            case ControlWithIMU:
                this->Change_PIDWithIMU();
                this->timer_delta_t = 0;
                this->timer_instance->ECF_DWT_GetDeltaT(&this->timer_cnt);
                break;
            case AutoControl:
                this->Change_PIDWithIMU();
                break;
            default:
                break;
        }
    }

    void Gimbal_c::StateExit(void)
    {
        switch (this->current_state)
        {
            case Disable:
                this->yaw_motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch_angle_pid->pid_measure_.Ierror = 0;
                break;
            case ControlWithEncoder:
                this->yaw_motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch_angle_pid->pid_measure_.Ierror = 0;
                break;
            case ControlWithIMU:
                this->yaw_motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch_angle_pid->pid_measure_.Ierror = 0;
                break;
            case AutoControl:
                this->yaw_motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch_angle_pid->pid_measure_.Ierror = 0;
                break;
            default:
                break;
        }
    }

    void StateLoop(void)
    {
        if(this_ptr == nullptr)return;
        if(!this_ptr->is_loop)return;

        this_ptr->imu->BMI_UpData();
        this_ptr->Update_ActualAngle();

        if(this_ptr->cmd_instance->connect_state == RobotCMD_n::DR16_CMD)
        {
            switch (this_ptr->current_state)
            {
                case Disable: // region Disable
                    /*
                     * 检测 拨杆居中 切换 CWI 状态
                     * 检测 拨杆上拨 切换 Auto 状态
                     * 计时器未到指定时间 调用限位检测函数
                     * 计时器未到指定时间 pitch 填充发送数据
                     * 计时器未到指定时间 计时器计时
                     * 计时器未到指定时间 两电机使能
                     * 计时器到指定时间 两电机失能
                     */
                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_MIDDLE)
                    {
                        this_ptr->ChangeState(ControlWithIMU);
                        return;
                    }
                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
                    {
                        this_ptr->ChangeState(AutoControl);
                        return;
                    }
                    if(this_ptr->timer_delta_t < DISABLE_TIME)
                    {
                        this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                        this_ptr->yaw_motor->DJIMotorEnable();
                        this_ptr->Yaw_AngleLimit();
                        this_ptr->Pitch_AngleLimit();
                        this_ptr->Yaw_SetOutput_Encoder();
                        this_ptr->Pitch_SetOutput_Encoder();
                        if(this_ptr->pitch_motor->get_data_.nowState != DM_Motor_n::ENABLE)
                        {
                            this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE); // (O,o)! 设置状态要在发送缓冲区填充后
                        }
                    }
                    else
                    {
                        this_ptr->yaw_motor->DJIMotorStop();
                        this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_UNABLE); // (O,o)? 失能下CAN总线仍会发送信息，是否有问题
                    }
                    break;
                    // endregion
                case ControlWithEncoder: // region ControlWithEncoder
                    /*
                     * 电机使能
                     * 调用CtrlMove函数，更新目标值
                     * 调用限位检测函数
                     * 填充发送数据
                     * 检测 拨杆下拨 切换 Disable 状态
                     * 检测 拨杆上拨 切换 Auto 状态
                     * 检测 左拨杆右上角 计时器计时
                     * 检测 左拨杆右上角 计时器到指定时间 切换 CWI 状态
                     * 检测 左拨杆不在右上角 计时器清零
                     */
                    this_ptr->yaw_motor->DJIMotorEnable();
                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }
                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
                    {
                        this_ptr->ChangeState(AutoControl);
                        return;
                    }
                    if(this_ptr->cmd_instance->Get_RC_LJoyLRValue() < 640 && this_ptr->cmd_instance->Get_RC_LJoyUDValue() > 640)
                    {
                        if(this_ptr->timer_delta_t < CHANGE_STATE_TIME)
                        {
                            this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                        }
                        else
                        {
                            this_ptr->ChangeState(ControlWithIMU);
                        }
                    }
                    else
                    {
                        this_ptr->timer_delta_t = 0;
                    }
                    this_ptr->CtrlMove_DR16();
                    this_ptr->Yaw_AngleLimit();
                    this_ptr->Pitch_AngleLimit();
                    this_ptr->Yaw_SetOutput_Encoder();
                    this_ptr->Pitch_SetOutput_Encoder();
                    if(this_ptr->pitch_motor->get_data_.nowState != DM_Motor_n::ENABLE)
                    {
                        this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                    }
                    break;
                    // endregion
                case ControlWithIMU: // region ControlWithIMU
                    /*
                     * 电机使能
                     * 调用CtrlMove函数，更新目标值
                     * 调用限位检测函数
                     * pitch 角度环计算，填充发送数据
                     * 检测 拨杆下拨 切换 Disable 状态
                     * 检测 拨杆上拨 切换 Auto 状态
                     * 检测 左拨杆右下角 计时器计时
                     * 检测 左拨杆右下角 计时器到指定时间 切换 CWE 状态
                     * 检测 左拨杆不在右下角 计时器清零
                     */
                    this_ptr->yaw_motor->DJIMotorEnable();
                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }
                    else if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
                    {
                        this_ptr->ChangeState(AutoControl);
                        return;
                    }
                    if(this_ptr->cmd_instance->Get_RC_LJoyLRValue() > 640 && this_ptr->cmd_instance->Get_RC_LJoyUDValue() < -640)
                    {
                        if(this_ptr->timer_delta_t < CHANGE_STATE_TIME)
                        {
                            this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
                        }
                        else
                        {
                            this_ptr->ChangeState(ControlWithEncoder);
                        }
                    }
                    else
                    {
                        this_ptr->timer_delta_t = 0;
                    }
                    this_ptr->CtrlMove_DR16();
                    this_ptr->Yaw_AngleLimit();
                    this_ptr->Pitch_AngleLimit();
                    this_ptr->Yaw_SetOutput_IMU();
                    this_ptr->Pitch_SetOutput_IMU();
                    if(this_ptr->pitch_motor->get_data_.nowState != DM_Motor_n::ENABLE)
                    {
                        this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                    }
                    break;
                    // endregion
                case AutoControl: // region AutoControl
                    /*
                     * 电机使能
                     * 调用CtrlMove_Auto函数，更新目标值
                     * 调用限位检测函数
                     * pitch 角度环计算，填充发送数据
                     * 检测 拨杆下拨 切换 Disable 状态
                     * 检测 拨杆居中 切换 CWI 状态
                     */
                    this_ptr->yaw_motor->DJIMotorEnable();
                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_LOW)
                    {
                        this_ptr->ChangeState(Disable);
                        return;
                    }
                    else if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_MIDDLE)
                    {
                        this_ptr->ChangeState(ControlWithIMU);
                        return;
                    }
                    this_ptr->CtrlMove_Auto();
                    this_ptr->Yaw_AngleLimit();
                    this_ptr->Pitch_AngleLimit();
                    this_ptr->Yaw_SetOutput_IMU();
                    this_ptr->Pitch_SetOutput_IMU();
                    if(this_ptr->pitch_motor->get_data_.nowState != DM_Motor_n::ENABLE)
                    {
                        this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                    }
                    break;
                    // endregion
                default:
                    break;
            }
        } // DT7下的状态机
        else if(this_ptr->cmd_instance->connect_state == RobotCMD_n::TC_CMD)
        {
            switch (this_ptr->current_state)
            {
                case Disable: // region Disable
                    /*
                     * 检测到连接直接进入CWI
                     */
                    this_ptr->ChangeState(ControlWithIMU);
                    return;
                    // endregion
                case ControlWithEncoder: // region ControlWithEncoder
                    /*
                     * 搁置此状态
                     */
                    this_ptr->ChangeState(ControlWithIMU);
                    return;
//                    this_ptr->yaw_motor->DJIMotorEnable();
//                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_LOW)
//                    {
//                        this_ptr->ChangeState(Disable);
//                        return;
//                    }
//                    if(this_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
//                    {
//                        this_ptr->ChangeState(AutoControl);
//                        return;
//                    }
//                    if(this_ptr->cmd_instance->Get_RC_LJoyLRValue() < -640 && this_ptr->cmd_instance->Get_RC_LJoyUDValue() > 640)
//                    {
//                        if(this_ptr->timer_delta_t < CHANGE_STATE_TIME)
//                        {
//                            this_ptr->timer_delta_t += this_ptr->timer_instance->ECF_DWT_GetDeltaT(&this_ptr->timer_cnt);
//                        }
//                        else
//                        {
//                            this_ptr->ChangeState(ControlWithIMU);
//                        }
//                    }
//                    else
//                    {
//                        this_ptr->timer_delta_t = 0;
//                    }
//                    /* CtrlMove */
//                    this_ptr->Yaw_AngleLimit();
//                    this_ptr->Pitch_AngleLimit();
//                    this_ptr->Yaw_SetOutput_Encoder();
//                    this_ptr->Pitch_SetOutput_Encoder();
//                    if(this_ptr->pitch_motor->get_data_.nowState != DM_Motor_n::ENABLE)
//                    {
//                        this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
//                    }
                    // endregion
                case ControlWithIMU: // region ControlWithIMU
                    /*
                     * 电机使能
                     * 调用CtrlMove函数，更新目标值
                     * 调用限位检测函数
                     * pitch 角度环计算，填充发送数据
                     * 检测 右键 按下，切换Auto状态
                     */
                    this_ptr->yaw_motor->DJIMotorEnable();
                    if(this_ptr->cmd_instance->Check_TC_KeyDown('r',this_ptr->cmd_instance->TC_cmd->mouse.press_r))
                    {
                        this_ptr->ChangeState(AutoControl);
                        return;
                    }
                    this_ptr->CtrlMove_TC();
                    this_ptr->Yaw_AngleLimit();
                    this_ptr->Pitch_AngleLimit();
                    this_ptr->Yaw_SetOutput_IMU();
                    this_ptr->Pitch_SetOutput_IMU();
                    if(this_ptr->pitch_motor->get_data_.nowState != DM_Motor_n::ENABLE)
                    {
                        this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                    }
                    break;
                    // endregion
                case AutoControl: // region AutoControl
                    /*
                     * 电机使能
                     * 调用CtrlMove_Auto函数，更新目标值
                     * 调用限位检测函数
                     * pitch 角度环计算，填充发送数据
                     * 检测 右键 松开，切换CWI状态
                     */
                    this_ptr->yaw_motor->DJIMotorEnable();
                    if(this_ptr->cmd_instance->Check_TC_KeyUp('r',this_ptr->cmd_instance->TC_cmd->mouse.press_r))
                    {
                        this_ptr->ChangeState(ControlWithIMU);
                        return;
                    }
                    this_ptr->CtrlMove_Auto();
                    this_ptr->Yaw_AngleLimit();
                    this_ptr->Pitch_AngleLimit();
                    this_ptr->Yaw_SetOutput_IMU();
                    this_ptr->Pitch_SetOutput_IMU();
                    if(this_ptr->pitch_motor->get_data_.nowState != DM_Motor_n::ENABLE)
                    {
                        this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                    }
                    break;
                    // endregion
                default:
                    break;
            }
        } // TC下的状态机
        else
        {
            this_ptr->ChangeState(Disable);
            this_ptr->yaw_motor->DJIMotorStop();
            this_ptr->pitch_motor->DMMotorStateSet(DM_Motor_n::DM_UNABLE);
        }
    }
// endregion
}


