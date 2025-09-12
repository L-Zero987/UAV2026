#include "gimbal.hpp"
extern "C"
{
#include "main.h"
#include "spi.h"
}

namespace Gimbal_n
{
    /* region 宏以及变量声明 */
#define RAD_TO_ANGLE               57.29f // 弧度转角度的系数
#define DISABLE_TIME               0.5f   // 失能处理时间
#define CHANGE_STATE_TIME          2.0f   // 摇杆切换状态时间

    Gimbal_c* gimbal_ptr = nullptr;
    user_maths_c math_;
    // endregion

    /* region 神秘DM框架适配 */
    void pitch_rx_call_back(BSP_CAN_Part_n::CANInstance_c *registerinstance)
    {
        int p_int, v_int, t_int;
        uint8_t rx_buf[8] = {0};
        memcpy(rx_buf, registerinstance->rx_buff, 8); // 存储数据，防止变化
//        ins->pitch_motor->get_data_.id = (rx_buf[0]) & 0x0F;
        gimbal_ptr->pitch->motor->get_data_.nowState = (DM_Motor_n::DM_NowState_e)(rx_buf[0] >> 4);
        p_int = (rx_buf[1] << 8) | rx_buf[2];
        v_int = (rx_buf[3] << 4) | (rx_buf[4] >> 4);
        t_int = ((rx_buf[4] & 0xF) << 8) | rx_buf[5];
        gimbal_ptr->pitch->motor->get_data_.mos_temperture = rx_buf[6];
        gimbal_ptr->pitch->motor->get_data_.motor_temperture = rx_buf[7];
        gimbal_ptr->pitch->motor->get_data_.postion = DM_Motor_n::uint_to_float(p_int, -2, 2, 16);    // (-12.5,12.5)
        gimbal_ptr->pitch->motor->get_data_.velocity = DM_Motor_n::uint_to_float(v_int, -45, 45, 12); // (-45.0,45.0)
        gimbal_ptr->pitch->motor->get_data_.toeque = DM_Motor_n::uint_to_float(t_int, -18, 18, 12);   //(-18.0,18.0)
    }
    // endregion

    /* region 实例创建 */
    Gimbal_c* Gimbal_c::Get_InstancePtr()
    {
        static Gimbal_c _instance;
        gimbal_ptr = &_instance;
        return &_instance;
    }

    Gimbal_c::Gimbal_c()
    {
        this->IMU_Init();
        this->yaw            = Yaw_c::Get_InstancePtr();
        this->pitch          = Pitch_c::Get_InstancePtr();
        this->cmd_instance   = RobotCMD_n::RobotCMD_c::Get_InstancePtr();
        this->timer_instance = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
        this->is_loop = true;
    }

    Yaw_c* Yaw_c::Get_InstancePtr()
    {
        static Yaw_c _instance;
        return &_instance;
    }

    Pitch_c* Pitch_c::Get_InstancePtr()
    {
        static Pitch_c _instance;
        return &_instance;
    }

    Yaw_c::Yaw_c()
    {
        this->Motor_Init();
        this->Model_Init();
    }

    Pitch_c::Pitch_c()
    {
        this->Motor_Init();
        this->Model_Init();
    }

// endregion

    /* region 初始化 */
    void Yaw_c::Motor_Init()
    {
#if (GIMBAL_IS_USE_PID == 0x01u)
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
        this->motor = new DJI_Motor_n::DJI_Motor_Instance(_config);
#else
        // 电机编码值闭环使用pid，imu闭环使用lqr
        Motor_General_Def_n::Motor_Init_Config_s _config = {
                .controller_param_init_config = {
                        .current_PID = {
                                .Kp = 1,
                                .Ki = 0,
                                .Kd = 0,
                                .ActualValueSource = NULL,
                                .mode = Output_Limit | Integral_Limit,
                                .max_out = 0,
                                .max_Ierror = 200,
                                .deadband = 5,
                        },
                        .speed_PID = {
                                .Kp = YAW_MOTOR_PID_S_KP,
                                .Ki = YAW_MOTOR_PID_S_KI,
                                .Kd = YAW_MOTOR_PID_S_KD,
                                .ActualValueSource = NULL,
                                .mode = Output_Limit | Integral_Limit,
                                .max_out = YAW_MOTOR_SPEED_MAX_OUT,
                                .max_Ierror = 200,
                                .deadband = 0.3,
                        },
                        .angle_PID = {
                                .Kp = YAW_MOTOR_PID_A_KP,
                                .Ki = YAW_MOTOR_PID_A_KI,
                                .Kd = YAW_MOTOR_PID_A_KD,
                                .ActualValueSource = NULL,
                                .mode = Output_Limit | Integral_Limit,
                                .max_out = 16000, // 角度转出角速度后最大为300
                                .max_Ierror = 200,
                                .deadband = 0.3,
                        },
                },
                .controller_setting_init_config = {
                        .outer_loop_type = Motor_General_Def_n::ANGLE_LOOP, // lqr直接开环
                        .close_loop_type = Motor_General_Def_n::OPEN_LOOP,
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
        this->motor = new DJI_Motor_n::DJI_Motor_Instance(_config);
#endif
    }

    void Pitch_c::Motor_Init()
    {
#if (GIMBAL_IS_USE_PID == 0x01u)
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
        this->motor = new DM_Motor_n::DM_Mit_Mode_c(&_config);
        this->motor->ECF_SetRxCallBack(pitch_rx_call_back);
        this->motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
#else
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
                .output = {.p_des = 0, .v_des = 0, .Kp = PITCH_MOTOR_PID_A_KP, .Kd = PITCH_MOTOR_PID_A_KP, .t_des = 0},
        };
        this->motor = new DM_Motor_n::DM_Mit_Mode_c(&_config);
        this->motor->ECF_SetRxCallBack(pitch_rx_call_back);
        this->motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
#endif
    }

    void Yaw_c::Model_Init()
    {
#if (GIMBAL_IS_USE_PID == 0x01u)
        // PID已在电机框架内置
#else
        this->lqr = new lqr_alg_n::lqr_alg_c(2, 1, this->k_lqr);
        pid_alg_n::PID_Init_Config_t _config = {
            .Kp = 0,
            .Ki = -0.0006f, // 0.02
            .Kd = 0,
            .Kfa = 0,
            .Kfb = 0,
            .ActualValueSource = &pos_,
            .mode = Integral_Limit | Separated_Integral | Feedforward,
            .max_out = 600,
            .max_Ierror = 200,
            .deadband = 0,
            .threshold_max = 0.5,
            .threshold_min = -0.5,
            .errorabsmax = 0.2,
            .errorabsmin = 0,
        };
        this->pid_i = new pid_alg_n::pid_alg_c(_config);
#endif
    }

    void Pitch_c::Model_Init()
    {
#if (GIMBAL_IS_USE_PID == 0x01u)
        pid_alg_n::PID_Init_Config_t _pid_config = {
                .Kp = PITCH_MOTOR_PID_A_KP,
                .Ki = PITCH_MOTOR_PID_A_KI,
                .Kd = PITCH_MOTOR_PID_A_KD,
                .ActualValueSource = NULL,
                .mode = Output_Limit | Integral_Limit,
                .max_out = 160, // (O,o)? 需要调
                .max_Ierror = 200,
                .deadband = 0.3,
        };
        this->pid_angle = new pid_alg_n::pid_alg_c(_pid_config);
#else
        this->lqr = new lqr_alg_n::lqr_alg_c(2, 1, this->k_lqr);
        pid_alg_n::PID_Init_Config_t _config = {
            .Kp = 0,
            .Ki = PITCH_MOTOR_PID_I_KI,
            .Kd = 0,
            .Kfa = 10,
            .Kfb = 0,
            .ActualValueSource = &set_pos_,
            .mode = Integral_Limit | Separated_Integral | Feedforward | Output_Limit,
            .max_out = 3,
            .max_Ierror = 800,
            .deadband = 0,
            .threshold_max = 2.5,
            .threshold_min = -2.5,
            .errorabsmax = 4,
            .errorabsmin = 0,
            .stepIn = 3,
        };
        this->pid_i = new pid_alg_n::pid_alg_c(_config);
#endif
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
    /**
     * @brief 6020参数转换,力矩转换成电压
     *
     */
    float torque_to_voltage_6020(float torque)
    {
        float voltage = 0.0f;

        float current = torque / 0.741f * 10000;
        voltage = (current - 128.3507f) / 0.7778f;
        voltage = math_.user_val_limit(voltage, -25000, 25000);
        return voltage;
    }

    inline void Gimbal_c::Change_PIDWithEncoder()
    {
        this->target_yaw = this->yaw->motor->MotorMeasure.measure.record_ecd;
        this->target_pitch = this->pitch->motor->get_data_.postion;
        this->yaw->SetOutput_Encoder();
        this->pitch->SetOutput_Encoder();
    }

    inline void Gimbal_c::Change_PIDWithIMU()
    {
        this->target_yaw = this->actual_yaw;
        this->target_pitch = this->actual_pitch;
        this->yaw->SetOutput_IMU();
        this->pitch->SetOutput_IMU();
    }

    inline void Gimbal_c::Update_ActualAngle()
    {
        // 看情况加滤波
        this->imu->BMI_UpData();
        this->actual_yaw = this->imu_date->Yaw;
        this->actual_pitch = this->imu_date->Pitch;
    }

    void Yaw_c::AngleLimit()
    {
        // math_.loop_fp32_constrain()

        // 转为int防止问题
        float can_increased = (YAW_ECD_MAX - (int16_t)this->motor->MotorMeasure.measure.feedback_ecd) / 8192.0f * 360.0f;
        float can_decreased = ((int16_t)this->motor->MotorMeasure.measure.feedback_ecd - YAW_ECD_MIN) / 8192.0f * 360.0f;
        float yaw_min = gimbal_ptr->actual_yaw - can_increased; // YAW周和编码值相反 ,yaw_min为正，yaw_max为负
        float yaw_max = gimbal_ptr->actual_yaw + can_decreased;
        // yaw_min = filter_alg_n::Recursive_ave_filter(&yaw_filter, yaw_min,20);
        // yaw_max = filter_alg_n::Recursive_ave_filter(&yaw_filter, yaw_max,20);
        if ((YAW_ECD_MAX - (int16_t)this->motor->MotorMeasure.measure.feedback_ecd) < -1500 ||
        ((int16_t)this->motor->MotorMeasure.measure.feedback_ecd - YAW_ECD_MIN) < -1500)
        {
            this->motor->DJIMotorStop(); // (O,o)! 需要修改急停方案
        }
        gimbal_ptr->target_yaw = math_.user_val_limit(gimbal_ptr->target_yaw, yaw_min, yaw_max);
        // 机头向前陀螺仪左正右负，编码值左小右大
        //  滤波
    }

    void Yaw_c::SetOutput_Encoder()
    {
        this->motor->motor_settings.close_loop_type = Motor_General_Def_n::ANGLE_AND_SPEED_LOOP;
        this->motor->motor_settings.angle_feedback_source = Motor_General_Def_n::MOTOR_FEED;
        this->motor->DJIMotorSetRef(gimbal_ptr->target_yaw * 22.7f);// 8192每一度为22.7 (O,o)!要宏定义
    }

    void Yaw_c::SetOutput_IMU()
    {
#if (GIMBAL_IS_USE_PID == 0x01u)
        this->motor->motor_settings.close_loop_type = Motor_General_Def_n::OPEN_LOOP;
        if(this->motor->motor_settings.angle_feedback_source != Motor_General_Def_n::OTHER_FEED)
        {
            this->motor->Set_ANGLE_PID_other_feedback(&gimbal_ptr->actual_yaw);
        }
        this->motor->DJIMotorSetRef(gimbal_ptr->target_yaw);
#else
        this->vel_ = this->motor->MotorMeasure.measure.feedback_speed;
        this->pos_ = gimbal_ptr->actual_yaw;
        this->set_pos_ = gimbal_ptr->target_yaw;
        this->pos_error_ = math_.float_min_distance(this->set_pos_, this->pos_, -180.0f, 180.0f);
        float yaw_system_state[2] = {(-this->pos_error_ / 57.295779513f), this->vel_}; // 输入目标差值与当前速度
        this->lqr->ECF_LQR_Data_Update(yaw_system_state);
        this->output_ = lqr->ECF_LQR_Calculate() + this->pid_i->ECF_PID_Calculate(this->set_pos_, this->pos_);
        this->output_ = torque_to_voltage_6020(output_);
#endif
    }

    void Pitch_c::AngleLimit()
    {
        float can_increased = user_abs((PITCH_POS_MAX - this->motor->get_data_.postion) * RAD_TO_ANGLE);
        float can_decreased = user_abs((this->motor->get_data_.postion - PITCH_POS_MIN) * RAD_TO_ANGLE);
        float pitch_min = gimbal_ptr->actual_pitch - can_decreased;
        float pitch_max = gimbal_ptr->actual_pitch + can_increased;
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
        gimbal_ptr->target_pitch = math_.user_val_limit(gimbal_ptr->target_pitch, pitch_min, pitch_max);
    }

    void Pitch_c::SetOutput_Encoder()
    {
        this->motor->Postion();
        this->motor->HandOver(gimbal_ptr->target_pitch,0,PITCH_MOTOR_PID_A_KP,PITCH_MOTOR_PID_A_KD,0);
    }

    void Pitch_c::SetOutput_IMU()
    {
#if (GIMBAL_IS_USE_PID == 0x01u)
        this->output_ = this->pid_angle->ECF_PID_Calculate(gimbal_ptr->target_pitch, gimbal_ptr->actual_pitch);
        this->motor->Speed();
        this->motor->HandOver(0,this->output_,0,0,0);
#else
        this->vel_ = this->motor->get_data_.velocity;
        this->pos_ = gimbal_ptr->actual_pitch;
        this->set_pos_ = gimbal_ptr->target_pitch;
        this->pos_error_ = math_.float_min_distance(this->set_pos_, this->pos_, -90, 90.0f);
        // 力控好用
        float toque_system_state[2] = {pos_error_ / 57.295779513f, vel_};
        this->lqr->ECF_LQR_Data_Update(toque_system_state);
        this->output_ = this->lqr->ECF_LQR_Calculate() + this->pid_i->ECF_PID_Calculate(this->set_pos_, this->pos_); // + pitch_gravity_compensate(pos_); (O,o)! pitch补偿
        this->output_ = math_.user_val_limit(output_, -7.0f, 7.0f); // 中间变量
        this->motor->Torque();
        this->motor->HandOver(0,0,0,0,this->output_);
#endif
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
                this->yaw->motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch->pid_angle->pid_measure_.Ierror = 0;
                break;
            case ControlWithEncoder:
                this->yaw->motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch->pid_angle->pid_measure_.Ierror = 0;
                break;
            case ControlWithIMU:
                this->yaw->motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch->pid_angle->pid_measure_.Ierror = 0;
                break;
            case AutoControl:
                this->yaw->motor->motor_controller.angle_PID.pid_measure_.Ierror = 0;
                this->pitch->pid_angle->pid_measure_.Ierror = 0;
                break;
            default:
                break;
        }
    }

    void StateLoop(void)
    {
        // 确定启动
        if(gimbal_ptr == nullptr)return;
        if(!gimbal_ptr->is_loop)return;

        // 更新数据
        gimbal_ptr->Update_ActualAngle();

        // 检查遥控连接
        RobotCMD_n::RobotCMD_ConnectState_e _connect_state = gimbal_ptr->cmd_instance->connect_state; // 缓存连接状态，不要运行的时候切换连接了
        if(_connect_state == RobotCMD_n::NOT_CONNECT)
        {
            gimbal_ptr->ChangeState(Disable);
            gimbal_ptr->yaw->motor->DJIMotorStop();
            gimbal_ptr->pitch->motor->DMMotorStateSet(DM_Motor_n::DM_UNABLE);
            return;
        }

        // 状态机
        switch (gimbal_ptr->current_state)
        {
            case Disable: // region Disable
                /* * * * * * * * * * * * * * * * * ctrl with DR16
                 * 检测 拨杆居中 切换 CWI 状态
                 * 检测 拨杆上拨 切换 Auto 状态
                 * 计时器未到指定时间 调用限位检测函数
                 * 计时器未到指定时间 pitch 填充发送数据
                 * 计时器未到指定时间 计时器计时
                 * 计时器未到指定时间 两电机使能
                 * 计时器到指定时间 两电机失能
                 * * * * * * * * * * * * * * * * * ctrl with TC
                 * 直接进使能状态
                 */
                if(_connect_state == RobotCMD_n::TC_CMD)
                {
                    gimbal_ptr->ChangeState(ControlWithIMU);
                    return;
                }
                if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_MIDDLE)
                {
                    gimbal_ptr->ChangeState(ControlWithIMU);
                    return;
                }
                if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
                {
                    gimbal_ptr->ChangeState(AutoControl);
                    return;
                }
                if(gimbal_ptr->timer_delta_t < DISABLE_TIME)
                {
                    gimbal_ptr->timer_delta_t += gimbal_ptr->timer_instance->ECF_DWT_GetDeltaT(&gimbal_ptr->timer_cnt);
                    gimbal_ptr->yaw->motor->DJIMotorEnable();
                    gimbal_ptr->yaw->AngleLimit();
                    gimbal_ptr->pitch->AngleLimit();
                    gimbal_ptr->yaw->SetOutput_Encoder();
                    gimbal_ptr->pitch->SetOutput_Encoder();
                    if(gimbal_ptr->pitch->motor->get_data_.nowState != DM_Motor_n::ENABLE)
                    {
                        gimbal_ptr->pitch->motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE); // (O,o)! 设置状态要在发送缓冲区填充后
                    }
                }
                else
                {
                    gimbal_ptr->yaw->motor->DJIMotorStop();
                    gimbal_ptr->pitch->motor->DMMotorStateSet(DM_Motor_n::DM_UNABLE); // (O,o)? 失能下CAN总线仍会发送信息，是否有问题
                }
                break;
                // endregion
            case ControlWithEncoder: // region ControlWithEncoder
                /* * * * * * * * * * * * * * * * * * ctrl with DR16
                 * 电机使能
                 * 调用CtrlMove函数，更新目标值
                 * 调用限位检测函数
                 * 填充发送数据
                 * 检测 拨杆下拨 切换 Disable 状态
                 * 检测 拨杆上拨 切换 Auto 状态
                 * 检测 左拨杆右上角 计时器计时
                 * 检测 左拨杆右上角 计时器到指定时间 切换 CWI 状态
                 * 检测 左拨杆不在右上角 计时器清零
                 * * * * * * * * * * * * * * * * * * ctrl with TC
                 * 此状态搁置（直接进入 CWI 状态
                 */
                if (_connect_state == RobotCMD_n::TC_CMD)
                {
                    gimbal_ptr->ChangeState(ControlWithIMU);
                    return;
                }
                gimbal_ptr->yaw->motor->DJIMotorEnable();
                if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_LOW)
                {
                    gimbal_ptr->ChangeState(Disable);
                    return;
                }
                if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
                {
                    gimbal_ptr->ChangeState(AutoControl);
                    return;
                }
                if(gimbal_ptr->cmd_instance->Get_RC_LJoyLRValue() < 640 && gimbal_ptr->cmd_instance->Get_RC_LJoyUDValue() > 640)
                {
                    if(gimbal_ptr->timer_delta_t < CHANGE_STATE_TIME)
                    {
                        gimbal_ptr->timer_delta_t += gimbal_ptr->timer_instance->ECF_DWT_GetDeltaT(&gimbal_ptr->timer_cnt);
                    }
                    else
                    {
                        gimbal_ptr->ChangeState(ControlWithIMU);
                    }
                }
                else
                {
                    gimbal_ptr->timer_delta_t = 0;
                }
                gimbal_ptr->CtrlMove_DR16();
                gimbal_ptr->yaw->AngleLimit();
                gimbal_ptr->pitch->AngleLimit();
                gimbal_ptr->yaw->SetOutput_Encoder();
                gimbal_ptr->pitch->SetOutput_Encoder();
                if(gimbal_ptr->pitch->motor->get_data_.nowState != DM_Motor_n::ENABLE)
                {
                    gimbal_ptr->pitch->motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                }
                break;
                // endregion
            case ControlWithIMU: // region ControlWithIMU
                /* * * * * * * * * * * * * * * * * * ctrl with DR16
                 * 电机使能
                 * 检测 拨杆下拨 切换 Disable 状态
                 * 检测 拨杆上拨 切换 Auto 状态
                 * 检测 左拨杆右下角 计时器计时
                 * 检测 左拨杆右下角 计时器到指定时间 切换 CWE 状态
                 * 检测 左拨杆不在右下角 计时器清零
                 * 调用CtrlMove函数，更新目标值
                 * 调用限位检测函数
                 * pitch 角度环计算，填充发送数据
                 * * * * * * * * * * * * * * * * * * ctrl with TC
                 * 电机使能
                 * 检测 右键 按下，切换Auto状态
                 * 调用CtrlMove函数，更新目标值
                 * 调用限位检测函数
                 * pitch 角度环计算，填充发送数据
                 */
                gimbal_ptr->yaw->motor->DJIMotorEnable();
                if(_connect_state == RobotCMD_n::TC_CMD)
                {
                    if(gimbal_ptr->cmd_instance->Check_TC_KeyDown('r',gimbal_ptr->cmd_instance->TC_cmd->mouse.press_r))
                    {
                        gimbal_ptr->ChangeState(AutoControl);
                        return;
                    }
                    gimbal_ptr->CtrlMove_TC();
                }
                else
                {
                    if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_LOW)
                    {
                        gimbal_ptr->ChangeState(Disable);
                        return;
                    }
                    else if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_HIGH)
                    {
                        gimbal_ptr->ChangeState(AutoControl);
                        return;
                    }
                    if(gimbal_ptr->cmd_instance->Get_RC_LJoyLRValue() > 640 && gimbal_ptr->cmd_instance->Get_RC_LJoyUDValue() < -640)
                    {
                        if(gimbal_ptr->timer_delta_t < CHANGE_STATE_TIME)
                        {
                            gimbal_ptr->timer_delta_t += gimbal_ptr->timer_instance->ECF_DWT_GetDeltaT(&gimbal_ptr->timer_cnt);
                        }
                        else
                        {
                            gimbal_ptr->ChangeState(ControlWithEncoder);
                        }
                    }
                    else
                    {
                        gimbal_ptr->timer_delta_t = 0;
                    }
                    gimbal_ptr->CtrlMove_DR16();
                }
                gimbal_ptr->yaw->AngleLimit();
                gimbal_ptr->pitch->AngleLimit();
                gimbal_ptr->yaw->SetOutput_IMU();
                gimbal_ptr->pitch->SetOutput_IMU();
                if(gimbal_ptr->pitch->motor->get_data_.nowState != DM_Motor_n::ENABLE)
                {
                    gimbal_ptr->pitch->motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                }
                break;
                // endregion
            case AutoControl: // region AutoControl
                /* * * * * * * * * * * * * * * * * ctrl with DR16
                 * 电机使能
                 * 检测 拨杆下拨 切换 Disable 状态
                 * 检测 拨杆居中 切换 CWI 状态
                 * 调用CtrlMove_Auto函数，更新目标值
                 * 调用限位检测函数
                 * pitch 角度环计算，填充发送数据
                 * * * * * * * * * * * * * * * * * ctrl with TC
                 * 电机使能
                 * 检测 右键 松开，切换CWI状态
                 * 调用CtrlMove_Auto函数，更新目标值
                 * 调用限位检测函数
                 * pitch 角度环计算，填充发送数据
                 */
                gimbal_ptr->yaw->motor->DJIMotorEnable();
                if(_connect_state == RobotCMD_n::TC_CMD)
                {
                    if(gimbal_ptr->cmd_instance->Check_TC_KeyUp('r',gimbal_ptr->cmd_instance->TC_cmd->mouse.press_r))
                    {
                        gimbal_ptr->ChangeState(ControlWithIMU);
                        return;
                    }
                }
                else
                {
                    if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_LOW)
                    {
                        gimbal_ptr->ChangeState(Disable);
                        return;
                    }
                    else if(gimbal_ptr->cmd_instance->Get_RC_SW1State() == RobotCMD_n::CMD_MIDDLE)
                    {
                        gimbal_ptr->ChangeState(ControlWithIMU);
                        return;
                    }
                }
                gimbal_ptr->CtrlMove_Auto();
                gimbal_ptr->yaw->AngleLimit();
                gimbal_ptr->pitch->AngleLimit();
                gimbal_ptr->yaw->SetOutput_IMU();
                gimbal_ptr->pitch->SetOutput_IMU();
                if(gimbal_ptr->pitch->motor->get_data_.nowState != DM_Motor_n::ENABLE)
                {
                    gimbal_ptr->pitch->motor->DMMotorStateSet(DM_Motor_n::DM_ENABLE);
                }
                break;
                // endregion
            default:
                break;
        }
    }
// endregion
}


