// 创建电机类，can类
//  pitch:3508 ,yaw: 6020
#include "gimbal.hpp"
#include "robot_cmd.hpp"

namespace GIMBAL_N
{
#define YAW_MOTOR_SPEED_MAX_OUT 20000.0f
    user_maths_c math_;
    // float DM_kp = 8.5f;     // 大喵电机参数
    float DM_kp = 7.5f;
    // float DM_kd = 1.7f;
    float DM_kd = 1.0f;
    float DM_v = 0.1;
    float DM_T = 0;
    float k_yaw_lqr[2] = {10,0.198};    //{6, 0.175};  // 0.23};             // LQR算法参数，增益矩阵k
    float k_toque_pitch_lqr[2] = {41.5, -0.77}; // {32.0, -0.59}; // +,-
    //  设定参数矩阵
    // test
    float test = -35; // (O,o)? 好像是用来debug观测的变量
    gimbal_c *gimbal_c::instance = nullptr;
    gimbal_c *ins = nullptr;
    float check_value = 0.7;

    /**
     * @brief 开火检测，检测实际位置是否与指定位置差值小于阈值(check_value)
     *
     * @return True: 允许发射, False: 还没转到位置
     */
    bool gimbal_c::fire_check()
    {
        if ((abs(config->yaw_imu_target - actual_yaw) < check_value) && abs(config->pitch_imu_target - actual_pitch) < check_value)
        {
            return true;
        }
        return false;
        // return true;
    }

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

    /**
     * @brief 获得单例
     *
     */
    gimbal_c *gimbal_c::get_instance()
    {
        if (instance == nullptr)
        {
            instance = new gimbal_c();
            ins = instance;
        }
        return instance;
    }

    /**
     * @brief 云台任务，非类方法需要使用单例指针ins
     *
     */
    void gimbal_task(void)
    {
        ins->position_updata();
        switch (ins->config->mode)
        {
        case GIMBAL_ENABLE: // 使能
        {
            // if (!instance->init_flag)
            // {
            //     instance->init_position();
            //     return;
            // }
            if (ins->config->use_ecd) //编码闭环做校准用
            {
                if (!ins->get_center_flag) // 不在中心时回到中心位置
                {
                    ins->get_center_position(); 
                }
                else 
                {
                    ins->yaw_only_ecd();
                    //
                    ins->picth_only_ecd();
                }
            }
            else
            {
                ins->get_center_flag = false;
                ins->yaw_move();        // 转到两个轴的运动函数
                ins->pitch_move();
            }
            break;
        }
        case GIMBAL_ZERO_FORCE: // 失能
        {
            ins->config->yaw_imu_target = ins->imu_data->Yaw; //防使能出现抬头阶跃，可优化
            ins->config->pitch_imu_target = ins->imu_data->Pitch;   // (O,o)! 应该设置在失能情况下目标值不会被外部改动才对
            ins->yaw_m->disable();
            ins->pitch_m->disable();
            break;
        }
        default:
            break;
        }
    }

    /**
     * @brief yaw轴运动
     *
     */
    void gimbal_c::yaw_move()
    {
        yaw_m->motor_ins.motor_settings.close_loop_type = Motor_General_Def_n::OPEN_LOOP;
        if (yaw_m->motor_ins.MotorMeasure.measure.init_flag && !yaw_m->motor_ins.is_lost_dji) // 初始化完成并且没断联
        {
            yaw_m->enable();
            // test = config->yaw_imu_target;
            yaw_angle_limit();
            if (config->vision_is_on == AUTO_AIM_MODE)
            {
                yaw_m->calculate(config->yaw_imu_target, actual_yaw, imu_data->Gyro[2]); // 输入目标差值与当前速度
            }
            else
            {
                yaw_m->calculate_no_i(config->yaw_imu_target, actual_yaw, imu_data->Gyro[2]); // 输入目标差值与当前速度
            }
            yaw_m->set_output();
        }
        else
        {
            config->yaw_imu_target = actual_yaw; // 失能
        }
        // todo: 信任值-》目标值小于一个值时使用电机反馈，大于该值时使用陀螺仪反馈、
    }

    /**
     * @brief pitch运动
     *
     */
    void gimbal_c::pitch_move()
    {
        if (ins->init_flag)
        {
            pitch_m->enable();
            // // 设定目标需要旋转的角度以及当前轴陀螺仪的值，进行数据更新
            if (pitch_m->motor_ins.get_data_.nowState == DM_Motor_n::ENABLE)
            {
                config->pitch_imu_target = math_.user_val_limit(config->pitch_imu_target, -41.0f, 8.0f); // 中间变量
                // act = actual_pitch;
                // test = config->pitch_imu_target;
                pitch_angle_limit();
                if (config->vision_is_on == AUTO_AIM_MODE) // 自瞄跟非自瞄(没瞄上
                {
                    pitch_m->calculate(config->pitch_imu_target, actual_pitch, imu_data->Gyro[1]); // 输入目标差值与当前速度
                }
                else
                {
                    pitch_m->calculate_no_i(config->pitch_imu_target, actual_pitch, imu_data->Gyro[1]); // 输入目标差值与当前速度
                }
                pitch_m->set_output();
            }
            else
            {
                config->pitch_imu_target = actual_pitch; // 失能
                pitch_m->set_output(0);
            }
        }
    }

    gimbal_c::gimbal_c() //: pitch_firstOrder(0.9), filter_num(30)
    {
        // filter_alg_n::NotchFilter_Init(&pitch_notch, 0.2, 0.04, 1000);
    }
    gimbal_c::~gimbal_c()
    {
    }

    /**
     * @brief 达妙电机接收回调
     *
     * @param registerinstance
     */
    void pitch_rx_call_back(BSP_CAN_Part_n::CANInstance_c *registerinstance) // 够潮的dm框架
    {
        gimbal_c *ins = gimbal_c::get_instance(); // (O,o)? 这跟前面的全局变量ins以及get_instance()方法中的ins = instance;是否重复?
        int p_int, v_int, t_int;
        uint8_t rx_buf[8] = {0};
        memcpy(rx_buf, registerinstance->rx_buff, 8); // 存储数据，防止变化
        ins->pitch_m->motor_ins.get_data_.id = (rx_buf[0]) & 0x0F;
        ins->pitch_m->motor_ins.get_data_.nowState = (DM_Motor_n::DM_NowState_e)(rx_buf[0] >> 4);
        p_int = (rx_buf[1] << 8) | rx_buf[2];
        v_int = (rx_buf[3] << 4) | (rx_buf[4] >> 4);
        t_int = ((rx_buf[4] & 0xF) << 8) | rx_buf[5];
        ins->pitch_m->motor_ins.get_data_.mos_temperture = rx_buf[6];
        ins->pitch_m->motor_ins.get_data_.motor_temperture = rx_buf[7];
        ins->pitch_m->motor_ins.get_data_.postion = DM_Motor_n::uint_to_float(p_int, -2, 2, 16);    // (-12.5,12.5)
        ins->pitch_m->motor_ins.get_data_.velocity = DM_Motor_n::uint_to_float(v_int, -45, 45, 12); // (-45.0,45.0)
        ins->pitch_m->motor_ins.get_data_.toeque = DM_Motor_n::uint_to_float(t_int, -18, 18, 12);   //(-18.0,18.0)
        if (!ins->init_flag)
        {
            ins->init_flag = true;
        }
    }

    /*****************************  *******************************/
    void gimbal_c::get_center_position()
    {
        yaw_m->motor_ins.motor_settings.close_loop_type = Motor_General_Def_n::ANGLE_AND_SPEED_LOOP;
        // 以一个较慢速度回到云台中心位置
        yaw_m->enable();
        pitch_m->enable();
        yaw_m->set_output(0);
        pitch_m->set_output(0.8); // (O,o)? 为什么这里是给力矩，肉眼校0？
        if(abs(yaw_m->motor_ins.MotorMeasure.measure.record_ecd)<200)// && abs(pitch_m->motor_ins.get_data_.postion+0.5)<0.1)
        {
            get_center_flag = true;
            config->yaw_ecd_target = 0;
            config->pitch_ecd_target = 0;
        }
    }


    void gimbal_c::picth_only_ecd() //锁编码pitch不动，需要优化
    {
        pitch_m->enable();
        pitch_m->tempf_set_pos(config->pitch_ecd_target / 59.7f); // 输入目标差值与当前速度
    }

    void gimbal_c::yaw_only_ecd()
    {
        yaw_m->motor_ins.motor_settings.close_loop_type = Motor_General_Def_n::ANGLE_AND_SPEED_LOOP;
        // yaw_m->enable();
        yaw_m->set_output(config->yaw_ecd_target * 22.7f); // 8192每一度为22.7
    }

    // 电机初始化
    void gimbal_c::motor_init()
    {
        // 电机编码值闭环使用pid，imu闭环使用lqr
        Motor_General_Def_n::Motor_Init_Config_s yaw_imu_config = {
            .controller_param_init_config = {
                .current_PID = {
                    .Kp = 1,
                    .Ki = 0,
                    .Kd = 0,
                    .ActualValueSource = NULL,
                    .mode = Output_Limit | Integral_Limit,
                    .max_out = 0,
                    .max_Ierror = 200, // (O,o)? 没积分为什么要限幅
                    .deadband = 5,
                },
                .speed_PID = {
                    .Kp = 3,
                    .Ki = 0,
                    .Kd = 0.03,
                    .ActualValueSource = NULL,
                    .mode = Output_Limit | Integral_Limit,
                    .max_out = YAW_MOTOR_SPEED_MAX_OUT,
                    .max_Ierror = 200,
                    .deadband = 0.3,
                },
                .angle_PID = {
                    .Kp = 6,
                    .Ki = 0,
                    .Kd = 0.2,
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
        yaw_m = new yaw_c(yaw_imu_config); // 设置为yaw电机
        yaw_m->init();
        DM_Motor_n::DM_ModePrame_s congfig_ = {
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
            .kp_bits = 12,
            .kd_bits = 12,
            // 位置速度
            .postion_bits = 16,
            .velocity_bits = 12,
            .toeque_bits = 12,
            .can_init_config = {
                .can_handle = &hcan1,
                .tx_id = 0x01,
                .rx_id = 0x11,
                .SAND_IDE = CAN_ID_STD,
            },
            .output = {.p_des = 0, .v_des = 0, .Kp = DM_kp, .Kd = DM_kd, .t_des = 0},
        };
        DM_Motor_n::DM_ModePrame_s *pconfig = new DM_Motor_n::DM_ModePrame_s(congfig_);
        pitch_m = new pitch_c(pconfig);
        pitch_m->init();
    }

    // 总 云台初始化
    void gimbal_init(const INS_t *pINS, uint8_t use_IMU) // 重载函数，传入陀螺仪数据指针
    { // (O,o)?
        ins = gimbal_c::get_instance(); // 这跟方法内容是否重复
        ins->motor_init();
        ins->imu_data = pINS;
        /****************pitch: DM4310************/
        /***********yaw电机初始化: DJI 6020*********/
        ins->config = ROBOT_CMD_N::get_gimbal_data();
        ins->init_flag = true;
        ins->yaw_filter = {};
        ins->pitch_filter = {};
        filter_alg_n::Recursive_ave_filter_init(&ins->yaw_filter);   // 滤波器初始化
        filter_alg_n::Recursive_ave_filter_init(&ins->pitch_filter); // 滤波器初始化
    }
    float fi_num = 20;
    void init_position()
    {
        for (uint8_t i = 0; i < 20; i++)
        {
            ins->position_updata();
        }
    }

    // 位置更新（获取当前角度
    void gimbal_c::position_updata()
    {
        this->actual_yaw = imu_data->Yaw;
        // actual_pitch = imu_data->Pitch;
        // 此处可加滤波
        this->actual_pitch = this->pitch_slfilter.sliding_mean_filter(imu_data->Pitch, fi_num);
        if (this->imu_data->Roll > 90 || this->imu_data->Roll < -90)
        {
            this->config->mode = GIMBAL_ZERO_FORCE; //
        }
        // actual_yaw = yaw_slfilter.sliding_mean_filter(imu_data->Yaw, fi_num);

        // actual_pitch = pitch_firstOrder.first_order_filter(imu_data->Pitch);
        // actual_pitch = filter_alg_n::NotchFilter_Update(&pitch_notch,imu_data->Pitch);
        // actual_pitch = filter_alg_n::Recursive_ave_filter(&pitch_filter, imu_data->Pitch, 20);
        // actual_yaw = filter_alg_n::Recursive_ave_filter(&yaw_filter, imu_data->Yaw, 20);
    }
    // float max_ = 0;
    // float min_ = 0;
    // float can_in = 0;
    // float can_de = 0;

    // 编码器限位
    void gimbal_c::pitch_angle_limit()
    {
        float can_increased = user_abs((PITCH_POS_MAX - ins->pitch_m->motor_ins.get_data_.postion) * 57.29);
        float can_decreased = user_abs((ins->pitch_m->motor_ins.get_data_.postion - PITCH_POS_MIN) * 57.29);
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
        config->pitch_imu_target = math_.user_val_limit(config->pitch_imu_target, pitch_min, pitch_max);
    }
    void gimbal_c::yaw_angle_limit()
    {
        // math_.loop_fp32_constrain()

        // 转为int防止问题
        float can_increased = (YAW_ECD_MAX - (int16_t)ins->yaw_m->motor_ins.MotorMeasure.measure.feedback_ecd) / 8192.0f * 360.0f;
        float can_decreased = ((int16_t)ins->yaw_m->motor_ins.MotorMeasure.measure.feedback_ecd - YAW_ECD_MIN) / 8192.0f * 360.0f;
        float yaw_min = actual_yaw - can_increased; // YAW周和编码值相反 ,yaw_min为正，yaw_max为负
        float yaw_max = actual_yaw + can_decreased;
        // yaw_min = filter_alg_n::Recursive_ave_filter(&yaw_filter, yaw_min,20);
        // yaw_max = filter_alg_n::Recursive_ave_filter(&yaw_filter, yaw_max,20);
        if ((YAW_ECD_MAX - (int16_t)ins->yaw_m->motor_ins.MotorMeasure.measure.feedback_ecd) < -1500 || ((int16_t)ins->yaw_m->motor_ins.MotorMeasure.measure.feedback_ecd - YAW_ECD_MIN) < -1500)
        {
            ins->yaw_m->disable(); // 急停
        }
        config->yaw_imu_target = math_.user_val_limit(config->yaw_imu_target, yaw_min, yaw_max);
        // 机头向前陀螺仪左正右负，编码值左小右大
        //  滤波
    }
    void yaw_c::calculate(float set_position, float actual_pos, float actual_vel)
    {
        vel_ = actual_vel;
        pos_ = actual_pos;
        set_pos_ = set_position;
        pos_error_ = math_.float_min_distance(set_position, pos_, -180.0f, 180.0f);
        float yaw_system_state[2] = {(-pos_error_ / 57.295779513f), vel_}; // 输入目标差值与当前速度
        lqr.ECF_LQR_Data_Update(yaw_system_state);
        output_ = lqr.ECF_LQR_Calculate() + pid_only_i.ECF_PID_Calculate(set_pos_, pos_);
        output_ = torque_to_voltage_6020(output_);
    }
    void yaw_c::calculate_no_i(float set_position, float actual_pos, float actual_vel)
    {
        vel_ = actual_vel;
        pos_ = actual_pos;
        set_pos_ = set_position;
        pos_error_ = math_.float_min_distance(set_position, pos_, -180.0f, 180.0f);
        float yaw_system_state[2] = {(-pos_error_ / 57.295779513f), vel_}; // 输入目标差值与当前速度
        lqr.ECF_LQR_Data_Update(yaw_system_state);
        output_ = lqr.ECF_LQR_Calculate();
        output_ = torque_to_voltage_6020(output_);
    }
    void yaw_c::init()
    {
        lqr.ECF_LQR_Init(2, 1, k_yaw_lqr);
        pid_alg_n::PID_Init_Config_t only_i_config = {
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
        pid_only_i.pid_init(only_i_config);
    }
    yaw_c::yaw_c(Motor_General_Def_n::Motor_Init_Config_s config) : motor_ins(config)
    {
    }
    void yaw_c::set_output() // 将计算的output传给电机
    {
        motor_ins.DJIMotorSetRef(output_);
    }
    void yaw_c::set_output(float set_output_) // 将计算的output传给电机
    {
        motor_ins.DJIMotorSetRef(set_output_);
    }
    void yaw_c::enable()
    {
        motor_ins.DJIMotorEnable();
    }
    void yaw_c::disable()
    {
        motor_ins.DJIMotorStop();
    }

    void pitch_c::init()
    {
        lqr.ECF_LQR_Init(2, 1, k_toque_pitch_lqr);
        pid_alg_n::PID_Init_Config_t picth_only_i = {
            .Kp = 0,
            .Ki = 0.0023, // 0.0031
            // 0.0031   //0.00183,//0.00075, // 0.00075, // 0.0085, // 0.02
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
        pid_only_i.pid_init(picth_only_i);
        // DM_Motor_n::DM_ModePrame_s congfig_ = {
        //     // 速度kp-0.00372 0.01572
        //     .kp_max = 500,
        //     .kp_min = 0,
        //     .kd_min = 0,
        //     .kd_max = 5,
        //     .v_min = -30,
        //     .v_max = 30,
        //     .p_min = -2,
        //     .p_max = 2,
        //     .t_min = -10,
        //     .t_max = 10,
        //     .kp_bits = 12,
        //     .kd_bits = 12,
        //     // 位置速度
        //     .postion_bits = 16,
        //     .velocity_bits = 12,
        //     .toeque_bits = 12,
        //     .can_init_config = {
        //         .can_handle = &hcan1,
        //         .tx_id = 0x01,
        //         .rx_id = 0x11,
        //         .SAND_IDE = CAN_ID_STD,
        //     },
        //     .output = {.p_des = 0, .v_des = 0, .Kp = DM_kp, .Kd = DM_kd, .t_des = 0},
        // };

        // DM_Motor_n::DM_ModePrame_s *pconfig = new DM_Motor_n::DM_ModePrame_s(congfig_);
        // motor_ins = DM_Motor_n::DM_Mit_Mode_c(pconfig);
        // motor_ins.param_ = pconfig;
        motor_ins.ECF_SetRxCallBack(pitch_rx_call_back);
        motor_ins.StateSet(DM_Motor_n::DM_UNABLE);
        set_pos(-38); // 初始值设为-38

        // motor_ins.StateSet(DM_Motor_n::DM_PORTECT_ZERO_POSITION); // 设置零点
    }
    void pitch_c::enable()
    {
        motor_ins.state = DM_Motor_n::DM_ENABLE;
    }
    void pitch_c::disable()
    {
        motor_ins.state = DM_Motor_n::DM_UNABLE;
    }

    void pitch_c::calculate(float set_pos, float actual_pos, float actual_vel)
    {
        vel_ = motor_ins.get_data_.velocity;
        pos_ = actual_pos;
        set_pos_ = set_pos;
        pos_error_ = math_.float_min_distance(set_pos, actual_pos, -90, 90.0f);
        // 力控好用
        float toque_system_state[2] = {pos_error_ / 57.295779513f, vel_};
        lqr.ECF_LQR_Data_Update(toque_system_state);
        output_ = pitch_gravity_compensate(pos_) + lqr.ECF_LQR_Calculate() + pid_only_i.ECF_PID_Calculate(set_pos, actual_pos);
        output_ = math_.user_val_limit(output_, -7.0f, 7.0f); // 中间变量
    }

    void pitch_c::calculate_no_i(float set_pos, float actual_pos, float actual_vel) //忘记优化成一个函数了
    {
        vel_ = motor_ins.get_data_.velocity;
        pos_ = actual_pos;
        set_pos_ = set_pos;
        pos_error_ = math_.float_min_distance(set_pos, actual_pos, -90, 90.0f);
        // 力控好用
        float toque_system_state[2] = {pos_error_ / 57.295779513f, vel_};
        lqr.ECF_LQR_Data_Update(toque_system_state);
        output_ = pitch_gravity_compensate(pos_) + lqr.ECF_LQR_Calculate();
        output_ = math_.user_val_limit(output_, -7.0f, 7.0f); // 中间变量
    }

    // 随便写的临时函数
    void pitch_c::tempf_set_pos(float pos)
    {
        this->motor_ins.output_set(pos, 0, DM_kp, DM_kd, 0);
    }

    // 35 -0.6199
    const float mg_ = 16; // 13.5; // 14.73;
    // 云台力控需要的补偿的力矩
    float pitch_c::pitch_gravity_compensate(float now_picth)
    {
        float angle_diff_pitch0 = now_picth - LEVER_ARM_TO_PITCH_COF;
        float compensate = LEVER_ARM_OF_PITCH * cos(angle_diff_pitch0 / 57.295779513f) * mg_;
        return compensate;
    }
    void pitch_c::set_output()
    {
        motor_ins.output_set(0, 0, 0, 0, output_);
    }
    void pitch_c::set_output(float set_output_){
        motor_ins.output_set(0, 0, 0, 0, set_output_);
    }
    pitch_c::pitch_c(DM_Motor_n::DM_ModePrame_s *config) : motor_ins(config)
    {
    }

}
