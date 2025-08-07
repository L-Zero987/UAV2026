#include "shoot.hpp"
#include "robot_cmd.hpp"
namespace SHOOT_N
{
#define Encoder_To_ShootNum 36782 // 打一发// 8192*36/8 //-294257 //转一圈
#define Encoder_To_Shoot_8 294256 // 打一梭// 8192*36/8 //-294257 //转一圈
    // float loader_speed = 5000;
    static user_maths_c math_;
    /***************************************物理量转换*************************************************/
    // 射速转换(rpm->m/s)
    // static float convert_shootspd(int16_t rpm) { return (float)(rpm * MOTORRPM_TO_FIRESPEED); }
    // 射频转换(rpm->Hz)
    // static float convert_shootrat(int16_t rpm) { return (float)(rpm * MOTORRPM_TO_FIRERATE); }
    BSP_DWT_n::BSP_DWT_c *shoot_delay = nullptr;
    shoot_c *inastance = shoot_c::get_instance();
    static loader_mode_e last_loader_mode = LOADER_OFF;
    // test
    float load_speed = 0;
    float left_speed = 0, right_speed = 0;
    float add_speed1 = 0;
    float add_speed2 = 0;

    static uint16_t plunk_flag = 0; // 堵转检测
    shoot_c::shoot_c() : shoot_speed(5000), friction_speed(6050)
    {
        shoot_delay = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
    }
    void shoot_c::friction_init()
    {
        Motor_General_Def_n::Motor_Init_Config_s friction_m = {
            .controller_param_init_config = {
                .speed_PID = {
                    .Kp = 8.0f, // 12 0 3 、5，0，4.5
                    .Ki = 0.0f,
                    .Kd = 5.0f,
                    .ActualValueSource = NULL,
                    .mode = Output_Limit | Integral_Limit,
                    .max_out = 10000,
                    .max_Ierror = 100,
                    .deadband = 0,
                },
            },
            .controller_setting_init_config = {
                .outer_loop_type = Motor_General_Def_n::SPEED_LOOP, // 使用速度环
                .close_loop_type = Motor_General_Def_n::SPEED_LOOP,
                .motor_reverse_flag = Motor_General_Def_n::MOTOR_DIRECTION_REVERSE,
                .feedback_reverse_flag = Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
                .speed_feedback_source = Motor_General_Def_n::MOTOR_FEED, // 电机反馈
                                                                          //  .feedforward_flag = Motor_General_Def_n::FEEDFORWARD_NONE,
            },
            .motor_type = Motor_General_Def_n::M3508,
            .can_init_config = {
                .can_handle = &hcan2, // 使用can2
                .tx_id = 1,           // 看电调闪几下就填几
            }};
        friction_motor[0] = new DJI_Motor_n::DJI_Motor_Instance(friction_m);
        friction_m = {
            .controller_param_init_config = {
                .speed_PID = {
                    .Kp = 10.0f,
                    .Ki = 0.0f,
                    .Kd = 5.0f,
                    .ActualValueSource = NULL,
                    .mode = Output_Limit | Integral_Limit,
                    .max_out = 10000,
                    .max_Ierror = 450,
                    .deadband = 0,
                },
            },
            .controller_setting_init_config = {
                .outer_loop_type = Motor_General_Def_n::SPEED_LOOP, // 使用速度环
                .close_loop_type = Motor_General_Def_n::SPEED_LOOP,
                .motor_reverse_flag = Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
                .feedback_reverse_flag = Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
                .speed_feedback_source = Motor_General_Def_n::MOTOR_FEED, // 电机反馈
                //.feedforward_flag = Motor_General_Def_n::FEEDFORWARD_NONE,
            },
            .motor_type = Motor_General_Def_n::M3508,
            .can_init_config = {
                .can_handle = &hcan2, // 使用can2
                .tx_id = 2,           // 看电调闪几下就填几
            }};
        friction_motor[1] = new DJI_Motor_n::DJI_Motor_Instance(friction_m);
    }
    void shoot_c::loader_init()
    {
        Motor_General_Def_n::Motor_Init_Config_s loader_m = {
            .controller_param_init_config = {
                .speed_PID = {
                    .Kp = 10.0f,
                    .Ki = 0.0f,
                    .Kd = 2.0f,
                    .ActualValueSource = NULL,
                    .mode = Output_Limit | Integral_Limit,
                    .max_out = 9000,
                    .max_Ierror = 3000,
                    .deadband = 0.3,
                    // .stepIn = 3000,
                },
                .angle_PID = {
                    .Kp = 0.15f, .Ki = 0.0f, .Kd = 0.67f, .ActualValueSource = NULL, .mode = Integral_Limit, .max_out = 9000, .max_Ierror = 3000, .deadband = 0.3,
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
                .can_handle = &hcan2, // 使用can2
                .tx_id = 3,           // 看电调闪几下就填几
            },
            .zero_offset = 0,
        }; // 初始位置为0
        loader_motor = new DJI_Motor_n::DJI_Motor_Instance(loader_m);
    }

    void shoot_init()
    {
        inastance->friction_init();
        inastance->loader_init();
        inastance->cmd = ROBOT_CMD_N::get_shoot_data();
    }
    float actul = 0;
    float set_speed = 6500;
    void shoot_c::loader_on()
    {
        load_speed = loader_motor->MotorMeasure.measure.feedback_speed;
        loader_motor->DJIMotorEnable();

        switch (cmd->loader_mode) 
        {
        case LOADER_ON: // 只有摩擦轮开启时才能开启拨弹盘
            if (check_stuck())
            {
                return;
            }
            switch (cmd->shoot_num)
            {
            case 0:
            {   
                //两个速度模式
                loader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
                loader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
                switch (cmd->fire_hz)
                {
                case 0:
                    set_speed = 6500;
                    break;
                case 1:
                    set_speed = 8500;
                    break;
                default:
                    set_speed = 6500;
                    break;
                }
                // set_speed = loader_speed;

                loader_motor->DJIMotorSetRef(set_speed); // 6000
                plunk_flag = 0;
                break;
            }

            case 1:
                loader_on_pos(1, 1);
                break;
            case 8: // 有间隔的连发
                loader_on_pos(1, 4);
                break;
            default:
                break;
            }
            break;
            // pShoot->loader_motor->DJIMotorSetRef(fire_bulck_check()*6000);
            // last_loader_mode = LOADER_ON_FORWARD;
            /***************************************************************/
            /**
             *卡弹回退： 检测转矩电流与设定速度，当转矩电流>9000且设定速度>0，给个 周期时间
             *           只要设定的周期时间不等于0，就给个反向速度，设定一次减一次，直到为0
             */
            // 测试拨蛋盘用
        case LOADER_ON_REVERSE:
            loader_motor->DJIMotorSetRef(-4500);
            last_loader_mode = LOADER_ON_REVERSE;
            break;
        case LOADER_OFF:
            loader_motor->motor_settings.close_loop_type = Motor_General_Def_n::OPEN_LOOP;
            loader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::OPEN_LOOP;
            loader_motor->DJIMotorSetRef(0);
            last_loader_mode = LOADER_OFF;
            break;
        default:
            loader_motor->DJIMotorSetRef(0);
            break;
        }
    }

    //num 弹数 freq 弹频
    void shoot_c::loader_on_pos(uint8_t num, float freq)
    {
        // last_loader_mode = LOADER_ON_SPEED;
        actul = loader_motor->MotorMeasure.measure.record_ecd;
        // last_loader_mode = LOADER_ON_SPEED;
        static float Last_Shoot_Time = 0;
        // 到位检测
        if (loader_motor->MotorMeasure.measure.record_ecd >= Encoder_To_ShootNum * num)
        {
            loader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
            loader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
            loader_motor->DJIMotorSetRef(0);
        }
        if (shoot_delay->ECF_DWT_GetTimeline_s() - Last_Shoot_Time > (1.0f / freq))
        {
            loader_motor->MotorMeasure.measure.record_ecd = 0;
            now_ecd += Encoder_To_ShootNum;
            Last_Shoot_Time = shoot_delay->ECF_DWT_GetTimeline_s();
            loader_motor->motor_settings.close_loop_type = Motor_General_Def_n::ANGLE_AND_SPEED_LOOP;
            loader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::ANGLE_LOOP;
            plunk_flag = 0;
            loader_motor->DJIMotorSetRef(loader_motor->MotorMeasure.measure.feedback_ecd + num * Encoder_To_ShootNum);
        }
    }
    void shoot_c::friction_on()
    {

        DISABLE_cnt = 0;
        friction_motor[0]->DJIMotorEnable();
        friction_motor[1]->DJIMotorEnable();
        switch (cmd->friction_mode)
        {
        case FRICTION_OFF:
            friction_motor[0]->DJIMotorSetRef(0);
            friction_motor[1]->DJIMotorSetRef(0);
            break;
        case FRICTION_ON:
            // test
            left_speed = -friction_motor[0]->MotorMeasure.measure.feedback_speed;
            right_speed = friction_motor[1]->MotorMeasure.measure.feedback_speed;
            static float fric_speed = math_.user_val_limit(friction_speed + cmd->firction_compansate, 5400, 6900);
            friction_motor[0]->DJIMotorSetRef(fric_speed + add_speed1);
            friction_motor[1]->DJIMotorSetRef(fric_speed + add_speed2);
            break;
        default:
            friction_motor[0]->DJIMotorSetRef(0);
            friction_motor[1]->DJIMotorSetRef(0);
            break;
        }
    }

    void shoot_c::fire_on()
    {
        friction_on();
        loader_on();
    }
    void shoot_c::fire_off()
    {
        loader_motor->motor_settings.close_loop_type = Motor_General_Def_n::SPEED_LOOP;
        loader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::SPEED_LOOP;
        if (DISABLE_cnt < 3000)
        {
            friction_motor[0]->DJIMotorSetRef(0);
            friction_motor[1]->DJIMotorSetRef(0);
            loader_motor->DJIMotorSetRef(0);
            DISABLE_cnt++;
        }
        else
        {
            friction_motor[0]->DJIMotorStop();
            friction_motor[1]->DJIMotorStop();
            loader_motor->DJIMotorStop();
        }
    }

    void shoot_task()
    {
        if (inastance->cmd->shoot_mode == SHOOT_ON)
        {
            inastance->fire_on();
        }
        else
        {
            inastance->fire_off();
        }
    }
    bool shoot_c::check_stuck()
    {
        if (abs(loader_motor->MotorMeasure.measure.feedback_real_current) > 7900)
        {
            plunk_flag = 220;
        }
        if (plunk_flag != 0)
        {
            loader_motor->motor_settings.close_loop_type = Motor_General_Def_n::OPEN_LOOP;
            loader_motor->motor_settings.outer_loop_type = Motor_General_Def_n::OPEN_LOOP;
            plunk_flag--;
            loader_is_blocked = 1;
            loader_motor->DJIMotorSetRef(-1000);
            return true;
        }
        return false;
    }
    void shoot_c::speed_cheak(float cheak_speed)
    {
        if (cheak_speed > LIMIT_SPEED)
        {
            cmd->firction_compansate -= 50;
        }
    }
    void motor_clear_ecd(DJI_Motor_n::DJIMotorMeasure_t *measure)
    {
        measure->total_angle = 0;
        measure->relative_angle = 0;
        measure->angle_single_round = 0;
        measure->record_ecd = 0;
        measure->total_round = 0;
    }
    shoot_c *shoot_c::get_instance()
    {
        static shoot_c instance;
        return &instance;
    }

}
