#ifndef __GIMBAL_HPP
#define __GIMBAL_HPP
#include "robot_def.hpp"
#include "BMI088driver.hpp"

namespace GIMBAL_N
{
#define USE_IMU 1
#define NOT_USE_IMU 0

    class yaw_c
    {
    public:
        lqr_alg_n::lqr_alg_c lqr;
        pid_alg_n::pid_alg_c pid_only_i;
        DJI_Motor_n::DJI_Motor_Instance motor_ins; // 电机实例
        yaw_c(Motor_General_Def_n::Motor_Init_Config_s config);
        void init();
        void set_pos(float pos) { this->set_pos_ = pos; }
        void enable();
        void disable();
        void calculate(float set_pos, float actual_pos, float actual_vel); // 输入设定值和实际值,计算输出
        void calculate_no_i(float set_position, float actual_pos, float actual_vel);
        void set_output();                                                 // 做限幅后将计算的output传给电机
        void set_output(float set_output_);                                // 将计算的output传给电机
    protected:
        float set_pos_ = 0; // 一般是设置的目标值
        float pos_error_ = 0;
        float pos_ = 0; // 当前位置
        float vel_ = 0; // 当前速度
        float output_ = 0;
    };

    class pitch_c // 继承电机类，DM4310 采用力控
    { // (O,o)? 继承了啥？
    public:
        lqr_alg_n::lqr_alg_c lqr;
        pid_alg_n::pid_alg_c pid_only_i;
        DM_Motor_n::DM_Mit_Mode_c motor_ins; // 电机实例
        pitch_c(DM_Motor_n::DM_ModePrame_s *config);
        void init();
        void set_pos(float pos) { this->set_pos_ = pos; }
        void enable();
        void disable();
        void calculate(float set_pos, float actual_pos, float actual_vel); // 输入设定值和实际值,计算输出
        void calculate_no_i(float set_position, float actual_pos, float actual_vel);
        void set_output();                                                 // 做限幅后将计算的output传给电机
        void set_output(float set_output_);                                // 将计算的output传给电机
        float pitch_gravity_compensate(float now_picth);
        void tempf_set_pos(float pos);
        // todo:加pid编码值位置环
    protected:
        float set_pos_ = 0; // 一般是设置的目标值
        float pos_error_ = 0;
        float pos_ = 0; // 当前位置
        float vel_ = 0; // 当前速度
        float output_ = 0;
    };
    class gimbal_c
    {
    public:
        gimbal_c();
        ~gimbal_c();
        // 获取唯一实例的静态方法
        static gimbal_c *get_instance();
        gimbal_control_t *config = nullptr;
        const INS_t *imu_data = nullptr;
        bool init_flag = false;       // 初始标志
        bool get_center_flag = false; // 标志
        /*****************yaw************* */
        yaw_c *yaw_m;
        pitch_c *pitch_m;
        void yaw_move();
        void yaw_only_ecd();
        void yaw_angle_limit();
        filter_alg_n::Recursive_ave_filter_type_t yaw_filter;
        filter_alg_n::sliding_mean_filter_c yaw_slfilter;
        /*****************pitch*****************/
        filter_alg_n::sliding_mean_filter_c pitch_slfilter;
        // filter_alg_n::first_order_filter_c pitch_firstOrder;
        // filter_alg_n::NotchFilter pitch_notch;
        filter_alg_n::Recursive_ave_filter_type_t pitch_filter;
        void pitch_move();
        void pitch_angle_limit();
        void picth_only_ecd();
        void position_updata();
        bool fire_check();
        void get_center_position();
        void motor_init();

    protected:
        static gimbal_c *instance;
        uint8_t filter_num;
        float actual_pitch = 0;
        float actual_yaw = 0;
    };
    void gimbal_init(const INS_t *pINS, uint8_t use_IMU);
    float torque_to_voltage_6020(float torque);
    void init_position();
    void gimbal_task(void);
}

#endif /*__GIMBAL_H*/