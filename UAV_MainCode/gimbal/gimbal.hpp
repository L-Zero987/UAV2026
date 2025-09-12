#ifndef __GIMBAL_HPP
#define __GIMBAL_HPP

#include "Config.hpp"
#include "dji_motor.hpp"
#include "dm_mit_mode.hpp"
#include "BMI088driver.hpp"
#include "bsp_dwt.hpp"
#include "robot_cmd.hpp"
#include "lqr_alg.hpp"

namespace Gimbal_n
{
    typedef enum
    {
        Disable            = 0x00, // 失能
        ControlWithEncoder = 0x01, // 编码器的手动控制
        ControlWithIMU     = 0x02, // 陀螺仪的手动控制
        AutoControl        = 0x03, // 自瞄
    } Gimbal_State_e;

    class Yaw_c
    {
    public:
        DJI_Motor_n::DJI_Motor_Instance *motor;
        pid_alg_n::pid_alg_c            *pid_i;     // 用于lqr
        lqr_alg_n::lqr_alg_c            *lqr;
        float k_lqr[2] = {YAW_MOTOR_LQR_1, YAW_MOTOR_LQR_2};

        static Yaw_c* Get_InstancePtr(void);

        void AngleLimit(void);
        void SetOutput_Encoder(void);
        void SetOutput_IMU(void);

    private:
        float set_pos_ = 0; // 一般是设置的目标值 // (O,o)! 可以改成指针
        float pos_error_ = 0;
        float pos_ = 0; // 当前位置
        float vel_ = 0; // 当前速度
        float output_ = 0;

        Yaw_c();

        void Motor_Init(void);
        void Model_Init(void);
    };

    class Pitch_c
    {
    public:
        DM_Motor_n::DM_Mit_Mode_c       *motor;
        pid_alg_n::pid_alg_c            *pid_angle;
        pid_alg_n::pid_alg_c            *pid_i;   // 用于lqr
        lqr_alg_n::lqr_alg_c            *lqr;
        float k_lqr[2] = {PITCH_MOTOR_LQR_1, PITCH_MOTOR_LQR_2};

        static Pitch_c* Get_InstancePtr(void);

        void AngleLimit(void);
        void SetOutput_Encoder(void);
        void SetOutput_IMU(void);

    private:
        float set_pos_ = 0; // 一般是设置的目标值
        float pos_error_ = 0;
        float pos_ = 0; // 当前位置
        float vel_ = 0; // 当前速度
        float output_ = 0;

        Pitch_c();

        void Motor_Init(void);
        void Model_Init(void);
    };

    class Gimbal_c
    {
    public:
        /* instance ptr */
        Yaw_c               *yaw;
        Pitch_c             *pitch;
        BMI088Instance_c    *imu;
        const INS_t         *imu_date;

        /* ctrl_date */
        float actual_yaw   = 0.0f;
        float actual_pitch = 0.0f;
        float target_yaw   = 0.0f;
        float target_pitch = 0.0f;

        // 看情况加滤波

        /* function */
        static Gimbal_c* Get_InstancePtr(void);
        void ChangeState(Gimbal_State_e _new_state);

        /* Loop */
        friend void StateLoop(void);

    private:
        /* state */
        Gimbal_State_e current_state = Disable;

        /* flag */
        bool is_loop = false;

        /* ctrl */
        RobotCMD_n::RobotCMD_c* cmd_instance;

        /* timer */
        float timer_delta_t                  = 0.0f;
        uint32_t timer_cnt                   = 0;
        BSP_DWT_n::BSP_DWT_c *timer_instance = nullptr;

        /* class */
        Gimbal_c();

        /* function */
        void Motor_Init(void);
        void IMU_Init(void);

        void Change_PIDWithIMU(void);
        void Change_PIDWithEncoder(void);

        void Update_ActualAngle(void);

        void CtrlMove_DR16(void);
        void CtrlMove_TC(void);
        void CtrlMove_Auto(void);

        void StateStart(void);
        void StateExit(void);

    };
    void StateLoop(void);
}

#endif //! __GIMBAL_HPP
