#ifndef __GIMBAL_HPP
#define __GIMBAL_HPP

#include "dji_motor.hpp"
#include "dm_mit_mode.hpp"
#include "BMI088driver.hpp"
#include "bsp_dwt.hpp"
#include "robot_cmd.hpp"

namespace Gimbal_n
{
    typedef enum
    {
        Disable            = 0x00, // 失能
        ControlWithEncoder = 0x01, // 编码器的手动控制
        ControlWithIMU     = 0x02, // 陀螺仪的手动控制
        AutoControl        = 0x03, // 自瞄
    } Gimbal_State_e;

    class Gimbal_c
    {
    public:
        /* instance ptr */
        DJI_Motor_n::DJI_Motor_Instance *yaw_motor;
        DM_Motor_n::DM_Mit_Mode_c       *pitch_motor;
        pid_alg_n::pid_alg_c            *pitch_angle_pid;
        BMI088Instance_c                *imu;
        const INS_t                     *imu_date;
        // 看情况加滤波

        /* function */
        static Gimbal_c* Get_InstancePtr(void);
        void ChangeState(Gimbal_State_e _new_state);

        /* Loop */
        friend void StateLoop(void);

    private:
        /* state */
        Gimbal_State_e current_state = Disable;

        float actual_yaw   = 0.0f;
        float actual_pitch = 0.0f;
        float target_yaw   = 0.0f;
        float target_pitch = 0.0f;

        /* flag */
        bool is_loop = false;

        /* command */
        RobotCMD_n::RobotCMD_c* cmd_instance;

        /* timer */
        float timer_delta_t                  = 0.0f;
        uint32_t timer_cnt                   = 0;
        BSP_DWT_n::BSP_DWT_c *timer_instance = nullptr;

        /* class */
        Gimbal_c();

        /* function */
        void Yaw_Init(void);
        void Pitch_Init(void);
        void IMU_Init(void);
        void Change_PIDWithIMU(void);
        void Change_PIDWithEncoder(void);
        void Yaw_AngleLimit(void);
        void Yaw_SetOutput_Encoder(void);
        void Yaw_SetOutput_IMU(void);
        void Pitch_AngleLimit(void);
        void Pitch_SetOutput_Encoder(void);
        void Pitch_SetOutput_IMU(void);
        void Update_ActualAngle(void);

        void StateStart(void);
        void StateExit(void);

    };
    void StateLoop(void);
}

#endif //! __GIMBAL_HPP
