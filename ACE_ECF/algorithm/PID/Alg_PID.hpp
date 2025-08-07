#ifndef __PID_ALG_HPP
#define __PID_ALG_HPP

#include "filter.hpp"
#include "user_maths.hpp"
#ifdef __cplusplus
extern "C"
{

#include <stdint.h>
#include "main.h"

#endif

#ifdef __cplusplus
}

// PID 枚举结构体
typedef enum
{
    NONE = 0X00,                      // 0000 0000 无
    Deadzone = 0x01,                  // 0000 0001 死区
    Integral_Limit = 0x02,            // 0000 0010 积分限幅
    Output_Limit = 0x04,              // 0000 0100 输出限幅
    Derivative_On_Measurement = 0x08, // 0000 1000 微分先行 TODO:
    Separated_Integral = 0x10,        // 0001 0000 积分分离
    ChangingIntegrationRate = 0x20,   // 0010 0000 变积分
    OutputFilter = 0x40,              // 0100 0000 输出滤波
    DerivativeFilter = 0x80,          // 1000 0000 微分滤波
    StepIn = 0x0100,                  // 0000 0001 0000 0000 步进式
    Trapezoid_integral = 0x0200,      // 0000 0010 0000 0000 梯形积分
    Feedforward = 0x0400,             // 0000 0100 0000 0000 前馈PID
} PID_mode_e;

namespace pid_alg_n
{
    // PID初始化结构体
    typedef struct
    {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;

        // 前馈PID系数
        float Kfa = 0;
        float Kfb = 0;

        float *ActualValueSource = NULL; // 数据来源

        uint32_t mode = 0; // pid模式
        /* 输出限幅 */
        float max_out = 0;
        /* 积分限幅 */
        float max_Ierror = 0; // 最大积分输出
        /* 微分先行 */
        float gama = 0; // 微分先行滤波系数
        /* 误差死区 */
        float deadband = 0;
        /* 积分分离 */
        float threshold_max = 0; // 积分分离最大值
        float threshold_min = 0; // 积分分离最小值
        /* 变积分 */
        float errorabsmax = 0; // 偏差绝对值最大值
        float errorabsmin = 0; // 偏差绝对值最小值
        /* 微分滤波 */
        float d_filter_num = 0;
        /* 输出滤波 */
        float out_filter_num = 0;
        /* 步进数 */
        float stepIn = 0;
    } PID_Init_Config_t;

    typedef struct // pid结构体变量
    {
        float Kp;
        float Ki;
        float Kd;

        // 前馈PID系数
        float Kfa = 0;
        float Kfb = 0;

        float SetValue; // 设定值
        float LastSetValue;
        float PerrSetValue;
        float ActualValue; // 实际值
        float LastActualValue;

        float Ierror; // 误差积累
        float Pout;
        float Iout;
        float Dout;
        float Fout;
        float out;

        float Derror; // 微分项
        float LastDerror;
        float LastLastDerror;
        float error; // 误差项
        float LastError;

        float max_out; // 最大输出

        uint32_t mode = 0; // pid模式

        /* 积分限幅 */
        float max_Ierror; // 最大积分输出
        /* 误差死区 */
        float deadband;
        /* 积分分离 */
        float threshold_max; // 积分分离最大值
        float threshold_min; // 积分分离最小值
        /* 抗积分饱和 */
        // float maximum; //最大值
        // float minimum; //最小值
        /* 变积分 */
        float errorabsmax; // 偏差绝对值最大值
        float errorabsmin; // 偏差绝对值最小值
        /* 微分先行 */
        float gama; // 微分先行滤波系数

        /* 步进数 */
        float stepIn;

    } pid_parameter_t;

    // pid算法类
    class pid_alg_c
    {

    public:
        float pid_input;
        float pid_output;

        pid_alg_c(PID_Init_Config_t pid_config);
        pid_alg_c();
        void pid_init(PID_Init_Config_t pid_config);
        void ECF_PID_CLEAR(void);
        float ECF_PID_Calculate(float SetValuew);
        float ECF_PID_Calculate(float SetValuew, float ActualValue);
        void ECF_PID_ChangeActValSource(float *ActValSource);
        void ECF_PID_Reset_StepIn(float StepIn);
        float *ECF_PID_GetActValSource(void);
        pid_parameter_t pid_measure_;

    private:
        /* 微分滤波 */
        filter_alg_n::first_order_filter_c d_filter; // 微分滤波结构体
        /* 输出滤波 */
        filter_alg_n::first_order_filter_c out_filter; // 输出滤波结构体
        float *ActualValueSource_;                     // pid 运算实际值来源
        inline void f_Separated_Integral(void);
        inline void f_Integral_Limit(void);
        inline void f_Derivative_On_Measurement(void);
        inline float Changing_Integration_Rate(void);
        inline void f_Output_Limit(void);
        inline void f_StepIn(void);
        inline void f_feedforward(void);
    };

    class cascade_pid_c // 串级PID
    {
    public:
        const float Output_Limit; // 输出限幅
        float outer_pid_output;   // 外环计算结果给内环
        float inter_pid_output;   // 最终输出结果
        pid_alg_c outer_pid;      // 外环pid
        pid_alg_c inter_pid;      // 内环pid

        const float *Target = nullptr;        // 输入值
        const float *Outer_ActualVal;         // 外环实际值指针
        const float *Inter_ActualVal;         // 内环实际值指针
        const float *SpeedFeedWard = nullptr; // 速度前馈

        cascade_pid_c(PID_Init_Config_t outer_pid_config, PID_Init_Config_t inter_pid_config, float output_limit); // 串级PID构造函数
        float Cascade_pid_Calculate(const float *SetPosVal);
        float Cascade_pid_Calculate(const float *SetPosVal, const float *outer_ActValSource, const float *inter_ActValSource);
        void Cascade_pid_Clear();
        void Cascade_SetValPoint(const float *outer_ActValSource, const float *inter_ActValSource); // 传入实际值指针
        void Cascade_SetSpeedfeedward(const float *speed);
    };

};

#endif

#endif
