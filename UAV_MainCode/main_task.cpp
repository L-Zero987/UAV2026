#include "main_task.hpp"
#include "debug_task.hpp"
#include "fire.hpp"
#include "gimbal.hpp"
#include "robot_cmd.hpp"
#include "Config.hpp"

extern "C"
{
#include "FreeRTOS.h"
#include "task.h"
}

/* region ***************** 全局实例指针定义 ******************/
#if (IS_DEBUG_MODE == 0)

 Fire_n::Fire_c* fire_instance;
 Gimbal_n::Gimbal_c* gimbal_instance;

#else

 Debug_n::Debug_c* debug_instance;

#endif
// endregion


/* region ********************* 任务 **********************/
/**
 * @brief 初始化函数，仅调用该函数完成所有自定义初始化
 *
 */
void MainTask_Init()
{
    HAL_Delay(2048);
#if (IS_DEBUG_MODE == 0)
    fire_instance = Fire_n::Fire_c::Get_InstancePtr();
    gimbal_instance = Gimbal_n::Gimbal_c::Get_InstancePtr();
#else
    debug_instance = new Debug_n::Debug_c();

#endif
}

/**
 * @brief 主要循环函数，优先级最高，1ms执行
 *
 */
void MainLoop_Task(void const * argument)
{
    while (1)
    {
#if (IS_DEBUG_MODE == 0)
        RobotCMD_n::StateLoop();
#else
        debug_instance->Loop1();
#endif
        vTaskDelay(1);
    }
}

/**
 * @brief 电机发送任务，优先级中等，2ms执行
 *
 */
void MotorLoop_Task(void const * argument)
{
    while (1)
    {
#if (IS_DEBUG_MODE == 0)
        DJI_Motor_n::DJIMotorControl();
        gimbal_instance->pitch_motor->Transmit();
#else

#endif
        vTaskDelay(2);
    }
}

/**
 * @brief 状态循环任务，优先级最低，1ms执行
 *
 */
void StateLoop_Task(void const * argument)
{
    while(1)
    {
#if (IS_DEBUG_MODE == 0)
        Gimbal_n::StateLoop();
        Fire_n::StateLoop();
#else

#endif
        vTaskDelay(1);
    }
}
// endregion