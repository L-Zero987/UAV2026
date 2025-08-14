#include "main_task.hpp"
#include "debug_task.hpp"
#include "fire.hpp"
#include "Config.hpp"

extern "C"
{
#include "FreeRTOS.h"
#include "task.h"
}

/****************** 全局实例指针定义 ******************/
#if (IS_DEBUG_MODE == 0)

 Fire_n::Fire_c* fire_instance;

#else

 Debug_n::Debug_c* debug_instance;

#endif
// 全局实例指针定义end


/********************** 任务 **********************/
/**
 * @brief 初始化函数，仅调用该函数完成所有自定义初始化
 *
 */
void MainTask_Init()
{
#if (IS_DEBUG_MODE == 0)
    fire_instance = new Fire_n::Fire_c();

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
#else
        DJI_Motor_n::DJIMotorControl();
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
        fire_instance->StateLoop();
#else

#endif
        vTaskDelay(1);
    }
}
// 任务end