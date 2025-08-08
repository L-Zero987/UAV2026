//没用到
// #ifndef _ROBOT_TASK_HPP
// #define _ROBOT_TASK_HPP

// #ifdef __cplusplus
// extern "C"
// {
// #include "FreeRTOS.h"
// #include "task.h"
// #include "main.h"
// #include "cmsis_os.h"
// #include "safe_task.h"

// #include "robot_cmd.hpp"
// #include "shoot.hpp"
// #include "gimbal.hpp"
// #include "DJI_motor.hpp"
// #endif
// #ifdef __cplusplus

// }

// #endif
// namespace APP_n
// {

//     // __attribute__((noreturn)) void
//     // start_motor_task(void const *argument) // 不返回到调用该函数的代码处
//     // {
//     //     while (1)
//     //     {
//     //         DJI_Motor_n::DJIMotorControl();
//     //         vTaskDelay(10);
//     //     }
//     // }

//     extern "C"
//     {
//         osThreadId motor_task_handle;
//         osThreadId imu_task_handle;
//         osThreadId robot_task_handle;
//         osThreadId safe_task_handle;
//         void task_init() // 任务创建
//         {
//             // osThreadDef(motortask, start_motor_task, osPriorityNormal, 0, 256);
//             // motor_task_handle = osThreadCreate(osThread(motortask), NULL);
//             char *robot = "robot_task";
//             char *safe = "SAFE_TASK";
//             osThreadDef(robot, start_robot_task, osPriorityNormal, 0, 1250);
//             robot_task_handle = osThreadCreate(osThread(robot), NULL);

//             osThreadDef(safe, safe_task, osPriorityHigh, 0, 128);
//             safe_task_handle = osThreadCreate(osThread(safe), NULL);
//         }
//         __attribute__((noreturn)) void start_robot_task(void const *argument)
//         {
//             while (1)
//             {
//                 robot_cmd_task();
//                 gimbal_task();
//                 shoot_task();
//                 vTaskDelay(1);
//             }
//         }
//     }

//     }

//#endif /*_ROBOT_TASK_H*/


