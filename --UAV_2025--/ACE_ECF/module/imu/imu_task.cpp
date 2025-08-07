// #include "BMI088driver.hpp"
// #include "task.h"
// extern "C"
// {
// #include "imu_task.h"

// }

// void IMU_Task(void const *argument)
// {
//     while (1)
//     {
//         taskENTER_CRITICAL();
//         BMI088Instance_c::BMI_UpData();
//         taskEXIT_CRITICAL();
//         vTaskDelay(1);

//     }
// }