#ifndef __ROBOT_HPP
#define __ROBOT_HPP

#ifdef __cplusplus
extern "C"
{
#endif
// #include "bsp_referee.h"
    __attribute__((noreturn)) void start_robot_task(void const *argument);
    __attribute__((noreturn)) void start_motor_task(void const *argument);
    __attribute__((noreturn)) void _UI_Task(void const *argument);
    void IMU_Task(void const *argument);
#ifdef __cplusplus
}
#endif

#include "robot_cmd.hpp"
#include "shoot.hpp"
#include "gimbal.hpp"
#include "BMI088driver.hpp"
#include "UI_Task.hpp"
void robot_init();
#endif  /*__ROBOT_H*/