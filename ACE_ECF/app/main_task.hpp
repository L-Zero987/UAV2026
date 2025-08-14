#ifndef __MAIN_TASK_HPP
#define __MAIN_TASK_HPP

#ifdef __cplusplus
extern "C"
{
#endif

 void MainTask_Init(void);
 void MainLoop_Task(void const * argument);
 void MotorLoop_Task(void const * argument);
 void StateLoop_Task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif //! __MAIN_TASK_HPP
