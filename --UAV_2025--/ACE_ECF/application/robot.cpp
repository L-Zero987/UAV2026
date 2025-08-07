#include "robot.hpp"
BMI088Instance_c bmi088_instance;
const static INS_t *INS;

// String_Data_t ;
void vision_task()
{
    static vision_c *vision_data = get_visiual_data();
    vision_data->vision_receive();
    if (INS != nullptr)
    {
        vision_data->vision_send_data(INS->Pitch, INS->Yaw, INS->Roll);
    }
}
// uint32_t sendtimes = 0; //测试任务次数
// FreeRTOS是以系统时钟节拍作为计量的时间单位的 ，而系统时钟节拍对应的物理时间长短于 FreeRTOSConfig.h文件中的配置项ConfigTICK_RATE_HZ有关
// 配置项 configTICK_RATE_HZ是用于配置系统时钟节拍的频率的，当前tick-> 1000hz
__attribute__((noreturn)) void
start_motor_task(void const *argument) // 不返回到调用该函数的代码处
{
    while (1)
    {
        // sendtimes++;
        DJI_Motor_n::DJIMotorControl();
        DM_Motor_n::DM_motor_control();
        vTaskDelay(1);
    }
}
__attribute__((noreturn)) void start_robot_task(void const *argument)
{
    vTaskDelay(1500); //上电先失能1秒，等待陀螺仪计算出正确的值后再启动防肘击
    GIMBAL_N::init_position();
    while (1)
    {
        vision_task(); //视觉任务直接在该任务执行即可
        ROBOT_CMD_N::robot_cmd_task();
        GIMBAL_N::gimbal_task();
        SHOOT_N::shoot_task();
        vTaskDelay(1);
    }
}

void IMU_Task(void const *argument)
{
    while (1)
    {
        taskENTER_CRITICAL();
        BMI088Instance_c::BMI_UpData();
        taskEXIT_CRITICAL();
        vTaskDelay(1);
    }
}

void robot_init()
{

    __disable_irq();
    // debug的时候想查找读取的数据直接定义应该相同的全局结构体变量，这样子就可以看见数据了
    bmi088_instance.BMI088Init(&hspi1, 1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    // 读取陀螺仪数据
    INS = bmi088_instance.Get_INS_Data_Point();
    ECF_referee_uart_init();
    vision_init();
    ROBOT_CMD_N::robot_cmd_init();
    GIMBAL_N::gimbal_init(INS, USE_IMU);
    SHOOT_N::shoot_init();
    __enable_irq();
}

__attribute__((noreturn)) void _UI_Task(void const *argument)
{
#ifdef TC_CONTROL
    // UI每秒最多发30次
    REFEREE_n::CILENT_UI_n::Cilent_UI_Task();
    while (1)
    {
        /* code */
    }
    
#endif
}
/**
 * @brief      失控处理
 * @param[in]  none
 * @retval     none
 * @attention
 */
// void out_of_control(void)
// {
//     // 将任务挂起
//     vTaskSuspend(TASK_GIMBALHandle);
//     vTaskSuspend(ShootTask_Handler);

//     // 解挂失控保护控制任务
//     //     vTaskResume(OutOf_Control_THandle);
// }

/**
 * @brief      正常 | 解除失控
 * @param[in]  none
 * @retval     none
 * @attention
 */
// void normal_control(void)
// {
//     // 解挂任务
//     vTaskResume(TASK_GIMBALHandle);
//     vTaskResume(ShootTask_Handler);

//     // 失控保护控制任务任务挂起
//     //     vTaskSuspend(OutOf_Control_THandle);
// }